#include <Arduino.h>
#include <algorithm>
#include "max6675.h"
#include <RBDdimmer.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

// === CONFIGURACIÓN WiFi y Telegram ===
const char* ssid = "<Nombre de tu red WiFi>";
const char* password = "<Contraseña de tu red WiFi>";
const char* botToken = "<Token de tu bot>";
const int64_t chatId = <Tu ID de usuario>;

WiFiClientSecure client;
UniversalTelegramBot bot(botToken, client);

// === Pines y Hardware ===
int thermoDO = 19, thermoCS = 23, thermoCLK = 5;
const int zeroCrossPin = 25, acdPin = 27, ventiPin = 32;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
dimmerLamp acd(acdPin, zeroCrossPin);

// === PID ===
double temperaturaMedida = 0.0, potenciaSalida = 0.0;
double temperaturaSet = 180, temperaturaObjetivo = 180;
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
PID controladorPID(&temperaturaMedida, &potenciaSalida, &temperaturaSet, Kp, Ki, Kd, DIRECT);

// === Telegram ===
unsigned long lastBotCheck = 0;
const unsigned long botInterval = 2000;

// === Estado del Horno ===
enum EstadoHorno { APAGADO, CONFIG_TEMP, CONFIG_TIEMPO, CALENTANDO, MANTENIENDO };
EstadoHorno estado = APAGADO;
bool hornoEncendido = false;
bool objetivoAlcanzado = false;

unsigned long tiempoInicio = 0;
unsigned long duracionMinutos = 0;
unsigned long tiempoObjetivoAlcanzado = 0;
bool tiempoIndefinido = false;
bool aviso1MinHecho = false;

// === Lectura Temperatura (con suavizado) ===
const int numLecturas = 10;
float lecturas[numLecturas];
int indiceLectura = 0;
bool lleno = false;

float redondearA025(float t) {
  return round(t / 0.25) * 0.25;
}
float leerTemperaturaFiltrada() {
  float nueva = thermocouple.readCelsius();
  lecturas[indiceLectura] = nueva;
  indiceLectura = (indiceLectura + 1) % numLecturas;
  if (indiceLectura == 0) lleno = true;

  int tamano = lleno ? numLecturas : indiceLectura;
  float temp[tamano];
  memcpy(temp, lecturas, sizeof(float) * tamano);
  std::sort(temp, temp + tamano);
  float suma = 0;
  for (int i = 0; i < tamano; i++) suma += temp[i];
  return redondearA025(suma / tamano);
}

// === PID y Control ===
void actualizarPID() {
  controladorPID.Compute();
  potenciaSalida = constrain(potenciaSalida, 10, 100);
  acd.setPower((int)potenciaSalida);
}

void manejarMensajesTelegram() {
  int numMensajes = bot.getUpdates(bot.last_message_received + 1);
  while (numMensajes) {
    for (int i = 0; i < numMensajes; i++) {
      String msg = bot.messages[i].text;
      msg.trim();

      if (msg == "Horno ENCENDIDO") {
        hornoEncendido = true;
        estado = CONFIG_TEMP;
        digitalWrite(ventiPin, LOW);
        bot.sendMessage(String(chatId), "Horno encendido. Ingrese temperatura deseada (°C, entre 0 y 300):", "");
      }
      else if (estado == CONFIG_TEMP && msg.toInt() > 0 && msg.toInt() <= 300) {
        temperaturaObjetivo = msg.toInt();
        temperaturaSet = temperaturaObjetivo;
        estado = CONFIG_TIEMPO;
        bot.sendMessage(String(chatId), "¿Por cuánto tiempo desea mantener esta temperatura?\nResponda con 1-60 minutos o 'INDEFINIDO'", "");
      }
      else if (estado == CONFIG_TIEMPO) {
        if (msg == "INDEFINIDO") {
          tiempoIndefinido = true;
          estado = CALENTANDO;
          bot.sendMessage(String(chatId), "Modo indefinido. Esperando alcanzar temperatura objetivo...", "");
        } else {
          int t = msg.toInt();
          if (t >= 1 && t <= 60) {
            duracionMinutos = t;
            tiempoIndefinido = false;
            estado = CALENTANDO;
            bot.sendMessage(String(chatId), "Duración registrada: " + String(duracionMinutos) + " min. Esperando alcanzar temperatura...", "");
          }
        }
      }
      else if (msg.startsWith("Extender tiempo: ")) {
        int extra = msg.substring(17).toInt();
        if (estado == MANTENIENDO && extra > 0) {
          duracionMinutos += extra;
          aviso1MinHecho = false;
          bot.sendMessage(String(chatId), "Tiempo extendido " + String(extra) + " minutos.", "");
        }
      }
      else if (msg == "Horno APAGADO") {
        hornoEncendido = false;
        estado = APAGADO;
        acd.setPower(0);
        digitalWrite(ventiPin, HIGH);
        bot.sendMessage(String(chatId), "Horno apagado.", "");
      }
      else if (msg == "Estado") {
        String info = "Temp actual: " + String(temperaturaMedida) + " °C\n";
        if (estado == MANTENIENDO) {
          unsigned long transcurrido = (millis() - tiempoObjetivoAlcanzado) / 60000;
          info += "Tiempo transcurrido: " + String(transcurrido) + " min\n";
          if (!tiempoIndefinido) {
            unsigned long restante = duracionMinutos - transcurrido;
            info += "Tiempo restante: " + String(restante) + " min\n";
          }
        }
        bot.sendMessage(String(chatId), info, "");
      }
    }
    numMensajes = bot.getUpdates(bot.last_message_received + 1);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(ventiPin, OUTPUT);
  acd.begin(NORMAL_MODE, ON);
  controladorPID.SetMode(AUTOMATIC);
  controladorPID.SetOutputLimits(10, 100);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  client.setInsecure();
  bot.sendMessage(String(chatId), "Horno listo para recibir comandos.", "");
}

void loop() {
  temperaturaMedida = leerTemperaturaFiltrada();

  if (millis() - lastBotCheck > botInterval) {
    manejarMensajesTelegram();
    lastBotCheck = millis();
  }

  if (!hornoEncendido) return;

  switch (estado) {
    case CALENTANDO:
      actualizarPID();
      if (!objetivoAlcanzado && abs(temperaturaMedida - temperaturaObjetivo) <= 2.0) {
        objetivoAlcanzado = true;
        tiempoObjetivoAlcanzado = millis();
        if (tiempoIndefinido) estado = MANTENIENDO;
        else estado = MANTENIENDO;
        bot.sendMessage(String(chatId), "Temperatura objetivo alcanzada. Manteniendo...", "");
      }
      break;

    case MANTENIENDO:
      actualizarPID();
      if (!tiempoIndefinido) {
        unsigned long transcurrido = (millis() - tiempoObjetivoAlcanzado) / 60000;
        unsigned long restante = duracionMinutos - transcurrido;
        if (restante <= 1 && !aviso1MinHecho) {
          bot.sendMessage(String(chatId), "Falta 1 minuto. ¿Desea extender el tiempo? Responda: Extender tiempo: XX", "");
          aviso1MinHecho = true;
        }
        if (transcurrido >= duracionMinutos) {
          estado = APAGADO;
          hornoEncendido = false;
          acd.setPower(0);
          bot.sendMessage(String(chatId), "Tiempo finalizado. Horno apagado.", "");
        }
      }
      break;

    default:
      break;
  }

  delay(500);
}
