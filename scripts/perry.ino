#include <Arduino.h>

#include <algorithm>
#include "max6675.h"
#include <RBDdimmer.h>
#include <PID_v1.h>

// BOT TELEGRAM
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

// Configuración WiFi
const char* ssid = "SSID de red wifi";
const char* password = "contrasena de la red";

// Token y chat del bot
#define BOT_TOKEN "Token_del_bot"
#define CHAT_ID "ID_del_Chat"

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

// LED
const int ledPin = 2;
bool ledState = false;

// Estados
bool esperandoTiempo = false;
bool esperandoTemperatura = false;
bool esperandoTemperaturaTemporizada = false;
bool enModoSoldadura = false;
bool confirmacionApagadoSoldadura = false;
unsigned long apagarEn = 0;
unsigned long tiempoInicio = 0;
unsigned long tiempoEtapaInicio = 0;
int temperatura = 0;
int tiempoTemporizado = 0;
int etapaSoldadura = 0;

// Recordatorio
unsigned long ultimoRecordatorio = 0;
const unsigned long intervaloRecordatorio = 5 * 60 * 1000; // 5 minutos

// Revisión de mensajes
unsigned long lastCheck = 0;
const unsigned long checkInterval = 1000;

// Log de eventos
#define MAX_LOGS 50
String logs[MAX_LOGS];
int logIndex = 0;

// Agregar entrada al log
void agregarLog(String mensaje) {
  String tiempo = "[" + String(millis() / 1000) + " s] ";
  String entrada = tiempo + mensaje;
  Serial.println(entrada);
  if (logIndex < MAX_LOGS) {
    logs[logIndex++] = entrada;
  }
}

// Teclado
void sendKeyboard(String chat_id) {
  String keyboardJson = "["
    "[{ \"text\" : \"Encender 🔥\", \"callback_data\" : \"on\" },"
     "{ \"text\" : \"Apagar ❌\", \"callback_data\" : \"off\" }],"
    "[{ \"text\" : \"Estado 📊\", \"callback_data\" : \"estado\" },"
     "{ \"text\" : \"Temporizador ⏱\", \"callback_data\" : \"temporizador\" }],"
    "[{ \"text\" : \"Modo Soldadura 🧪\", \"callback_data\" : \"soldadura\" }],"
    "[{ \"text\" : \"Logs 📃\", \"callback_data\" : \"logs\" }]"
  "]";
  bot.sendMessageWithInlineKeyboard(chat_id, "Selecciona una opción:", "", keyboardJson);
}

void iniciarModoSoldadura(String chat_id) {
  enModoSoldadura = true;
  etapaSoldadura = 1;
  temperatura = 100;
  tiempoEtapaInicio = millis();
  digitalWrite(ledPin, HIGH);
  ledState = true;
  agregarLog("Modo soldadura iniciado: Precalentamiento a 100 °C");
  bot.sendMessage(chat_id, "🧪 Modo soldadura iniciado: Precalentamiento a 100 °C por 90 segundos.", "");
  sendKeyboard(chat_id);
}

void terminarSoldadura(String chat_id) {
  enModoSoldadura = false;
  etapaSoldadura = 0;
  digitalWrite(ledPin, LOW);
  ledState = false;
  temperatura = 0;
  agregarLog("Modo soldadura cancelado o terminado");
  bot.sendMessage(chat_id, "🧪 Modo soldadura finalizado.", "");
  sendKeyboard(chat_id);
}



// CONTROL PID

int thermoDO = 32;
int thermoCS = 12;
int thermoCLK = 13;

const int zeroCrossPin  = 25;
const int acdPin  = 33;

// Motor A
int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;

// Configuración de propiedades PWM
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 225;

int power = 50;

// Definir variables del PID

double temperaturaSet = 10;  // Temperatura deseada en °C
double temperaturaMedida = 0.0; // Lectura del sensor
double potenciaSalida = 0.0;    // Potencia aplicada al dimmer

double Kp_base = 2.0, Ki_base = 0.5, Kd_base = 1.0;  
double Kp, Ki, Kd;
double errorAnterior = 0;

// Límites de las ganancias
double Kp_min = 0.4,  Kp_max = 30.0;
double Ki_min = 0.01, Ki_max = 20.0;
double Kd_min = 0.1,  Kd_max = 10.0;

// Variables auxiliares
double FF_GAIN = 1.0;  // Ganancia del feedforward

// Crear el controlador PID
double temperaturaPid = temperaturaSet - 13;
PID controladorPID(&temperaturaMedida, &potenciaSalida, &temperaturaPid, Kp, Ki, Kd, DIRECT);

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
dimmerLamp acd(acdPin,zeroCrossPin);

const int numLecturas = 10;  // Tamaño de la ventana de filtrado
float lecturas[numLecturas];  
int indice = 0;
bool lleno = false;  // Indica si ya se llenó el buffer una vez

float redondearA025(float temperatura) {
  return round(temperatura / 0.25) * 0.25;
}

// Función para leer temperatura con filtrado y redondeo
float leerTemperaturaFiltrada() {
  // Leer temperatura del sensor
  float nuevaLectura = thermocouple.readCelsius();

  // Guardar en el buffer de lecturas
  lecturas[indice] = nuevaLectura;
  indice = (indice + 1) % numLecturas;
  if (indice == 0) lleno = true;

  // No aplicar filtro hasta que el buffer esté lleno
  int tamanoValido = lleno ? numLecturas : indice;

  // Copiar valores a un array temporal para ordenarlos
  float temp[tamanoValido];
  memcpy(temp, lecturas, sizeof(float) * tamanoValido);
  std::sort(temp, temp + tamanoValido);

  // Obtener la mediana
  float mediana;
  if (tamanoValido % 2 == 0)
      mediana = (temp[tamanoValido / 2 - 1] + temp[tamanoValido / 2]) / 2.0;
  else
      mediana = temp[tamanoValido / 2];

  // Calcular el promedio móvil
  float suma = 0;
  for (int i = 0; i < tamanoValido; i++) {
      suma += temp[i];
  }
  float temperaturaSuavizada = suma / tamanoValido;

  // Redondear al múltiplo de 0.25 más cercano
  return redondearA025(temperaturaSuavizada);
}

float ultimaTemperatura = -100.0;  // Inicializamos con un valor imposible
const float histeresis = 0.50;  // Margen de cambio mínimo permitido

float leerTemperaturaSuavizada() {
    // Leer y filtrar temperatura
    float temperatura = redondearA025(leerTemperaturaFiltrada());  

    // Aplicar histéresis: solo actualizar si hay un cambio significativo
    if (abs(temperatura - ultimaTemperatura) >= histeresis || ultimaTemperatura == -100.0) {
        ultimaTemperatura = temperatura;
    }

    return ultimaTemperatura;
}

void actualizarPID() {
  double error = temperaturaSet - temperaturaMedida;
  double dError = error - errorAnterior;  // Derivada del error

  double feedforward = (temperaturaSet - temperaturaMedida) * FF_GAIN;

  if (abs(error) > 15) {
      Kp = Kp_base * 1.5;
  } else {
      Kp = Kp_base * 0.8;
  }
  Kp_base = constrain(Kp, Kp_min, Kp_max);

  if (abs(error) < 10) {
      Ki = Ki_base * 2.0;
  } else {
      Ki = Ki_base;
  }
  Ki_base = constrain(Ki, Ki_min, Ki_max);

  if (dError > 4) {  
      Kd *= 1.5;
  }
  else{
    Kd = Kd_base;
  }
  Kd_base = constrain(Kd, Kd_min, Kd_max);

  // Aplicar los nuevos valores al PID
  controladorPID.SetTunings(Kp_base, Ki_base, Kd_base);
  errorAnterior = error;
  controladorPID.Compute();
  potenciaSalida += feedforward;

  // Limitar la salida para evitar valores fuera de rango
  potenciaSalida = constrain(potenciaSalida, 10, 100);
}

// Nueva función para cambiar la temperatura deseada
void setTemperaturaSet(double nuevaTemp) {
  temperaturaSet = nuevaTemp;
}

// Nueva función que contiene toda la lógica del loop
void controlHorno() {
  temperaturaMedida = leerTemperaturaSuavizada();
  actualizarPID();
  power = round(potenciaSalida);
  acd.setPower(power);

  if (temperaturaMedida > temperaturaSet) {
    digitalWrite(enable1Pin, HIGH);  // Aplicar PWM
  } else {
    digitalWrite(enable1Pin, LOW);
  }

  Serial.print("C = ");
  Serial.print(temperaturaMedida);
  Serial.print(" -- POWER = ");
  Serial.print(power);
  Serial.print(" -- Kp: ");
  Serial.print(Kp);
  Serial.print(" -- Ki: ");
  Serial.print(Ki);
  Serial.print(" -- Kd: ");
  Serial.println(Kd);
}

void setup() {
  Serial.begin(115200);
  acd.begin(NORMAL_MODE, ON);

  controladorPID.SetMode(AUTOMATIC);
  controladorPID.SetOutputLimits(10, 100);  // Rango de control del dimmer (10-100%)

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  delay(500);

  // BOT TELEGRAM
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado");

  client.setInsecure();
  sendKeyboard(CHAT_ID);
}


void loop() {
  if (millis() - lastCheck > checkInterval) {
    int numMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numMessages) {
      for (int i = 0; i < numMessages; i++) {
        String chat_id = bot.messages[i].chat_id;
        String type = bot.messages[i].type;
        String text = bot.messages[i].text;
        String queryId = bot.messages[i].query_id;

        if (type == "callback_query") {
          if (text == "on") {
            esperandoTemperatura = true;
            bot.answerCallbackQuery(queryId, "Esperando temperatura...");
            bot.sendMessage(chat_id, "¿A qué temperatura deseas encender el LED? (10–200 °C)", "");
          }
          else if (text == "off") {
            if (enModoSoldadura) {
              confirmacionApagadoSoldadura = true;
              bot.answerCallbackQuery(queryId, "Confirmar apagado");
              bot.sendMessage(chat_id, "⚠ Estás en modo soldadura. Apagar ahora puede afectar el proceso. Escribe 'CONFIRMAR' para continuar.", "");
            } else {
              ledState = false;
              digitalWrite(ledPin, LOW);
              apagarEn = 0;
              agregarLog("LED apagado manualmente");
              bot.answerCallbackQuery(queryId, "LED apagado");
              bot.sendMessage(chat_id, "LED apagado ❌", "");
              sendKeyboard(chat_id);
            }
          }
          else if (text == "estado") {
            bot.answerCallbackQuery(queryId, "Estado del LED");

            String mensajeEstado = "Temperatura deseada: " + String(temperaturaSet, 2) + " °C\n" +
                        "Temperatura medida: " + String(temperaturaMedida, 2) + " °C\n" +
                        "Potencia salida: " + String(potenciaSalida, 2) + "%\n" +
                        "Kp: " + String(Kp, 2) + "\n" +
                        "Ki: " + String(Ki, 2) + "\n" +
                        "Kd: " + String(Kd, 2) + "\n";
            bot.sendMessage(chat_id, mensajeEstado);

            if (!ledState) {
              bot.sendMessage(chat_id, "LED apagado ❌\nTemperatura actual: " + String(temperatura) + " °C", "");
            }
            else if (enModoSoldadura) {
              unsigned long transcurrido = (millis() - tiempoEtapaInicio) / 1000;
              String etapa;
              if (etapaSoldadura == 1) etapa = "Precalentamiento";
              else if (etapaSoldadura == 2) etapa = "Activación";
              else if (etapaSoldadura == 3) etapa = "Fusión";
              else etapa = "Enfriamiento";
              bot.sendMessage(chat_id, "🧪 Modo soldadura activo\nEtapa: " + etapa + "\nTiempo en etapa: " + String(transcurrido) + " s\nTemperatura: " + String(temperatura) + " °C", "");
            }
            else if (apagarEn > 0) {
              unsigned long ahora = millis();
              int transcurrido = (ahora - tiempoInicio) / 1000;
              int restante = (apagarEn - ahora) / 1000;
              bot.sendMessage(chat_id, "LED encendido ⏱\nEncendido hace: " + String(transcurrido) + " s\nTiempo restante: " + String(restante) + " s\nTemperatura: " + String(temperatura) + " °C", "");
            }
            else {
              bot.sendMessage(chat_id, "LED encendido ✅ a " + String(temperatura) + " °C", "");
            }
            sendKeyboard(chat_id);
          }
          else if (text == "temporizador") {
            esperandoTiempo = true;
            bot.answerCallbackQuery(queryId, "Esperando tiempo...");
            bot.sendMessage(chat_id, "¿Por cuántos segundos deseas encender el LED?", "");
          }
          else if (text == "logs") {
            bot.answerCallbackQuery(queryId, "Mostrando logs...");
            String logTexto = "📃 Historial de eventos:\n";
            for (int j = 0; j < logIndex; j++) {
              logTexto += logs[j] + "\n";
            }
            bot.sendMessage(chat_id, logTexto, "");
            sendKeyboard(chat_id);
          }
          else if (text == "soldadura") {
            iniciarModoSoldadura(chat_id);
          }
        }

        if (type == "message") {
          if (confirmacionApagadoSoldadura && text == "CONFIRMAR") {
            confirmacionApagadoSoldadura = false;
            terminarSoldadura(chat_id);
          }
          else if (esperandoTemperatura) {
            int t = text.toInt();
            if (t >= 10 && t <= 200) {
              esperandoTemperatura = false;
              temperatura = t;
              ledState = true;
              digitalWrite(ledPin, HIGH);
              setTemperaturaSet(temperatura);
              controlHorno();
              ultimoRecordatorio = millis();
              agregarLog("LED encendido manualmente a " + String(temperatura) + " °C");
              bot.sendMessage(chat_id, "LED encendido a " + String(temperatura) + " °C ✅", "");
              sendKeyboard(chat_id);
            } else {
              bot.sendMessage(chat_id, "Temperatura inválida. Ingresa un número entre 10 y 200.", "");
            }
          }
          else if (esperandoTiempo) {
            int segundos = text.toInt();
            if (segundos > 0) {
              esperandoTiempo = false;
              esperandoTemperaturaTemporizada = true;
              tiempoTemporizado = segundos;
              bot.sendMessage(chat_id, "Indica la temperatura para encender el LED (10–200 °C)", "");
            } else {
              bot.sendMessage(chat_id, "Ingresa un número válido de segundos (ej. 10)", "");
            }
          }
          else if (esperandoTemperaturaTemporizada) {
            int t = text.toInt();
            if (t >= 10 && t <= 200) {
              esperandoTemperaturaTemporizada = false;
              temperatura = t;
              ledState = true;
              digitalWrite(ledPin, HIGH);
              setTemperaturaSet(temperatura);
              controlHorno();
              tiempoInicio = millis();
              apagarEn = tiempoInicio + (tiempoTemporizado * 1000);
              agregarLog("LED encendido por " + String(tiempoTemporizado) + " s a " + String(temperatura) + " °C (temporizador)");
              bot.sendMessage(chat_id, "LED encendido por " + String(tiempoTemporizado) + " segundos a " + String(temperatura) + " °C ⏱", "");
              sendKeyboard(chat_id);
            } else {
              bot.sendMessage(chat_id, "Temperatura inválida. Ingresa un número entre 10 y 200.", "");
            }
          }
        }
      }
      numMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastCheck = millis();
  }

  // Apagado automático
  if (apagarEn > 0 && millis() >= apagarEn) {
    ledState = false;
    digitalWrite(ledPin, LOW);
    setTemperaturaSet(10);
    controlHorno();
    apagarEn = 0;
    agregarLog("LED apagado automáticamente tras temporizador.");
    bot.sendMessage(CHAT_ID, "⏱ Tiempo terminado. LED apagado.");
  }

  // Modo soldadura progresión
  if (enModoSoldadura) {
    unsigned long etapaDuracion = (millis() - tiempoEtapaInicio) / 1000;
    if (etapaSoldadura == 1 && etapaDuracion >= 90) {
      etapaSoldadura = 2;
      temperatura = 130;
      setTemperaturaSet(temperatura);
      controlHorno();
      tiempoEtapaInicio = millis();
      agregarLog("Etapa Activación: 130 °C");
      bot.sendMessage(CHAT_ID, "Etapa de activación: 130 °C por 120 segundos.", "");
    } else if (etapaSoldadura == 2 && etapaDuracion >= 120) {
      etapaSoldadura = 3;
      temperatura = 195;
      setTemperaturaSet(temperatura);
      controlHorno();
      tiempoEtapaInicio = millis();
      agregarLog("Etapa Fusión: 195 °C");
      bot.sendMessage(CHAT_ID, "Etapa de fusión: 195 °C por 60 segundos.", "");
    } else if (etapaSoldadura == 3 && etapaDuracion >= 60) {
      etapaSoldadura = 4;
      temperatura = 10;
      setTemperaturaSet(tempratura);
      controlHorno();
      tiempoEtapaInicio = millis();
      agregarLog("Etapa Enfriamiento: 10 °C");
      bot.sendMessage(CHAT_ID, "Etapa de enfriamiento: 10 °C por 5 minutos.", "");
    } else if (etapaSoldadura == 4 && etapaDuracion >= 300) {
      terminarSoldadura(CHAT_ID);
    }
  }

  // Recordatorio cada 5 minutos si LED encendido sin temporizador
  if (ledState && apagarEn == 0 && !enModoSoldadura && millis() - ultimoRecordatorio >= intervaloRecordatorio) {
    ultimoRecordatorio = millis();
    agregarLog("Recordatorio enviado: LED sigue encendido");
    bot.sendMessage(CHAT_ID, "⚠ El LED sigue encendido desde hace más de 5 minutos. ¿Deseas apagarlo?", "");
    sendKeyboard(CHAT_ID);
  }
}
