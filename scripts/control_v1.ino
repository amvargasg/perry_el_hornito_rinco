#include <Arduino.h>

#include <algorithm>
#include "max6675.h"
#include <RBDdimmer.h>
#include <PID_v1.h>

int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;

const int zeroCrossPin  = 25;
const int acdPin  = 27;

const int ventiPin = 32;

int power = 50;

// Definir variables del PID

double temperaturaALlegar = 190;  // Temperatura deseada en °C
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


double temperaturaSet = 180; 
// Crear el controlador PID
PID controladorPID(&temperaturaMedida, &potenciaSalida, &temperaturaSet, Kp, Ki, Kd, DIRECT);




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

void setup() {
  Serial.begin(9600);
  acd.begin(NORMAL_MODE, ON);

  controladorPID.SetMode(AUTOMATIC);
  controladorPID.SetOutputLimits(10, 100);  // Rango de control del dimmer (10-100%)

  pinMode(ventiPin, OUTPUT);

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp

  digitalWrite(ventiPin, LOW);
  
  temperaturaMedida = leerTemperaturaSuavizada();
  actualizarPID();
  power = round(potenciaSalida);
  acd.setPower(power);


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

  delay(500);
}