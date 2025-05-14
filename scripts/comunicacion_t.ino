#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

const char* ssid = "<Nombre de tu red WiFi>";
const char* password = "<Contraseña de tu red WiFi>";
const char* botToken = "<Token de tu bot>";
const int64_t chatId = <Tu ID de usuario>;
 

WiFiClientSecure client;
UniversalTelegramBot bot(botToken, client);

const int relePin = 5;

unsigned long lastTimeBotRan;
const unsigned long botInterval = 2000;

void setup() {
  Serial.begin(9600);
  pinMode(relePin, OUTPUT);
  digitalWrite(relePin, LOW);  // horno apagado al inicio

  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi con IP: " + WiFi.localIP().toString());

  client.setInsecure();  // para evitar errores con certificados SSL
}

void loop() {
  if (millis() - lastTimeBotRan > botInterval) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      for (int i = 0; i < numNewMessages; i++) {
        String msg = bot.messages[i].text;
        String fromName = bot.messages[i].from_name;
        String userId = bot.messages[i].chat_id;

        Serial.println("=====================================");
        Serial.println("Mensaje recibido de: " + fromName);
        Serial.println("ID del usuario: " + userId);
        Serial.println("Texto recibido: " + msg);
        Serial.println("=====================================");

        if (msg == "/on") {
          digitalWrite(relePin, HIGH);
          bot.sendMessage(String(chatId), "Horno ENCENDIDO", "");
        } else if (msg == "/off") {
          digitalWrite(relePin, LOW);
          bot.sendMessage(String(chatId), "Horno APAGADO", "");
        } else {
          bot.sendMessage(String(chatId), "Comando no reconocido. Usa /on o /off", "");
        }
      }
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    lastTimeBotRan = millis();
  }
}