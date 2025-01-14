#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define PIN 3              // Pin per il LED WS2812B
#define NUMPIXELS 10       // Numero di LED WS2812B
#define DHTPIN 2           // Pin per DHT11 sensore temperatura e umidità interno
#define DHTPIN_EXT 8       // Pin per DHT11 sensore temperatura e umidità esterno
#define DHTTYPE DHT11      // Modello sensore DHT
#define MQ135_PIN A0       // Pin per MQ-135 sensore gas
#define SOIL_PIN A1        // Pin analogico per il sensore di umidità del suolo
#define FAN_PIN 4          // Pin per la ventola
#define PUMP_PIN 5         // Pin per la pompa
#define SERVO_PIN 6        // Pin per il servomotore
#define BUZZER_PIN 7       // Pin per il cicalino

// WiFi Configuration
const char* ssid = "SSID";
const char* password = "SSID_PASSWORD";

// MQTT Broker Configuration:
const char *topic = "v1/devices/me/telemetry";
const char *mqtt_server = "LOCAL_MQTT_SERVER";
const int mqtt_port = 1884;
const char *mqtt_clientID = "SmartGreenhouse";
const char *mqtt_username = "Smart_User";
const char *mqtt_password = "Smart_Password";

// MQTT Client setup
WiFiClient wifiClient;
PubSubClient client(wifiClient); 

BH1750 lightMeter;         
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
DHT dht(DHTPIN, DHTTYPE);
DHT dhtExt(DHTPIN_EXT, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servoMotor;

enum Mode { AUTO, MANUAL };
Mode mode = AUTO;

float Ro = 5;    // Valore stimato di Ro in aria pulita
float RL = 1;     // Resistenza di carico del sensore
float m = -0.2;  // Fattore di sensibilità
float b = 1.2;    // Offset della curva caratteristica

unsigned long lastLightUpdate = 0;   // Timer per il sensore di luminosità
unsigned long lastSensorUpdate = 0;  // Timer per gli altri sensori

int currentServoPosition = 5;        // Posizione corrente del servomotore
bool fanOn = false;                  // Stato attuale della ventola
bool pumpOn = false;                 // Stato attuale della pompa
bool servoOn = false;                // Stato attuale del servomotore
unsigned long fanStartTime = 0;      // Momento di accensione della ventola
unsigned long pumpStartTime = 0;     // Memorizza il tempo in cui la pompa è stata accesa

float smoothedLux = 0.0;
const float alpha = 0.1;

// Variabili per memorizzare il preset luci
int currentPreset = 1;  // Predefinito a "Seed Germination"
bool alarmMode = false; // Stato di allarme LED

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 FVI Sensor Initialized");
  } else {
    Serial.println("Error: BH1750 not found.");
  }

  pixels.begin();
  pixels.show();

  lcd.init();
  lcd.backlight();

  dht.begin();
  dhtExt.begin();

  servoMotor.attach(SERVO_PIN);
  servoMotor.write(currentServoPosition);

  pinMode(FAN_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  if (!connectToWiFi(5000, 3)) {
    Serial.println("Wi-Fi not connected. Running in offline mode.");
  }

  // Configura il client MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(handleMQTTMessage); // Assegna la funzione di callback

  if (!reconnectMQTT(5000, 3)) {
    Serial.println("MQTT not connected. Running in offline mode.");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Verifica periodica delle connessioni
  static unsigned long lastWiFiCheck = 0;
  static unsigned long lastMQTTCheck = 0;

  if (currentMillis - lastWiFiCheck >= 120000) {
    lastWiFiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi(3000, 3);
    }
  }

  if (currentMillis - lastMQTTCheck >= 120000) {
    lastMQTTCheck = currentMillis;
    if (!client.connected()) {
      reconnectMQTT(3000, 3);
    }
  }

  client.loop();

  // Logica principale
  if (currentMillis - lastLightUpdate >= 1000) {
    lastLightUpdate = currentMillis;
    updateLights();
    controlAlarm();
  }

  if (currentMillis - lastSensorUpdate >= 40000) {
    lastSensorUpdate = currentMillis;
    updateSensors();
  }
}

bool connectToWiFi(unsigned long timeoutMs, int maxAttempts) {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttemptTime >= timeoutMs) {
      attempts++;
      Serial.print("Wi-Fi attempt ");
      Serial.print(attempts);
      Serial.println(" failed.");

      if (attempts >= maxAttempts) {
        Serial.println("Wi-Fi connection failed. Continuing without Wi-Fi.");
        return false; // Interrompi dopo il numero massimo di tentativi
      }

      startAttemptTime = millis(); // Resetta il timer
    }
    delay(100);
  }

  Serial.println("Wi-Fi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

bool reconnectMQTT(unsigned long timeoutMs, int maxAttempts) {
  if (client.connected()) {
    return true;
  }

  Serial.println("Connecting to MQTT...");
  unsigned long startAttemptTime = millis();
  int attempts = 0;

  while (!client.connected()) {
    if (millis() - startAttemptTime >= timeoutMs) {
      attempts++;
      Serial.print("MQTT attempt ");
      Serial.print(attempts);
      Serial.println(" failed.");

      if (attempts >= maxAttempts) {
        Serial.println("MQTT connection failed. Continuing without MQTT.");
        return false; // Interrompi dopo il numero massimo di tentativi
      }

      startAttemptTime = millis(); // Resetta il timer
    }

    if (client.connect(mqtt_clientID, mqtt_username, mqtt_password)) {
      Serial.println("MQTT connected!");
      client.subscribe("v1/devices/me/rpc/request/+");
      return true;
    }
    delay(100);
  }

  return false;
}

void handleMQTTMessage(char* topic, byte* payload, unsigned int length) {
  // Converti il payload in JSON
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Parse del messaggio JSON per estrarre "params"
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, message);
  String paramsValue;

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Recupera il valore di "params" se presente
  if (doc.containsKey("params")) {
    paramsValue = doc["params"].as<String>();
  } else {
    Serial.println("Key 'params' not found in message.");
  }

  handleMQTTCommand(paramsValue);
}

void handleMQTTCommand(String command) {
  int cmd = command.toInt();

  switch (cmd) {
    case 0:
      mode = AUTO;
      Serial.println("Mode set to: Automatic");
      break;

    case 99:
      mode = MANUAL;
      Serial.println("Mode set to: Manual");
      break;

    case 1: case 2: case 3: case 4: case 5: case 6:
      currentPreset = cmd;
      Serial.print("Preset changed to: ");
      Serial.println(cmd);
      break;

    case 7:
      if (mode == MANUAL) {
        digitalWrite(FAN_PIN, HIGH);
        fanOn = true;
        Serial.println("Fan turned ON (Manual mode).");
      } else {
        Serial.println("Command ignored. Switch to manual mode to control the fan.");
      }
      break;

    case 8:
      if (mode == MANUAL) {
        digitalWrite(FAN_PIN, LOW);
        fanOn = false;
        Serial.println("Fan turned OFF (Manual mode).");
      } else {
        Serial.println("Command ignored. Switch to manual mode to control the fan.");
      }
      break;

    case 9:
      if (mode == MANUAL) {
        digitalWrite(PUMP_PIN, HIGH);
        pumpOn = true;
        Serial.println("Pump turned ON (Manual mode).");
      } else {
        Serial.println("Command ignored. Switch to manual mode to control the pump.");
      }
      break;

    case 10:
      if (mode == MANUAL) {
        digitalWrite(PUMP_PIN, LOW);
        pumpOn = false;
        Serial.println("Pump turned OFF (Manual mode).");
      } else {
        Serial.println("Command ignored. Switch to manual mode to control the pump.");
      }
      break;

    case 11:
      if (mode == MANUAL) {
        moveServoSlowly(90);
        currentServoPosition = 90;
        servoOn = true;
        Serial.println("Servo opened (Manual mode).");
      } else {
        Serial.println("Command ignored. Switch to manual mode to control the servo.");
      }
      break;

    case 12:
      if (mode == MANUAL) {
        moveServoSlowly(5);
        currentServoPosition = 5;
        servoOn = false;
        Serial.println("Servo closed (Manual mode).");
      } else {
        Serial.println("Command ignored. Switch to manual mode to control the servo.");
      }
      break;

    case 13:
      alarmMode = true;
      Serial.println("Alarm On! (Manual mode).");
      break;

    case 14:
      alarmMode = false;
      Serial.println("Alarm Off! (Manual mode).");
      break;

    default:
      Serial.println("Invalid command. Enter a valid number.");
      break;
  }
}

void updateLights() {
  if(alarmMode == true) {
    return;
  }

  float lux = lightMeter.readLightLevel();

  if (lux < 0) {
    Serial.println("Error reading light level from BH1750");
    return; // Evita il crash continuando il ciclo
  }

  if (lux >= 0) {
    smoothedLux = alpha * lux + (1 - alpha) * smoothedLux;

    int maxLux = 80;
    int minLux = 0;
    int ledBrightness = map(smoothedLux, maxLux, minLux, 0, 255);
    ledBrightness = constrain(ledBrightness, 0, 255);

    switch (currentPreset) {
      case 1:
        applySeedGerminationPreset(ledBrightness);
        break;
      case 2:
        applyVegetativeGrowthPreset(ledBrightness);
        break;
      case 3:
        applyLeafStrengtheningPreset(ledBrightness);
        break;
      case 4:
        applyFlowerInductionPreset(ledBrightness);
        break;
      case 5:
        applyFruitGrowthPreset(ledBrightness);
        break;
      case 6:
        applyNaturalLightPreset(ledBrightness);
        break;
      default:
        applySeedGerminationPreset(ledBrightness);
    }
  } else {
    Serial.println("Error reading light level.");
  }
}

// Di seguito i vari preset LED per la crescita della pianta
void applySeedGerminationPreset(int ledBrightness) {
  int redBrightness = ledBrightness * 0.3;   // 30% rosso
  int blueBrightness = ledBrightness * 0.7;  // 70% blu
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(redBrightness, 0, blueBrightness));
  }
  pixels.show();
}

void applyVegetativeGrowthPreset(int ledBrightness) {
  int redBrightness = ledBrightness * 0.2;   // 20% rosso
  int blueBrightness = ledBrightness * 0.8;  // 80% blu
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(redBrightness, 0, blueBrightness));
  }
  pixels.show();
}

void applyLeafStrengtheningPreset(int ledBrightness) {
  int redBrightness = ledBrightness * 0.4;   // 40% rosso
  int blueBrightness = ledBrightness * 0.6;  // 60% blu
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(redBrightness, 0, blueBrightness));
  }
  pixels.show();
}

void applyFlowerInductionPreset(int ledBrightness) {
  int redBrightness = ledBrightness * 0.7;   // 70% rosso
  int blueBrightness = ledBrightness * 0.3;  // 30% blu
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(redBrightness, 0, blueBrightness));
  }
  pixels.show();
}

void applyFruitGrowthPreset(int ledBrightness) {
  int redBrightness = ledBrightness * 0.9;   // 90% rosso
  int blueBrightness = ledBrightness * 0.1;  // 10% blu
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(redBrightness, 0, blueBrightness));
  }
  pixels.show();
}

void applyNaturalLightPreset(int ledBrightness) {
  int redBrightness = ledBrightness * 0.9;   // 90% rosso
  int greenBrightness = ledBrightness * 0.6;  // 60% verde
  int blueBrightness = ledBrightness * 0.2;   // 20% blu
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(redBrightness, greenBrightness, blueBrightness));
  }
  pixels.show();
}

void updateSensors() {
  int temperature = dht.readTemperature();
  int temperatureExt = dhtExt.readTemperature();
  int humidity = dht.readHumidity();
  int humidityExt = dhtExt.readHumidity();
  int lux = lightMeter.readLightLevel();

  displayTemperatureAndHumidity(temperature, humidity);

  float gasPpm = readGasLevel();
  int soilMoisturePercent = readSoilMoisture();

  controlTransistor(temperature, gasPpm, soilMoisturePercent, humidity);
  controlServo(temperature, humidity, gasPpm);

  // Creazione del payload JSON
  char json_payload[200];

  snprintf(
      json_payload,
      sizeof(json_payload),
      "{\"temperature\":%d,\"temperatureExt\":%d,\"humidity\":%d,\"humidityExt\":%d,\"gasPpm\":%2f,\"soilMoisturePercent\":%d,\"lux\":%d}",
      (int)temperature,
      (int)temperatureExt,
      (int)humidity,
      (int)humidityExt,
      (float)gasPpm,
      (int)soilMoisturePercent,
      (int)lux
    );

  client.publish(topic, json_payload);
}

void displayTemperatureAndHumidity(int temperature, int humidity) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp/Hum:");
  lcd.print(temperature);
  lcd.print("C ");
  lcd.print(humidity);
  lcd.print("%");
}

float readGasLevel() {
  const int maxPpm = 3000;
  int sensorValue = analogRead(MQ135_PIN);
  float sensorVoltage = sensorValue * (5.0 / 1023.0);

  // Verifica letture valide
  if (sensorVoltage <= 0 || sensorVoltage >= 5.0) {
    Serial.println("Error: invalid sensor voltage");
    return 0;
  }

  float Rs = (5.0 - sensorVoltage) / sensorVoltage * RL;
  if (Rs <= 0) {
    Serial.println("Error: invalid Rs value");
    return 0;
  }

  float ratio = Rs / Ro;
  float gasPpm = (0.1) * (pow(10, ((log10(ratio) - b) / m)));
  
  gasPpm = constrain(gasPpm, 0, maxPpm);

  lcd.setCursor(0, 1);
  lcd.print("CO2: ");
  lcd.print(gasPpm);
  lcd.print("ppm");

  return gasPpm;
}

int readSoilMoisture() {
  int soilValue = analogRead(SOIL_PIN);
  int soilMoisturePercent = map(soilValue, 1023, 0, 0, 100);

  return soilMoisturePercent;
}

void controlTransistor(int temperature, float gasPpm, int soilMoisturePercent, int humidity) {
  if (mode == AUTO) {
    // Logica automatica
    bool shouldTurnOnFan = (temperature > 27 || humidity > 70 || gasPpm > 700);
    
    if (shouldTurnOnFan) {
      if (!fanOn) { // Accendi la ventola solo se non è già accesa
        digitalWrite(FAN_PIN, HIGH);
        fanOn = true;
        Serial.println("Fan turned ON (Auto mode).");
      }
    } else {
      if (fanOn) { // Spegni la ventola solo se è accesa
        digitalWrite(FAN_PIN, LOW);
        fanOn = false;
        Serial.println("Fan turned OFF (Auto mode).");
      }
    }

    // Logica della pompa (accensione temporanea per 2 secondi)
    if (soilMoisturePercent < -0.5 && !pumpOn) {
      digitalWrite(PUMP_PIN, HIGH);
      pumpStartTime = millis(); // Memorizza il momento dell'accensione
      pumpOn = true;
      Serial.println("Pump turned ON for 2 seconds.");
    }

    // Spegni la pompa dopo 2 secondi
    if (pumpOn && millis() - pumpStartTime >= 2000) { // 2000 ms = 2 secondi
      digitalWrite(PUMP_PIN, LOW);
      pumpOn = false;
      Serial.println("Pump turned OFF after 2 seconds.");
    }
  }

  // Logica manuale
  if (mode == MANUAL) {
    if (gasPpm > 700) { // Override per sicurezza
      digitalWrite(FAN_PIN, HIGH);
      fanOn = true;
      Serial.println("Fan forced ON due to high gas level (Manual mode).");
    } else {
      digitalWrite(FAN_PIN, LOW);
      fanOn = false;
    }
  }
}

void moveServoSlowly(int targetPosition) {
  if (targetPosition != currentServoPosition) {
    if (targetPosition > currentServoPosition) {
      for (int pos = currentServoPosition; pos <= targetPosition; pos++) {
        servoMotor.write(pos);
        delay(15); // Ritardo per rallentare il movimento
      }
    } else if (targetPosition < currentServoPosition) {
      for (int pos = currentServoPosition; pos >= targetPosition; pos--) {
        servoMotor.write(pos);
        delay(15); // Ritardo per rallentare il movimento
      }
    }
    currentServoPosition = targetPosition; // Aggiorna la posizione corrente
  }
}

void controlServo(int temperature, int humidity, float gasPpm) {
  if (mode == AUTO) {
    int newServoPosition = (temperature > 27 || humidity > 65 || gasPpm > 700) ? 90 : 5;
    if (newServoPosition != currentServoPosition) {
      moveServoSlowly(newServoPosition);
      servoOn = true;
    } else {
      servoOn = false;
    }
  }

  // Bypass della modalità manuale per motivi di sicurezza in caso di rilevamento di gas
  if (mode == MANUAL && gasPpm > 700) {
    int newServoPosition = 90;
    if (newServoPosition != currentServoPosition) {
      moveServoSlowly(newServoPosition);
      servoOn = true;
    }
  } else {
    servoOn = false;
  }
}

void controlAlarm() {
  // Controlla condizioni di allarme per il cicalino
  if (alarmMode == true) {
    static unsigned long lastBlinkTime = 0;
    static bool ledState = false;

    unsigned long currentMillis = millis();

    // LED lampeggianti
    if (currentMillis - lastBlinkTime >= 500) {
      lastBlinkTime = currentMillis;
      ledState = !ledState; // Inverte lo stato dei LED

      if (ledState) {
        // Accendi i LED di rosso
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(255, 0, 0));
        }
      } else {
        // Spegni i LED
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        }
      }
      pixels.show();
    }

    tone(BUZZER_PIN, 3000);
    delay(100);
    tone(BUZZER_PIN, 2000);
    delay(100);
    noTone(BUZZER_PIN);

  } 
}