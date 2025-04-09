#define BLYNK_TEMPLATE_ID "BLYNK TEMPLATE ID HERE"
#define BLYNK_TEMPLATE_NAME "Home Automation"
#define BLYNK_AUTH_TOKEN "AUTH TOKEN HERE"
#define WIFI_SSID "WIFI NAME"
#define WIFI_PASSWORD "WIFI PASSWORD"
#include <DHT.h>
#include <BlynkSimpleEsp32.h>

// Define pins
#define DHTPIN 4          // GPIO pin for DHT11
#define RELAY1_PIN 23      // GPIO pin for Relay 1 (LED)
#define RELAY2_PIN 22     // GPIO pin for Relay 2 (FAN)
#define RELAY3_PIN 21     // GPIO pin for Relay 3 (DOOR LOCK)
#define RELAY4_PIN 19     // GPIO pin for Relay 4 (MOTOR)
#define BUZZER_PIN 32     // GPIO pin for Buzzer
#define MQ2_PIN 34        // GPIO pin for MQ2 Sensor (Analog)
#define PIR_PIN 27        // GPIO pin for PIR Sensor
#define TRIGGER_PIN 5
#define ECHO_PIN 18

// Constants
#define DHTTYPE DHT11     // DHT11 sensor type
#define TEMP_THRESHOLD 45 // Temperature threshold for fan
#define GAS_THRESHOLD 1500 // Gas concentration threshold for buzzer
#define DISTANCE_THRESHOLD 5 // Distance threshold in centimeters

// Variables
bool relay1State = true; // Initial state of Relay 1 (LED)
bool relay2State = true; // Initial state of Relay 2 (FAN)
bool relay3State = true; // Initial state of Relay 3 (DOOR LOCK)
bool relay4State = true; // Initial state of Relay 4 (MOTOR)
int pirState = LOW; // start with no motion detected
int val = 0; // variable to store the PIR Sensor reading

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);


void setup() {
  // Start Serial Monitor for debugging
  Serial.begin(115200);

  // Initialize DHT sensor
  dht.begin();

  // Set pin modes
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);           // PIR sensor as input
  pinMode(TRIGGER_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);

  // Initialize relays and buzzer to off
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  digitalWrite(RELAY3_PIN, HIGH);
  digitalWrite(RELAY4_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);

// Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Wait for the connection to establish
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("WiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Connect to Blynk
  Serial.println("Connecting to Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
  
  // Check Blynk connection status
  while (!Blynk.connected()) {
    Serial.print(".");
    delay(1000); // Wait for a second before checking again
  }

  // Once Blynk is connected
  Serial.println("\nBlynk connected!");
}

void loop() {
  // Run Blynk
  Blynk.run();

  // Handle DHT11 sensor and control fan/buzzer
  handleDHT11();

  // Handle MQ2 sensor and trigger buzzer
  handleMQ2();

  // Handle PIR sensor and control LED (Relay 1)
  handlePIR();

  // Handle UltraSonic sensor to control Motor (Relay 4)
  handleUltrasonic();

  // Small delay to avoid overloading the loop
  delay(3000);
}

// Function to handle DHT11 sensor
void handleDHT11() {
  // Read temperature and humidity
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Check if readings are valid
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Print temperature and humidity to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  // Send data to Blynk
  Blynk.virtualWrite(V0, temperature); // Virtual Pin V0 for temperature
  Blynk.virtualWrite(V1, humidity);   // Virtual Pin V1 for humidity

  // Control fan based on temperature
  if (temperature > TEMP_THRESHOLD) {
    digitalWrite(RELAY2_PIN, LOW); // Turn on the fan
    Blynk.virtualWrite(V2, 1);     // Virtual Pin V2 for fan status (1 = ON)
    Serial.println("Fan is ON,(Temperature)");
  } else {
    digitalWrite(RELAY2_PIN, HIGH); // Turn off the fan
    Blynk.virtualWrite(V2, 0);     // Virtual Pin V2 for fan status (0 = OFF)
    Serial.println("Fan is OFF, (Temperature)");
  }
}



// Function to handle MQ2 sensor
void handleMQ2() {
  // Read gas concentration from MQ2 sensor
  int gasValue = analogRead(MQ2_PIN);

  // Print gas value to Serial Monitor
  Serial.print("Gas Concentration: ");
  Serial.println(gasValue);

  // Send gas concentration to Blynk
  Blynk.virtualWrite(V5, gasValue); // Virtual Pin V5 for gas concentration

  // Trigger buzzer if gas concentration exceeds threshold
  if (gasValue > GAS_THRESHOLD) {
    digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
    Blynk.virtualWrite(V3, 1);     // Virtual Pin V3 for buzzer status (1 = ON)
    Serial.println("Buzzer is ON (Gas)");
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer
    Blynk.virtualWrite(V3, 0);     // Virtual Pin V3 for buzzer status (0 = OFF)
    Serial.println("Buzzer is OFF (Gas)");
  }
}

// Function to handle PIR sensor
void handlePIR() {
  // Read PIR sensor state
  val = digitalRead(PIR_PIN);

  if(val == 1){
    Serial.println("Motion Detected!: Light is ON");
    digitalWrite(RELAY1_PIN,LOW);
  } else {
    digitalWrite(RELAY1_PIN,HIGH);
  }
  delay(1000);
}

// Turn on Water Moter
void handleUltrasonic() {
    long duration, distance;
  
  // Send a pulse to the Trig pin
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Read the Echo pin and calculate the distance
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // Convert to centimeters
  Blynk.virtualWrite(V7, distance);
  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Control the relay based on the distance
  if (distance > DISTANCE_THRESHOLD) {
    // If the distance is greater than the threshold, turn on the relay (LOW for active LOW relay)
    digitalWrite(RELAY4_PIN, LOW);
    Serial.println("Motor ON");
  } else {
    // If the distance is greater than the threshold, turn off the relay (HIGH for active LOW relay)
    digitalWrite(RELAY4_PIN, HIGH);
    Serial.println("Motor OFF");
  }

  // Small delay to avoid rapid switching
  delay(500);
}

// Blynk virtual pin for manual control of relays
BLYNK_WRITE(V8) { // Virtual Pin V8 for Relay 1 (LED)
  int value = param.asInt();
  digitalWrite(RELAY1_PIN, value);
  relay1State = value;
  Serial.print("Relay 1 (LED) is ");
  Serial.println(value ? "ON" : "OFF");
}

BLYNK_WRITE(V9) { // Virtual Pin V9 for Relay 2 (FAN)
  int value = param.asInt();
  digitalWrite(RELAY2_PIN, value);
  relay2State = value;
  Serial.print("Relay 2 (FAN) is ");
  Serial.println(value ? "ON" : "OFF");
}

BLYNK_WRITE(V10) { // Virtual Pin V10 for Relay 3 (DOOR LOCK)
  int value = param.asInt();
  digitalWrite(RELAY3_PIN, value);
  relay3State = value;
  Serial.print("Relay 3 (DOOR LOCK) is ");
  Serial.println(value ? "ON" : "OFF");
}
BLYNK_WRITE(V11) { // Virtual Pin V10 for Relay 4 (MOTOR)
  int value = param.asInt();
  digitalWrite(RELAY4_PIN, value);
  relay4State = value;
  Serial.print("Relay 4 (MOTOR) is ");
  Serial.println(value ? "ON" : "OFF");
}
