# Assignment2_iot2
Assignment 2 due nov30
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Joey_Simon";
const char* password = "Joey_SimonD";
const char* mqtt_server = "YOUR_RASPBERRY_PI_IP";

WiFiClient espClient;
PubSubClient client(espClient);

// Define sensor variables (replace with your sensor connection details)
const int sensorPin = 34; // Analog pin for sensor (potentiometer/photocell)

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqtt_server, 1883); // Set MQTT broker address and port
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Read sensor value
  int sensorValue = analogRead(sensorPin);

  // Publish sensor value to appropriate topic (e.g., "aquarium/temperature")
  client.publish("aquarium/temperature", String(sensorValue).c_str());

  delay(60000); // Publish every minute (adjust as needed)
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

import paho.mqtt.client as mqtt

# Define MQTT broker address and topic
broker_address = "YOUR_RASPBERRY_PI_IP"
topic = "aquarium/heater"

def on_connect(client, userdata, flags, rc):
  if rc == 0:
    print("Connected to MQTT Broker!")
    client.subscribe(topic)
  else:
    print("Failed to connect, return code ", rc)

def on_message(client, userdata, msg):
  # Set GPIO pin 18 to output mode
  import RPi.GPIO as GPIO
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(18, GPIO.OUT)

  # Control LED based on message payload
  if msg.payload.decode() == "on":
    GPIO.output(18, GPIO.HIGH)
  else:
    GPIO.output(18, GPIO.LOW)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address)

# Run the loop forever
client.loop_forever()
// Add these lines to your existing `sensor.ino` sketch

#include <Wire.h> // Include for I2C communication (if using)

// Define LED and resistor connection (replace with your pin numbers)
const int ledPin = 18;

void setup() {
  // ... existing setup code ...

  pinMode(ledPin, OUTPUT);
}

void loop() {
  // ... existing loop code ...

