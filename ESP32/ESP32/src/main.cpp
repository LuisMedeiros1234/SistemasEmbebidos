#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// SPI Pins
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCK 18
#define VSPI_SS 5
#define bufferSize 25

// Encoder counts from STM32 to ESP32 Ratio = 256
// Middle Sensor Pins
#define SCL_PIN 33
#define SDA_PIN 32
#define XSHUT_PIN 17
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float middlesensor = 0, leftsensor = 0, rightsensor = 0;

const int sensorPin1 = 36; // VP pin connected to the Sensor 1 Left
const int sensorPin3 = 34; // 34 pin connected to the Sensor 3 Right
const char *ssid = "ESP32-Access-Point";
const char *password = "password";

void Left_RightSensors();
void middleSensor();
void DataSPI();
void moveToTarget(float targetX, float targetY, float targetTheta);

// Constants for tuning the behavior
const float Kp_linear = 1.0;    // Proportional gain for linear velocity
const float Kp_angular = 2.0;   // Proportional gain for angular velocity
const float wheelBase = 0.1605;  // Distance between wheels
const float safety_distance = 0.15; // Safety distance from obstacles
const float k_att = 1.6;        // Attractive force constant
const float k_rep = 4; 
float errorX = 0, errorY = 0, errorTheta = 0;
float distanceError = 0, angleError = 0;
float linearVelocity = 0, angularVelocity= 0;
float leftWheelVel = 0, rightWheelVel = 0;
float targetX =0, targetY=0, targetTheta=0;
float CurrentSpeed =0;

typedef union _packet_buffer_t
{
  uint8_t buff[4];
  float valueOdo;
} packet_buffer_t;

packet_buffer_t _packet_send;
packet_buffer_t _packet_receive;

uint8_t RxBuffer[bufferSize];
uint8_t TxBuffer[bufferSize];

float X, Y, Theta;

AsyncWebServer server(80);

// Position variables
float x = 0.0;
float y = 0.0;
float w = 0.0;
float theta = 0.0;
uint8_t state = 0; // State variable (1 or 0)
float Px = 0, Py = 0, Pt = 0;
int storedSpeed = 0; // Declare as global variable

// Forward declarations
String getJsonResponse();

void setup()
{
  //--- SPI Setup ---//
  pinMode(VSPI_SS, OUTPUT);
  pinMode(VSPI_MISO, INPUT);
  pinMode(VSPI_MOSI, OUTPUT);
  pinMode(VSPI_SCK, OUTPUT);

  digitalWrite(VSPI_SS, HIGH); // Ensure SS is high to avoid accidental SPI communication

  SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);

  //--- MiddleSensor/Lox ---//
  while (! Serial) {
    delay(1);
  }
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
  lox.startRangeContinuous();

  //--- Initialize SPIFFS ---//
  if(!SPIFFS.begin(true)){
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }

  //--- Initialize Access Point/WebPage ---//
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println(IP);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.HTML", "text/html"); });

  server.on("/javascript.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/javascript.js", "application/javascript"); });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request)
            {String response = getJsonResponse(); request->send(200, "application/json", response); });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        if (request->hasParam("state")) {
            String stateParam = request->getParam("state")->value();
            state = stateParam.toInt(); // Update state based on the value received from client
            Serial.printf("Received state update: %d\n", state);
            request->send(200, "application/json", "{\"status\":\"success\"}");
        } else {
            request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"State parameter missing\"}");
        } });

  server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("speed")) {
        String speedParam = request->getParam("speed")->value();
        int speedValue = speedParam.toInt();
        if (speedValue >= 0 && speedValue <= 10) {
            // Store the speed value for later use
            storedSpeed = speedValue;
            Serial.printf("Speed value stored: %d\n", storedSpeed);
            request->send(200, "application/json", "{\"status\":\"success\"}");
        } else {
            request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Invalid speed value\"}");
        }
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Speed parameter missing\"}");
    }
  });

  server.on("/coordinates", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    if (request->hasParam("Px") && request->hasParam("Py") && request->hasParam("Pt")) {
        String PxParam = request->getParam("Px")->value();
        String PyParam = request->getParam("Py")->value();
        String PtParam = request->getParam("Pt")->value();
        Px = PxParam.toFloat();
        Py = PyParam.toFloat();
        Pt = PtParam.toFloat();
        Serial.printf("Px: %f, Py: %f, Pt: %f\n", Px, Py, Pt);
        request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Parameters missing\"}");
    }
  });

  server.begin();
}

void loop(){ 

    if(state == 1){
    middleSensor();
    Left_RightSensors();
    moveToTarget((float)Px, (float)Py, (float)Pt);
    }

    if(state == 0){
        leftWheelVel = 0;
        rightWheelVel = 0;
        DataSPI();
    }

  // Sends the value to Webpage 
  x = X;
  y = Y;
  theta = Theta;
  w = CurrentSpeed;
  delay(200);
}

// SPI Sending and Receiving Data
void DataSPI(){
  /*---Sending Data To STM32---*/
  _packet_send.valueOdo = leftWheelVel;

  TxBuffer[5] = _packet_send.buff[0];
  TxBuffer[6] = _packet_send.buff[1];
  TxBuffer[7] = _packet_send.buff[2];
  TxBuffer[8] = _packet_send.buff[3];

  _packet_send.valueOdo = rightWheelVel;

  TxBuffer[9] = _packet_send.buff[0];
  TxBuffer[10] = _packet_send.buff[1];
  TxBuffer[11] = _packet_send.buff[2];
  TxBuffer[12] = _packet_send.buff[3];

  digitalWrite(VSPI_SS, LOW);
  SPI.transferBytes(TxBuffer, RxBuffer, bufferSize);
  digitalWrite(VSPI_SS, HIGH);

  /*---Receiving Data From STM32---*/
  _packet_receive.buff[0] = RxBuffer[5];
  _packet_receive.buff[1] = RxBuffer[6];
  _packet_receive.buff[2] = RxBuffer[7];
  _packet_receive.buff[3] = RxBuffer[8];

  X = _packet_receive.valueOdo;

  _packet_receive.buff[0] = RxBuffer[9];
  _packet_receive.buff[1] = RxBuffer[10];
  _packet_receive.buff[2] = RxBuffer[11];
  _packet_receive.buff[3] = RxBuffer[12];

  Y = _packet_receive.valueOdo;

  _packet_receive.buff[0] = RxBuffer[13];
  _packet_receive.buff[1] = RxBuffer[14];
  _packet_receive.buff[2] = RxBuffer[15];
  _packet_receive.buff[3] = RxBuffer[16];

  Theta = _packet_receive.valueOdo;

  _packet_receive.buff[0] = RxBuffer[17];
  _packet_receive.buff[1] = RxBuffer[18];
  _packet_receive.buff[2] = RxBuffer[19];
  _packet_receive.buff[3] = RxBuffer[20];

  CurrentSpeed = _packet_receive.valueOdo;
}

//Sensor 1 and 3, Left and Right Respectively
void Left_RightSensors(){
  // Read the analog value from the sensor
  int sensorValue1 = analogRead(sensorPin1);
  int sensorValue3 = analogRead(sensorPin3);

  // Convert the sensor value to voltage (assuming 12-bit ADC)
  float voltage1 = sensorValue1 * (3.3 / 4095.0);
  float voltage3 = sensorValue3 * (3.3 / 4095.0);

  // Convert voltage to distance using a linear approximation
  // The conversion formula, calibration needed based on the actual sensor characteristics Data Sheets
  leftsensor = (12.08 * pow(voltage1, -1.058)) / 100;
  rightsensor = (12.08 * pow(voltage3, -1.058)) / 100;

  // Print the distance1
  Serial.print("Distance from left: ");
  Serial.print(leftsensor);
  Serial.println(" m");

  // Print the distance3
  Serial.print("Distance from right: ");
  Serial.print(rightsensor);
  Serial.println(" m");
}

//Sensor 2, Middle Sensor
void middleSensor(){
  middlesensor = (lox.readRange()) / 1000;
  Serial.print("Middle Distance: ");
  Serial.print(middlesensor);
  Serial.println(" m");
}

//Send Robot to coordinates
void moveToTarget(float targetX, float targetY, float targetTheta) {
    // Calculate attractive force towards the target
    float attractiveForceX = k_att * (targetX - X);
    float attractiveForceY = k_att * (targetY - Y);

    // Initialize repulsive forces
    float repulsiveForceX = 0;
    float repulsiveForceY = 0;

    float d = 0.045; // Example distance of side sensors from the center (in meters)

    // Check for obstacles and calculate repulsive forces
    float obstacleDistance1 = leftsensor;
    float obstacleDistance2 = middlesensor;
    float obstacleDistance3 = rightsensor;

    if (obstacleDistance1 < safety_distance) {
      float repulsiveMagnitude1 = k_rep * (1.0 / obstacleDistance1 - 1.0 / safety_distance) * (1.0 / (obstacleDistance1 * obstacleDistance1));
      float angle1 = -M_PI_2; // Angle of the left sensor w.r.t. the robot's center
      repulsiveForceX += repulsiveMagnitude1 * cos(angle1);
      repulsiveForceY += repulsiveMagnitude1 * sin(angle1);
    }

    /*if (obstacleDistance2 < safety_distance) {
    float repulsiveMagnitude2 = k_rep * (1.0 / obstacleDistance2 - 1.0 / safety_distance) * (1.0 / (obstacleDistance2 * obstacleDistance2));
    float angle2 = atan2(0, 0); // Angle of the middle sensor w.r.t. the robot's center
    repulsiveForceX += repulsiveMagnitude2 * cos(angle2);
    repulsiveForceY += repulsiveMagnitude2 * sin(angle2);
    }*/

    if (obstacleDistance3 < safety_distance) {
      float repulsiveMagnitude3 = k_rep * (1.0 / obstacleDistance3 - 1.0 / safety_distance) * (1.0 / (obstacleDistance3 * obstacleDistance3));
      float angle3 = M_PI_2;// Angle of the right sensor w.r.t. the robot's center
      repulsiveForceX += repulsiveMagnitude3 * cos(angle3);
      repulsiveForceY += repulsiveMagnitude3 * sin(angle3);
    }

    // Calculate total forces
    float totalForceX = attractiveForceX + repulsiveForceX;
    float totalForceY = attractiveForceY + repulsiveForceY;

    // Calculate desired velocities
    linearVelocity = Kp_linear * sqrt(totalForceX * totalForceX + totalForceY * totalForceY);
    angularVelocity = Kp_angular * (atan2(totalForceY, totalForceX) - Theta);

    // Cap the velocities based on stored speed
    linearVelocity = linearVelocity * ((float)storedSpeed/10);
    angularVelocity = angularVelocity * ((float)storedSpeed/10);

    float maxLinearVelocity = 0.1822;
    float maxAngularVelocity = 2.2773;

    linearVelocity = fminf(linearVelocity, maxLinearVelocity);
    angularVelocity = fminf(fabsf(angularVelocity), maxAngularVelocity)*(angularVelocity);

    // Convert velocities to wheel speeds
    leftWheelVel = linearVelocity - (wheelBase / 2.0) * angularVelocity;
    rightWheelVel = linearVelocity + (wheelBase / 2.0) * angularVelocity;
    DataSPI();

    // Check if the robot has reached the target
    if (abs(targetX - X) < 0.04 && abs(targetY - Y) < 0.04) {
        // Stop the robot
        leftWheelVel = 0;
        rightWheelVel = 0;
        DataSPI();

        // Adjust orientation
        while (abs(targetTheta - Theta) > 0.5) {
            angularVelocity = Kp_angular * (targetTheta - Theta);
            angularVelocity = fminf(angularVelocity, (float)storedSpeed);

            leftWheelVel = -angularVelocity;
            rightWheelVel = angularVelocity;

            DataSPI();
        }

        // Stop the robot once orientation is adjusted
        leftWheelVel = 0;
        rightWheelVel = 0;
        DataSPI();
    }
}

// Function to generate JSON response
String getJsonResponse() {
    DynamicJsonDocument doc(1024);
    doc["x"] = x;
    doc["y"] = y;
    doc["theta"] = theta;
    doc["angularVelocity"] = w;
    String response;
    serializeJson(doc, response);
    return response;
}