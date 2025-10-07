#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char EMERGENCY_NUMBER[] = "+919096241905";

const float ACCEL_THRESHOLD = 50.47;
const float GYRO_THRESHOLD = 3.00;

const unsigned long DEBOUNCE_DELAY = 5000;
const unsigned long GPS_TIMEOUT = 30000;
const unsigned long GSM_TIMEOUT = 5000;

const int GPS_RX_PIN = 4;
const int GPS_TX_PIN = 5;
const int GSM_RX_PIN = 7;
const int GSM_TX_PIN = 8;

enum SystemState {
  STATE_IDLE,
  STATE_ACCIDENT_DETECTED,
  STATE_SENDING_ALERT,
  STATE_ALERT_SENT
};
SystemState currentState = STATE_IDLE;

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN);

unsigned long lastTriggerTime = 0;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);

  Serial.println("Crash Node System Initializing...");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Halting.");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("System Ready. Monitoring for impact.");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  switch (currentState) {
    case STATE_IDLE:
      if (checkForAccident()) {
        if (millis() - lastTriggerTime > DEBOUNCE_DELAY) {
          lastTriggerTime = millis();
          currentState = STATE_ACCIDENT_DETECTED;
          Serial.println("STATE: Accident Detected! Debounce timer started.");
        }
      }
      break;

    case STATE_ACCIDENT_DETECTED:
      if (millis() - lastTriggerTime > DEBOUNCE_DELAY) {
        currentState = STATE_SENDING_ALERT;
      }
      break;

    case STATE_SENDING_ALERT:
      Serial.println("STATE: Sending Emergency Alert...");
      triggerEmergencyResponse();
      currentState = STATE_ALERT_SENT;
      break;

    case STATE_ALERT_SENT:
      Serial.println("STATE: Alert Sent. System halted until reset.");
      while(true);
      break;
  }
}

bool checkForAccident() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelMagnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
  float gyroMagnitude = sqrt(pow(g.gyro.x, 2) + pow(g.gyro.y, 2) + pow(g.gyro.z, 2));

  if (accelMagnitude > ACCEL_THRESHOLD || gyroMagnitude > GYRO_THRESHOLD) {
    return true;
  }
  return false;
}

void triggerEmergencyResponse() {
  Serial.println("Attempting to get GPS lock...");
  unsigned long gpsStartTime = millis();
  while (millis() - gpsStartTime < GPS_TIMEOUT) {
    while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
    if (gps.location.isValid() && gps.location.age() < 2000) {
      break;
    }
  }

  String smsMessage = "Emergency! Accident detected at: ";
  if (gps.location.isValid()) {
    smsMessage += "http://maps.google.com/maps?q=";
    smsMessage += String(gps.location.lat(), 6);
    smsMessage += ",";
    smsMessage += String(gps.location.lng(), 6);
  } else {
    smsMessage += "Last known location not available.";
    Serial.println("WARNING: Could not get a valid GPS lock.");
  }

  sendSMS(smsMessage);
  delay(5000);
  makeCall();
}

bool waitForGsmResponse(String targetResponse) {
  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < GSM_TIMEOUT) {
    if (gsmSerial.available()) {
      char c = gsmSerial.read();
      response += c;
      if (response.indexOf(targetResponse) != -1) {
        return true;
      }
    }
  }
  Serial.print("GSM TIMEOUT. Response: "); Serial.println(response);
  return false;
}

void sendSMS(String message) {
  Serial.print("Sending SMS...");
  gsmSerial.println("AT+CMGF=1");
  if (!waitForGsmResponse("OK")) {
    Serial.println("Failed to set text mode.");
    return;
  }

  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(EMERGENCY_NUMBER);
  gsmSerial.println("\"");
  if (!waitForGsmResponse(">")) {
    Serial.println("Failed to initiate SMS.");
    return;
  }

  gsmSerial.print(message);
  delay(100);
  gsmSerial.write(26);

  if (waitForGsmResponse("OK")) {
    Serial.println("SMS Sent Successfully.");
  } else {
    Serial.println("SMS sending failed.");
  }
}

void makeCall() {
  Serial.print("Making call...");
  gsmSerial.print("ATD");
  gsmSerial.print(EMERGENCY_NUMBER);
  gsmSerial.println(";");
  delay(20000);
  gsmSerial.println("ATH");
  if (waitForGsmResponse("OK")) {
    Serial.println("Call Ended.");
  } else {
    Serial.println("Failed to hang up call.");
  }
}