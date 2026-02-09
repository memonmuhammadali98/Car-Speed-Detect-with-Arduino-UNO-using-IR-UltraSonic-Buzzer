#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -------- PIN DEFINITIONS --------

// Ultrasonic
#define ULTRA1_TRIG 2
#define ULTRA1_ECHO 3
#define ULTRA2_TRIG 4
#define ULTRA2_ECHO 5

// IR Sensors (ACTIVE LOW)
#define IR1_PIN 6
#define IR2_PIN 7

// Buzzer
#define BUZZER_PIN 8

// -------- VARIABLES --------
unsigned long timeSensor1 = 0;
unsigned long timeSensor2 = 0;

bool sensor1Triggered = false;
bool sensor2Triggered = false;

const float distanceBetweenSensors = 0.1; // 10 cm
float speed = 0;

unsigned long lastDetectionTime = 0;
const unsigned long resetDelay = 2000;

// Buzzer control
unsigned long buzzerStartTime = 0;
unsigned long lastBuzzerToggle = 0;
const unsigned long buzzerDuration = 2000; // 2 seconds
const unsigned long buzzerInterval = 80;   // fast beep
bool buzzerState = false;
bool buzzerActive = false;

// Ultrasonic threshold
const int detectionDistance = 12;

// -------- SETUP --------
void setup() {

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.print("Hybrid Detector");
  delay(2000);
  lcd.clear();

  pinMode(ULTRA1_TRIG, OUTPUT);
  pinMode(ULTRA1_ECHO, INPUT);
  pinMode(ULTRA2_TRIG, OUTPUT);
  pinMode(ULTRA2_ECHO, INPUT);

  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH); // OFF (inverted)
}

// -------- LOOP --------
void loop() {

  unsigned long currentMillis = millis();

  long distance1 = getDistanceCM(ULTRA1_TRIG, ULTRA1_ECHO);
  long distance2 = getDistanceCM(ULTRA2_TRIG, ULTRA2_ECHO);

  bool ir1Detected = (digitalRead(IR1_PIN) == LOW);
  bool ir2Detected = (digitalRead(IR2_PIN) == LOW);

  bool ultra1Detected = (distance1 > 0 && distance1 < detectionDistance);
  bool ultra2Detected = (distance2 > 0 && distance2 < detectionDistance);

  // ---- SENSOR 1 FIRST ----
  if (!sensor1Triggered && (ir1Detected || ultra1Detected)) {
    timeSensor1 = micros();
    sensor1Triggered = true;
  }

  // ---- SENSOR 2 AFTER SENSOR 1 ----
  if (sensor1Triggered && !sensor2Triggered && (ir2Detected || ultra2Detected)) {
    timeSensor2 = micros();
    sensor2Triggered = true;
  }

  // ---- SPEED CALCULATION ----
  if (sensor1Triggered && sensor2Triggered) {

    unsigned long timeDiff = timeSensor2 - timeSensor1;

    if (timeDiff > 2000) { // ignore noise
      speed = distanceBetweenSensors / (timeDiff / 1000000.0);
      lastDetectionTime = currentMillis;

      // Start buzzer for 3 seconds
      buzzerStartTime = currentMillis;
      buzzerActive = true;
    }

    sensor1Triggered = false;
    sensor2Triggered = false;
  }

  // ---- RESET SPEED AFTER 2 SEC ----
  if (currentMillis - lastDetectionTime > resetDelay) {
    speed = 0;
  }

  // ---- LCD DISPLAY ----
  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(speed, 2);
  lcd.print(" m/s   ");

  lcd.setCursor(0, 1);
  if (speed > 0)
    lcd.print("OBJECT DETECTED ");
  else
    lcd.print("                ");

  // ---- BUZZER 3 SECOND CONTROL ----
  if (buzzerActive) {

    if (currentMillis - buzzerStartTime <= buzzerDuration) {

      if (currentMillis - lastBuzzerToggle >= buzzerInterval) {
        lastBuzzerToggle = currentMillis;
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState ? LOW : HIGH);
      }

    } else {
      // Stop buzzer after 3 sec
      buzzerActive = false;
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerState = false;
    }
  }

  delay(40);
}

// -------- ULTRASONIC FUNCTION --------
long getDistanceCM(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) return -1;

  return duration * 0.034 / 2;
}
