#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <MPU6050.h>

// Motor pins (gebruik L298N motor driver)
#define MOTOR_A_PWM 4
#define MOTOR_A_DIR1 16
#define MOTOR_A_DIR2 17
#define MOTOR_B_PWM 23
#define MOTOR_B_DIR1 18
#define MOTOR_B_DIR2 19

// PWM instellingen
#define PWM_FREQUENCY 2000
#define PWM_RESOLUTION 8

// SSD1306 display instellingen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MPU6050 sensor object
MPU6050 mpu;

// PID variabelen
float kp = 40.0;  // Proportioneel
float ki = 0.0;   // Integraal (start met 0)
float kd = 2.0;   // Differentieel

float error = 0;      // Huidige fout 
float lastError = 0;  // Vorige fout
float integral = 0;   // Integraal term
float derivative = 0; // Differentieel term
float pidOutput = 0;  // PID output

// Gewenste balans hoek (verticaal = 0 graden)
float targetAngle = 0.0;  // Gewenste hoek van de robot
float currentAngle = 0.0; // Hoek berekend uit accelerometer en gyroscoop

// Timing variabelen
unsigned long lastTime = 0; // Tijd van de laatste loop iteratie
float deltaTime = 0;        // Tijd sinds laatste iteratie (in seconden)

// Complementary filter variabelen
float alpha = 0.98;   // Filter constante
float gyroAngle = 0;  // Hoek berekend uit gyroscoop
float accelAngle = 0; // Hoek berekend uit accelerometer
float gyroRate = 0;   // Voeg deze variabele toe voor de gyro-snelheid


void stopMotors() {
  ledcWrite(MOTOR_A_PWM, 0);
  ledcWrite(MOTOR_B_PWM, 0);
}

void setMotorSpeed(int speedA, int speedB) {
  ledcWrite(MOTOR_A_PWM, speedA);
  ledcWrite(MOTOR_B_PWM, speedB);
}

void setMotorDirection(bool motorA_forward, bool motorB_forward) {
  // Motor A richting
  if (motorA_forward) {
    digitalWrite(MOTOR_A_DIR1, HIGH);
    digitalWrite(MOTOR_A_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_A_DIR1, LOW);
    digitalWrite(MOTOR_A_DIR2, HIGH);
  }
  
  // Motor B richting  
  if (motorB_forward) {
    digitalWrite(MOTOR_B_DIR1, HIGH);
    digitalWrite(MOTOR_B_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_B_DIR1, LOW);
    digitalWrite(MOTOR_B_DIR2, HIGH);
  }
}

void readSensorData() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Converteer naar graden
  accelAngle = atan2(ax, az) * 180 / PI;
  gyroRate = gy / 131.0; // LSB per graad/sec voor ±250°/s
  
  gyroAngle += gyroRate * deltaTime;
}

void calculateAngle() {
  // Gyro hoek integratie
  gyroAngle += gyroRate * deltaTime;  
  // Bereken accelerometer hoek
  float accelAngle = atan2(mpu.getAccelerationX(), mpu.getAccelerationZ()) * 180 / PI;
  // Combineer met complementary filter
  currentAngle = alpha * (currentAngle + gyroRate * deltaTime) + (1 - alpha) * accelAngle;
  // Begrens de hoek tussen -180 en 180 graden
  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;
}

void calculatePID() {
  // Bereken error
  error = targetAngle - currentAngle;
  
  // Proportioneel deel
  float proportional = kp * error;
  
  // Integraal deel (met windup beveiliging)
  integral += error * deltaTime;
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  float integralTerm = ki * integral;
  
  // Differentieel deel
  derivative = (error - lastError) / deltaTime;
  float differentialTerm = kd * derivative;
  
  // Combineer alles
  pidOutput = proportional + integralTerm + differentialTerm;
  
  // Begrens output
  if (pidOutput > 255) pidOutput = 255;
  if (pidOutput < -255) pidOutput = -255;
  
  lastError = error;
}

void moveMotors(float motorSpeed) {
  // Als robot te ver kantelt, stop motoren
  if (abs(currentAngle) > 45) {
    stopMotors();
    return;
  }
  
  int speed = abs(motorSpeed);      // Zet snelheid positief voor PWM
  
  if (motorSpeed > 0) {
    // Vooruit
    setMotorDirection(true, true);  // Beide motoren vooruit
    setMotorSpeed(speed, speed);
  } else if (motorSpeed < 0) {
    // Achteruit  
    setMotorDirection(false, false); // Beide motoren achteruit
    setMotorSpeed(speed, speed);
  } else {
    // Stop
    stopMotors();
  }
}

void calibrateSensor() {
  float sumAccel = 0;   // Voor accelerometer kalibratie  
  float sumGyro = 0;    // Voor gyroscoop kalibratie
  int samples = 1000;   // Aantal samples voor kalibratie
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    sumAccel += atan2(ax, az) * 180 / PI;
    sumGyro += gy / 131.0;
    delay(2);
  }
  
  // Zet kalibratie offset
  targetAngle = sumAccel / samples;
  gyroAngle = targetAngle;
  currentAngle = targetAngle;
  
  Serial.print("Kalibratie voltooid. Balans hoek: ");
  Serial.println(targetAngle);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialiseer OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // I2C adres 0x3C
    Serial.println(F("SSD1306 niet gevonden!"));
    while(1);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Segway!");
  display.display();
  delay(1000);
  
  // Initialiseer MPU6050
  Serial.println("Initialiseren MPU6050...");
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 verbinding succesvol");
  } else {
    Serial.println("MPU6050 verbinding gefaald!");
    while(1);
  }
  
  // Kalibreer de sensor (robot moet stil staan)
  Serial.println("Kalibreren... Houd robot stil!");
  delay(3000);
  calibrateSensor();
  
  // Setup motor PWM kanalen (nieuwe API)
  ledcAttach(MOTOR_A_PWM, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_B_PWM, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Setup motor richting pins
  pinMode(MOTOR_A_DIR1, OUTPUT);
  pinMode(MOTOR_A_DIR2, OUTPUT);
  pinMode(MOTOR_B_DIR1, OUTPUT);
  pinMode(MOTOR_B_DIR2, OUTPUT);
  
  Serial.println("Balancerende robot gestart!");
  lastTime = millis();
}

void loop() {
  // Bereken tijd sinds laatste iteratie
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Lees sensor data
  readSensorData();
  
  // Bereken huidige hoek met complementary filter
  calculateAngle();
  
  // Bereken PID output
  calculatePID();
  
  // Beweeg motoren
  moveMotors(pidOutput);
  
  // Debug output + OLED display (elke 100ms)
  static unsigned long debugTime = 0;
  if (currentTime - debugTime > 100) {
    Serial.print("Hoek: ");
    Serial.print(currentAngle);
    Serial.print(" | PID: ");
    Serial.print(pidOutput);
    Serial.print(" | Error: ");
    Serial.println(error);
    debugTime = currentTime;

    // OLED update
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.print("Hoek: ");
    display.println(currentAngle, 1);
    display.print("PID:  ");
    display.println(pidOutput, 1);
    display.display();
  }

  
  delay(10); // 100Hz update rate
}

