#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
float position = 0;
float positionSum = 0;
int sensorDetectCount = 0;
int lastSensorDetect = -1;

const float sensorPositions[SensorCount] = {-1.0, -0.714, -0.429, -0.143, 0.143, 0.429, 0.714, 1.0};

const int RIGHT_MOTOR_IN1 = 2;
const int RIGHT_MOTOR_IN2 = 4;
const int LEFT_MOTOR_IN3  = 16;
const int LEFT_MOTOR_IN4  = 17;

// PID variables
float Kp = 280;   
float Ki = 0;
float Kd = 20;

float error = 0;
float lastError = 0;
float integral = 0;

bool foundLine=false;

void setup() {
  // QTR configuration
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){14, 27, 26, 25, 33, 32, 35, 34}, SensorCount);
  qtr.setEmitterPin(2);

  // Motor pins
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_IN3, OUTPUT);
  pinMode(LEFT_MOTOR_IN4, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  qtr.read(sensorValues);
  Serial.print("Sensors: ");
  sensorDetectCount = 0;
  positionSum = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    int val = sensorValues[i];
    int threshold = 4000;

    if (val > threshold) {
      Serial.print("#");
      positionSum += sensorPositions[i];
      sensorDetectCount += 1;
      lastSensorDetect = i;
    } else {
      Serial.print("_");
    }
  }

  if (sensorDetectCount > 0) {
    position = positionSum / sensorDetectCount;
  } else {
    position = sensorPositions[lastSensorDetect]; // ή ενα fallback
  }

  Serial.print(" | Position: ");
  Serial.print(position);
  Serial.print(" | sensorDetectCount: ");
  Serial.print(sensorDetectCount);
  Serial.print(" | lastSensorDetect: ");
  Serial.println(lastSensorDetect);

  // ----- PID Control -----
  error = -position; // αν το ρομπότ στρίβει λάθος, βάλε: error = -position;
  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;



  // Ταχύτητα βάσης
  int baseSpeed = 80;
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Περιορισμός PWM
  leftSpeed = constrain(leftSpeed, 0, 100);
  rightSpeed = constrain(rightSpeed, 0, 100);

  //σταματαει οταν βλεπει γραμμη καθετη
  if (sensorDetectCount>5){
    leftSpeed=0;
    rightSpeed=0;
    foundLine=true;
  }

  if (foundLine){
    leftSpeed=0;
    rightSpeed=0;
  }
  
  // Εφαρμογή στα μοτέρ
  analogWrite(RIGHT_MOTOR_IN1, rightSpeed);
  analogWrite(RIGHT_MOTOR_IN2, 0);

  analogWrite(LEFT_MOTOR_IN3, leftSpeed);
  analogWrite(LEFT_MOTOR_IN4, 0);
}