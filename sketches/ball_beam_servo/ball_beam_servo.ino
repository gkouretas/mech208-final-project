#include<Servo.h>
#include<PID_v1.h>
#include <LiquidCrystal.h>

// LCD constants
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Sensor Pins
const int trigPin = 14;
const int echoPin = 15;

// Moving Average
const int numSamples = 5;
int sampleIndex = 0;
int samples[numSamples];
double averageDist;

// Servo Pins
const int servoPin = 9;
Servo myServo;
 
// PID variables
const int KpPin = A4;
const int KiPin = A5;
const int KdPin = A6;
const int inputPin = A7;
const int stateToggle = 19;
float Kp = 2.5;
float Ki = 0.0;
float Kd = 1.1;
double setPoint, input, output, servoOutput;                                       
PID myPid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

void lcdSetup() {
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Balance and Float!");
  lcd.setCursor(0, 0);
  lcd.print("                ");
}

void movingAvgSetup() {
  Serial.println("Averaging Setup");
  // initialize rolling window with zeros
  for (int i = 0; i < numSamples; i++) {
    samples[i] = 0;
  }
}

void sensorSetup() {
  Serial.println("Sensor Setup");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void servoSetup() {
  Serial.println("Servo Setup");
  myServo.attach(servoPin); 
  myServo.write(90);       // Set beam horizontal
}

void pidSetup() {
  Serial.println("PID Setup");
  input = getDistance();
  myPid.SetMode(AUTOMATIC);
  myPid.SetSampleTime(30);   // Changes from default of 100ms to 30ms
  myPid.SetOutputLimits(-90, 90);  // servo limits
}

void setup() {
  Serial.begin(9600);

  lcdSetup();
  // movingAvgSetup();
  sensorSetup();
  servoSetup();
  pidSetup();
}

void loop() {
  setPoint = 15;
  input = getDistance();                                            
  
  myPid.Compute();
  servoOutput = 90 + output; // horizontal + (-output)
  myServo.write(servoOutput);

  lcdPrint();
  serialPrint();
  delay(500);
}
      
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration / (29*2);
  if(distance > 30) {                 // 30 cm is the maximum position for the ball
    distance = 30;
  }

  samples[sampleIndex] = distance;
  sampleIndex = (sampleIndex + 1) % numSamples;
  int sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += samples[i];
  }
  averageDist = sum / numSamples;
  return averageDist;
}

void lcdPrint() {
  lcd.setCursor(0, 0);
  lcd.print("Dist/Kp/Ki/Kd ");
  lcd.setCursor(0, 1);
  lcd.print(input);
  lcd.print("/");
  lcd.print(Kp);
  lcd.print("/");
  lcd.print(Ki);
  lcd.print("/");
  lcd.print(Kd);
}

void serialPrint() {
  Serial.print(millis() / 1000);
  Serial.print(", ");
  Serial.print(input);
  Serial.print(", ");
  Serial.print(output);
  Serial.print(", ");
  Serial.println(servoOutput);
}
