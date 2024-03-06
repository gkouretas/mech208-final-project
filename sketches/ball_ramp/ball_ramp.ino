#include <HC_SR04.h>
#include <AccelStepper.h>
#include <AS5600.h>
#include <PID_v1.h>

// HC_SR04 DISTANCE SENSOR:
#define TRIG_PIN 2
#define ECHO_PIN 3
#define ECHO_INT 0
double distance;
double lastDistance;
HC_SR04<ECHO_PIN> sensor(TRIG_PIN);

// Timing Variables
const int UPDATE_INTERVAL = 25;  // specify the update interval in milliseconds
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;  // store the time of the previous update

// PID CONTROLLER:
double targetDist = 28; // set the main target distance here
double input, output;
PID pid(&input, &output, &targetDist, 1, 0, 0, DIRECT);

// STEPPER MOTOR:
int ENABLE_PIN = 8;
int stepPin = 4;
int dirPin = 7;
#define motorInterfaceType 1
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

// TARGET SWITCHING:
long targetSwitchTime = 10 * 1000;
long nextTargetSwitchTime = millis() + targetSwitchTime;
int potPin = A3;
int potPowerPin = 12;

void sensorSetup() {
  Serial.println("Sensor Setup");
  sensor.begin(); // Start HC_SR04 sensor
}

void stepperSetup() {
  Serial.println("Stepper Setup");
  pinMode(ENABLE_PIN, OUTPUT); // CNC shield enable pin
  digitalWrite(ENABLE_PIN, LOW); // Pull the enable pin LOW to enable the stepper driver
  stepper.setMaxSpeed(200000); // Stepper motor speed and acceleration parameters (Units are steps/sec and steps/second^2, respectively)
  stepper.setAcceleration(20000);
}

void pidSetup() {
  Serial.println("PID Setup");
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(30); // set the PID control loop to run every so often
  pid.SetOutputLimits(-800, 800); // limit the PID output to a range of motor steps
  float scale = 1;
  pid.SetTunings(scale*70, scale*14, scale*165); // set the PID tuning parameters (P, I, D)
}

void homingSetup() {
  Serial.println("Homing Setup");
  int startAngle = 200; // Home position of the crank
  encAngle = (encoder.getAngle())/11.37; // Initial angle measurement
  
  while (abs(encAngle-startAngle) > 1) { 
    
    encAngle = (encoder.getAngle())/11.37; // Read angle of shaft
    Serial.println(encAngle);
    stepper.move(-20);
    stepper.run();
    delayMicroseconds(50);
  }
  stepper.setCurrentPosition(0); // Reset stepper position as 0 after homing
}

void setup() {  
  Serial.begin(9600); 
  sensorSetup();
  
  pinMode(potPowerPin, OUTPUT);
  digitalWrite(potPowerPin, HIGH);

  // initialize the samples array with zeros
  for (int i = 0; i < numSamples; i++) {
    samples[i] = 0;
  }

  stepperSetup();
  pidSetup();
  Serial.println(millis());
  homingSetup(); // Run homing function to bring the crank to its home position from an arbirtary start point
  Serial.println(millis());
}

void loop() {
  currentMillis = millis();  // get the current time
  Serial.println(currentMillis);
/*
  //// TARGET DISTANCE SWITCHING:
  //  Serial.println(currentMillis - nextTargetSwitchTime);
  if (currentMillis >= nextTargetSwitchTime)
  {
    if ((int)targetDist == 28)
      targetDist = 9.0; 
    else
      targetDist = 28.0;

    nextTargetSwitchTime = currentMillis + targetSwitchTime;
  }
*/

  //// READING THE HC_SR04 DISTANCE SENSOR:
  if (currentMillis - previousMillis >= UPDATE_INTERVAL) {  // check if it's time to take another sensor reading
    previousMillis = currentMillis;  // update the previous time

    if(sensor.isFinished()) {
      // Do something with the range...
      distance = sensor.getDist_cm();
      
      if (distance > 52) { // Simple outlier rejection to reduce the effeects of some of the erroneous sensor readings
        distance = 16;
      }

      sensor.begin(); // Start sensor for next measurement
      
      samples[sampleIndex] = distance; // store the distance in the samples array  
      sampleIndex = (sampleIndex + 1) % numSamples; // increment the sample index and wrap around if necessary
      
      // compute the sum of the samples
      int sum = 0;
      for (int i = 0; i < numSamples; i++) {
        sum += samples[i];
      }

      averageDist = sum / numSamples; // compute the average of the samples
      targetDist = map(analogRead(potPin), 0, 1023, 8, 30); // Calculate the PID target distance from the potentiometer reading
    }
  }
  Serial.print(averageDist); // Print the processed distance sensor data
  Serial.print(",");
  Serial.println(targetDist);  // Print the PID target distance as set by the potentiometer      

  input = averageDist; // Provide the processed distance sensor data as the input to the PID controller
  pid.Compute();

  Serial.println(output);
  stepper.moveTo(output); // Use the stepper motor as the output for the PID controller
  stepper.run();
}
