// Infrared speed sensor connections
#define speedSensorPin 2
#include "TimerOne.h"
#include "Arduino.h"
// Constants for Interrupt Pins // Change values if not using Arduino Uno
const byte MOTOR1 = 2;
// Motor driver connections
#define in1 3
#define in2 5
#define in3 4
#define in4 6
#define ENA 12
#define ENB 7


// Variables
unsigned int counter1 = 0; 
float diskslots = 20;
volatile unsigned long distance = 0;    // Distance traveled by the car
volatile unsigned long prevTime = 0;
volatile unsigned long distanceTraveled = 0;

void ISR_count1()
{
counter1++; // increment Motor 1 counter value
}

void ISR_timerone()
{
// TimerOne ISR void ISR_timerone()
Timer1.detachInterrupt(); // Stop the timer
attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);
Serial.print("Motor Speed: ");
float wheelCircumference = 2;
float rps = counter1 / diskslots; // calculate RPs for Motor 
float v = 2.0 * 3.14 * 0.033 * rps;
Serial.print(v);
Serial.print(" m/s");
counter1 = 0; // reset counter to zero 
Timer1.attachInterrupt( ISR_timerone ); // Enable the timer

// Get the current time
  unsigned long currentTime = millis();

  // Calculate the time elapsed since the last distance update
  unsigned long elapsedTime = currentTime - prevTime;

  // Calculate the speed in meters per second
  float Speed = (wheelCircumference / elapsedTime) * 1000.0;

  // Calculate the distance traveled since the last update
  float distance = Speed * (elapsedTime / 1000.0);

  // Add the distance to the cumulative distance traveled
  distanceTraveled += distance;

  // Update the previous time for the next calculation
  prevTime = currentTime;

  // Print the cumulative distance traveled
  Serial.print("  Distance traveled: ");
  Serial.print(distanceTraveled);
  Serial.println(" cm");

}

void speedSensorInterrupt() {
  distanceTraveled++;
}


// Function to stop the motors
void stopMotors() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void corner() {
  const float wheelDiameter = 6; // Wheel diameter in centimeters
  const float turningRadius = 50.0;   // Turning radius in centimeters (half of the corner circumference)
  // Calculate the number of revolutions needed to cover the turning radius
  float circumference = PI * wheelDiameter;
  float revolutions = turningRadius / circumference;

  // Rotate the robot car around the corner
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(700);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(200);
  
  // Delay for the calculated time to complete the turn
 unsigned long delayTime = revolutions * 1000.0; // Convert revolutions to milliseconds
  delay(delayTime/3);
  // Move the robot car forward to exit the corner
  
}

void accelerateforward() { 
  const float wheelDiameter = 6; // Wheel diameter in centimeters
  float distance = 100.0;
  // Calculate the number of revolutions needed to cover the desired distance
  float circumference = PI * wheelDiameter;
  float revolutions = distance / circumference;
     // Delay for the calculated time to cover the distance
unsigned long delayTime = revolutions * 1000.0; // Convert revolutions to milliseconds

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(delayTime);
  // Control the motor direction 
}


void setup() {
  pinMode(speedSensorPin, INPUT_PULLUP);

  // Attach an interrupt to the speed sensor pin
  attachInterrupt(digitalPinToInterrupt(speedSensorPin), speedSensorInterrupt, FALLING);

  
  // Start serial communication
  Serial.begin(9600);
  Timer1.initialize(1000000); // set timer for 1sec attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING); // Increase counter 1 when speed sensor pin goes High 
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer

  // Motor pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
}
 
void loop()
 {
  // Move the robot car forward
  accelerateforward();
  corner();
  corner();
  accelerateforward();
  
    stopMotors();
    Serial.println("Target distance reached!");
    delay(1000);
  }
