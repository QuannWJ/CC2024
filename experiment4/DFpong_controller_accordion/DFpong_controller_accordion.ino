/*******************************************************************************
 * Experiment 4: Proximities - Pong Edition
 * WENJIA QUAN
 * for Creation and Computation OCADU 2024
 * Use Ultrasonic Distance Sensor to detect the pull of accordion pull, when pull accordion, control paddle down; when push accordion, control paddle up.
 * 
 * Real-time distance and motion tracking system using rolling average smoothing.
 * Calculates instantaneous motion and accumulates total movement distance.
 * 
 * Key Control Variables:
 * readInterval    = 30ms   - Main processing/output interval
 * AVERAGE_WINDOW  = 5      - Number of samples in rolling average
 * MOTION_THRESHOLD = 0.3cm - Minimum change to register as movement
 * 
 * Output Format:
 * Raw Distance (cm) | Smoothed Distance (cm) | Current Motion (cm) | 
 * Motion State (TOWARD/AWAY/STILL) | Total Motion (cm)
 * 
 * Hardware Setup:
 * TRIGGER_PIN = 2
 * ECHO_PIN = 3

 * Resources from:
 * DFpong_controller_2button: https://github.com/DigitalFuturesOCADU/CC2024/tree/main/experiment4/Arduino/BLE/DFpong_controller_2button
 * distance_raw_smoothed: https://github.com/DigitalFuturesOCADU/CC2024/blob/main/experiment4/Arduino/Sensors/Distance/distance_raw_smoothed/distance_raw_smoothed.ino
 *******************************************************************************/

#include <HCSR04.h>
#include <ArduinoBLE.h>
#include "ble_functions.h"
#include "buzzer_functions.h"

const char* deviceName = "Accordion";

#define TRIGGER_PIN 2
#define ECHO_PIN 3

const int BUZZER_PIN = 11;       // Pin for haptic feedback buzzer
const int LED_PIN = LED_BUILTIN; // Status LED pin

// Movement state tracking
int currentMovement = 0;         // Current movement value (0=none, 1=up, 2=down, 3=handshake)

UltraSonicDistanceSensor distanceSensor(TRIGGER_PIN, ECHO_PIN);

// Variables
float distance = 0.0f;
float smoothedDistance = 0.0f;
float lastSmoothedDistance = 0.0f;
float totalMotion = 0.0f;
unsigned long lastReadTime = 0;
const unsigned int readInterval = 30;
const float MOTION_THRESHOLD = 0.3;

// Rolling average
const int AVERAGE_WINDOW = 5;
float readings[AVERAGE_WINDOW];
int readIndex = 0;
float totalValue = 0;

void setup() {
  Serial.begin(9600);

  // Configure button pins with internal pullup resistors
  // Buttons will read LOW when pressed, HIGH when released
  // Configure LED for connection status indication
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize Bluetooth Low Energy with device name and status LED
  setupBLE(deviceName, LED_PIN);
  
  // Initialize buzzer for feedback
  setupBuzzer(BUZZER_PIN);

  for(int i = 0; i < AVERAGE_WINDOW; i++) readings[i] = 0;
  Serial.println("Distance Motion Detection Started");
  Serial.println("--------------------------------");
}

void processDistance() {
  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= readInterval) {
    float newDistance = distanceSensor.measureDistanceCm();
    
    if (newDistance > 0) {
      distance = newDistance;
      
      // Update rolling average
      totalValue = totalValue - readings[readIndex];
      readings[readIndex] = distance;
      totalValue = totalValue + distance;
      readIndex = (readIndex + 1) % AVERAGE_WINDOW;
      lastSmoothedDistance = smoothedDistance;
      smoothedDistance = totalValue / AVERAGE_WINDOW;
      
      // Calculate motion
      float change = smoothedDistance - lastSmoothedDistance;
      totalMotion += abs(change);
      
      // Print verbose status
      Serial.print("Raw Distance: ");
      Serial.print(distance);
      Serial.print(" cm | ");
      
      Serial.print("Smoothed: ");
      Serial.print(smoothedDistance);
      Serial.print(" cm | ");
      
      Serial.print("Current Motion: ");
      Serial.print(abs(change));
      Serial.print(" cm | ");
      
      Serial.print("Total Motion: ");
      Serial.print(totalMotion);
      Serial.println(" cm");
    }
    lastReadTime = currentTime;
  }
}

void loop() {
    // Update BLE connection status and handle incoming data
  updateBLE();
  
  //read the inputs te determine the current state
  //results in changing the value of currentMovement
  handleInput();

  //send the movement state to P5  
  sendMovement(currentMovement);

  //make the correct noise
  updateBuzzer(currentMovement);

  processDistance();
}

void handleInput() 
{
//flipped read method because of INPUT_PULLUP 
  if (smoothedDistance > 10)     // pull the accordion
  {
    currentMovement = 1;         // DOWN movement
  } 
  else if (smoothedDistance < 6) // push the accordion
  {
    currentMovement = 2;         // Up movement
  } 
  else 
  {
    currentMovement = 0;         // No movement
  }
}