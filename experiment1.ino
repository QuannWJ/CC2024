/* 
The hug ball-OCADU DIGF-6037 Creation & Computation

2 sensors and an LED string
When the value of the sensor (either of two) exceeds its threshold value, ALL LEDs lights up
the brightness goes from 0 to 255 and back to 0 in 10 seconds

Based on:
https://github.com/DigitalFuturesOCADU/CC2024/blob/main/experiment1/AnalogReadSerial5values/AnalogReadSerial5values.ino
https://github.com/DigitalFuturesOCADU/CC2024/blob/main/experiment1/fsr_led_thresholds/fsr_led_thresholds.ino
https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogWriteMega

*/
const int sensorPin1 = A1; // Analog input pins
const int sensorPin2 = A2;
const int ledPin = 2; // LED output pins PWM Pins

const int threshold1 = 900; // Individual thresholds for each analog input
const int threshold2 = 700;
const int fadeTime = 10;

int sensorValue1 = 0; 
int sensorValue2 = 0; 

void setup() {
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT); 
  pinMode(ledPin, OUTPUT); 
}

void loop() {
  // read the input on analog pin
  sensorValue1 = analogRead(sensorPin1);
  Serial.print(sensorValue1); // print the value
  Serial.print(", ");
  sensorValue2 = analogRead(sensorPin2);
  Serial.print(sensorValue2); 
  Serial.print(", ");
  delay(50);
  
  // either one of two sensors is touched
  if (sensorValue1 > threshold1 || sensorValue2 > threshold2) {
    // fade the LED from off to brightest
     for (int brightness = 0; brightness <= 255; brightness++) {
      analogWrite(ledPin, brightness); 
      delay(fadeTime); 
    }
    for (int brightness = 255; brightness >= 0; brightness--) {
      analogWrite(ledPin, brightness); 
      delay(fadeTime); 
    }
  } else {
    analogWrite(ledPin, 0); // no sensor touched
  }
}