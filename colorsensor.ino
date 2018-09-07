#include "Servo.h"
#include "Wire.h"
#include "Adafruit_TCS34725.h"
 
// Servo-Position
const int whitePos = 30;
const int orangePos = 140;
//const int nonePos = 81; 
 
// Servo-initialization
Servo myservo;
 
// Color Sensor-Object initialization
// Parameter siehe: https://learn.adafruit.com/adafruit-color-sensors/program-it
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
 
// setup()
void setup() {
 
 // serial print for serial monitior
 Serial.begin(9600);
 Serial.println("Color Sensor reading");
 
 // Color sensor detect or not
 if (tcs.begin()) {
 // all OK
 Serial.println("Sensor detect");
 } else {
 // Sensor not detect , stop all.
 Serial.println("Sensor not found, Stop");
 while (1); // Stop
 } 
 
 //servo wire to pin 9
 myservo.attach(9);
 
 // Servo default position
 myservo.write(75);
 
}
 
// Loop repeat as long as code is running
void loop() {
 
 // The sensor returns values​for R, G, B and a Clear value
 uint16_t clearcol, red, green, blue;
 float average, r, g, b;
 delay(500); // Color measurement takes c. 50ms
 tcs.getRawData(&red, &green, &blue, &clearcol);
 
 
 // get the average RGB 
 average = (red+green+blue)/3;
 
 // Color values​by average, all values​are now around 1
 
 r = red/average;
 g = green/average;
 b = blue/average;
 
 // Clear value and r, g, b output on serial control
 // r, g and b should be between about 0.5 and 1.5
 // move. If the sensor looks red, then r should be well above 1.0
 // lie, g and b between 0.5 and 1.0 etc.
 Serial.print("\tClear:"); Serial.print(clearcol);
 Serial.print("\tRed:"); Serial.print(r);
 Serial.print("\tGreen:"); Serial.print(g);
 Serial.print("\tBlue:"); Serial.print(b);
 
 // Attempt to determine the color using the r, g, b values.
 // The best way to start with red, green, blue are the thresholds
 // adjust with the serial output accordingly
 
 if((clearcol>250) && ( r > 0.8 && r < 1.06) && (g < 1.6) && (b >0.85)) {
 Serial.print("\tWHITE");
 delay(1000);
 //myservo.write(whitePos);
 }
 else if ((r > 1 && r<  2.7) && (g > 0.6) && (b >0.36 &&b<0.76)) {
 Serial.print("\tORANGE");
 delay(1000);
 //myservo.write(orangePos);
 } 

 else {
 Serial.print("\tNOT RECOGNIZED"); 
  //myservo.write(75);
 }
 
 
// output line break
Serial.println("");
 
//adjust wait time for serial debugging
delay(100);
 
}
