#include <QTRSensors.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <VL53L0X.h>
#include <Servo.h>

//Declare the sensors pins
//
Servo orange, white, laserSensor;
VL53L0X sensor0;
const int ENA = 2;
const int ENB = 3;
const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 6;
const int IN4 = 7;
unsigned char SensePin[1] = {9};
const int OrangePin = 12;
const int WhitePin = 11;
const int IR_frontL = 8;
const int IR_frontR = 10;
const int TS = 13;
int forward[] = {HIGH, LOW, HIGH, LOW};
int backward[] = {LOW, HIGH, LOW, HIGH};
int sharpleft[] = {HIGH, LOW, LOW, HIGH};
int sharpright[] = {LOW, HIGH, HIGH, LOW};
const int max_speed = 255;
int left_speed, right_speed;
float proportional, integral, derivative, last_proportional, last_error , error_value = 0;
int threshold = 300;
int current_position, target_position, delta_position = 0;
QTRSensorsRC qtrrc(SensePin, 1, 2500, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[1];
int east, west, north,south = 0;
bool turn2 = false;
bool init_turn = false;
int dropCount = 0;

// set IR state to HIGH because of pull up pin, set QTR pin to LOW
int stateL = 1;
int stateR = 1;
int IRstate = 1;
int lastIRState = 1;
int QTRstate = 0;
int lastQTRstate = 0;
int QTRcounter = 0;

//set touch sensor counter, state and previous state to zero. Set or adjust Debounce time to use it as a timer for the code ( similar to delay but more efficient as it does not require pausing
// the arduino)

int TScounter = 0;
int TS_state = 0;
int TSlastState = 0;
unsigned long lastDebounceTime = 0;
unsigned long DebounceTime = 20;
int navTurn = 1;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IR_frontR, INPUT);
  pinMode(IR_frontL, INPUT);
  pinMode(TS, INPUT);
  sensor0.init();
  sensor0.setTimeout(500);
  sensor0.startContinuous();
  mag.init();
  mag.enableDefault();  //enable magnetometer
  mag.read();
  compass(abs(mag.m.x),abs(mag.m.y));
  //Servos attach
  orange.attach(OrangePin);
  white.attach(WhitePin);
  //Initial servos position
  white.write(65);  //angle adjust to 65 degree
  orange.write(70); //angle adjust to 70 degree


}

// P and D are only used. P will damp the osciallation based on the difference of current value and set point value. D will damp osciallation based on the rate of change on the curve. 
// I value takes too long to generate and has lesser impact to the code.  Refer to osciallation curves and differential equation for better understand and more accurate way of 
// damping the osciallation. 

void PID_cal(float Kp, float Kd, int sensor_read, int setpoint) {
  proportional = sensor_read - setpoint ;
  derivative = proportional - last_proportional;
  last_proportional = proportional;
  error_value = int (proportional * Kp + derivative * Kd);

}
void calc_turn() {
  if (error_value < -255) {
    error_value = -255;
  }
  if (error_value > 255) {
    error_value = 255;
  }
  if (error_value < 0) {
    left_speed = max_speed + error_value;
    right_speed = max_speed;
  } else {
    left_speed = max_speed;
    right_speed = max_speed - error_value;
  }
}
void navigate(int left_motor, int right_motor, int Direction[]) {

  analogWrite(ENB, right_motor);
  analogWrite(ENA, left_motor);
  digitalWrite(IN1, Direction[0]);
  digitalWrite(IN2, Direction[1]);
  digitalWrite(IN3, Direction[2]);
  digitalWrite(IN4, Direction[3]);
}

void WF_D1() {
  int val = sensor0.readRangeContinuousMillimeters();
  PID_cal (3,7, val, 220);
  calc_turn();
  navigate(left_speed, right_speed, forward);
}
//wall follower distance 2
void WF_D2() {
  int val = sensor0.readRangeContinuousMillimeters();
  PID_cal (5, 10, val, 40);
  calc_turn();
  navigate(left_speed, right_speed, forward);
}

//navigation 1

//navigation 1
void nav_L1() {
  turn();
  WF_D1();
}
//navigation 2
void nav_L2() {
  turn();
  if (init_turn) {
    read_lines();
  } else {
    WF_D2();
  }
}

void read_lines() {
  qtrrc.read(sensorValues);
  if (sensorValues[0] > 1000) {
    turn2 = true;
  } else {
    turn2 = false;
  }
  if (turn2 && init_turn) {
    navigate(200, 200, sharpright); //initiate the second turn
    delay(800);
    init_turn = false;
  } else if (turn2 == false && init_turn) {
    WF_D1();
  }

}


void sweep(char drop_servo, int pos) {
  int interval = 100;
  unsigned long lastUpdate;
  if (drop_servo == 'w' || drop_servo == 'W') {
    if ((millis() - lastUpdate) > interval) {
      lastUpdate = millis();
      white.write(pos);
    }
  } else if (drop_servo == 'o' || drop_servo == 'O') {
    if ((millis() - lastUpdate) > interval) {
      lastUpdate = millis();
      orange.write(pos);
    }
  }
}
void servoDrop() {
  int interval = 2;
  unsigned long lastUpdate;
  TS_state = digitalRead(13);
  if (TS_state != TSlastState) {
    if ((millis() - lastUpdate) > interval) {
      lastUpdate = millis();
      if(TS_state == HIGH){
        if (TScounter == 0) {
          sweep('W', 0);
          navigate(0, 0, forward);
          delay(1000);
          TScounter++;
          if(dropCount = 2){
            dropCount++;
            TS_state = LOW;
          }
         }
         if (TScounter == 1 && dropCount == 6) {
          sweep('o', 180);
          navigate(0, 0, forward);
          delay(1000);
          if (dropCount == 6) {
            dropCount = 0;
           }
        }
       }TSlastState = TS_state; 
    } 
 }
}
void turn() {
 
  stateL = digitalRead(IR_frontR);
  stateR = digitalRead(IR_frontL);
  if (stateL == LOW && stateR == LOW) {
    IRstate = LOW;
  } else {
    IRstate = HIGH;
  }
  if (stateL == HIGH && stateR == HIGH) {
    IRstate = HIGH;
  } else if (stateL == LOW && stateR == LOW) {
    IRstate = LOW;
  } else {
    IRstate = HIGH;
  }

// only detect when IR sensor reads from HIGH to LOW (pull up.) This method will prevent the arduino to read the HIGH state for more than once during a certain time period. 
// For example: 
// During period t0 to t1, the arduino collects 20 samples. The code will still work but with less consistency and higher delay as it doesnt know what sample to collect. 
// This method will be used many times in the code. 

  if (IRstate != lastIRState) {
    if ((millis() - lastDebounceTime) > DebounceTime) {
      lastDebounceTime = millis();
      if (IRstate == LOW) {
        /*navigate(150, 150, backward);
        delay(400);
        navigate(255, 255, sharpright);
        delay(550);
        init_turn = true;*/
        if(navTurn == 3 && dropCount == 0){
          dropCount++;
        }else if (navTurn < 3){
          navigate(150, 150, backward);
          delay(400);
          navigate(255, 255, sharpright);
          delay(550);
          init_turn = true;
          navTurn++;
          }
      }
      lastIRState = IRstate;
      }
    }else if(dropCount == 1) {
      navigate(150,150, backward);
      delay(250);
      navigate(255, 255, sharpleft);
      delay(260);
      navigate(255,255,forward);
      delay(500);
      if(dropCount == 1){
        dropCount++;
      }
  }else if(dropCount == 2){
    servoDrop();
  }else if (dropCount == 3) {
    navigate(150, 150, backward);
    delay(400);
    navigate(255, 255, sharpright);
    delay(1000);
    if (dropCount == 3) {
      dropCount++;
    }
  }else if(dropCount == 4){
   qtrrc.read(sensorValues);
    if(sensorValues[0] > 1000){
     QTRstate = HIGH;
    }else{
     QTRstate = LOW;
    }
    if(QTRstate != lastQTRstate) {
      if ((millis() - lastDebounceTime) > DebounceTime) {
          lastDebounceTime = millis();
        if(QTRstate == HIGH){
          if(QTRcounter == 0){
            QTRcounter++;
          }else if(QTRcounter == 1){
            QTRcounter++;
          }
       }
        lastQTRstate = QTRstate;
      }
    }
    if(dropCount == 4 && QTRcounter == 2){
      dropCount++;
    }else{
      WF_D1();
    }
  }else if(dropCount == 5){   
    navigate(150,150,forward);
    delay(600);
    navigate(255, 255, sharpleft);
    delay(450);
    navigate(255,255,forward);
    delay(500);
    if(dropCount == 5){
      dropCount ++; 
    }
  }else if(dropCount == 6){
    servoDrop();
  }else{
    WF_D1();
  }

}



void loop() {
 
  //servoDrop();
  turn();
  //qtrrc.read(sensorValues);
  /*Serial.print(navTurn);
  Serial.print(" ");
  Serial.println(dropCount);*/
  //nav_drop();
  //Serial.println(sensorValues[0]);
 
 
}


