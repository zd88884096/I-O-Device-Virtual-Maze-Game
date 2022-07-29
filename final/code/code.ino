int dirs[5] = {1,0,-1,0,1};
long tune_delay = 500;
int tune1[] = {300, 500};
int tune2[] = {400, 500, 600, 800};
long vibrate_delay = 300;
float calx_arr[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float caly_arr[] = {0.0, 0.0, 0.0, 0.0, 0.0};
int stable_num = 0;
int ack = 0;
/*
  Code for Reading Values from Accelerometer is adapted from https://electropeak.com/learn/interfacing-gy-521-mpu6050-3-axis-accelerometer-gyroscope-with-arduino/
  *
  * GY-521 (MPU6050) 3-Axis Accelerometer-Gyroscope Sensor
  * modified on 28 Sep 2020
  * by Mohammad Reza Akbari @ Electropeak
  * Home

  * Based on Adafruit Example

  Code for Getting Distance from Ultrasound adopted from Prof. Lopes' provided code in the PID balance assignment
*/
#define ECHO 4 // attach pin D5 Arduino to pin Echo of HC-SR04
#define TRIG 5 //attach pin D6 Arduino to pin Trig of HC-SR04
#define BUZZER 6
#define BUTTON 3
#define LRA 2
// filter
#include "Filter.h" // install "MegunoLink" library
ExponentialFilter<float> FilteredDistance(10, 0);

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <SoftwareSerial.h>
SoftwareSerial MyBlue(7, 8); // Change this
Adafruit_MPU6050 mpu;
unsigned long button_press = 0;
unsigned long lastTime = 0;
float lastDistance = 0;
float raw_distance = 0;
boolean ultrasound_ongoing = false;
int ultrasound_phase = 0;
int ultrasound_frequency = 0;
unsigned long ultrasound_period = 0;
unsigned long ultrasound_last = 0;
float cal_x = 0.0, cal_y = 0.0;
float move_dist = 0.50;
boolean game_running = true;
void setup(void) {
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  MyBlue.write('a');
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Serial.println("");
  delay(100);

  pinMode(TRIG, OUTPUT);         // set ultrasonic sensor trigger pin
  pinMode(ECHO, INPUT);          // set ultrasonic sensor echo pin
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(LRA, OUTPUT);
  MyBlue.begin(9600);
  digitalWrite(LRA, LOW);
}

void loop() {
  unsigned long cur_t = millis();
  if(digitalRead(BUTTON) == HIGH){
    game_running = !game_running;
    button_press += 1;
    char cc = button_press % 2 == 0 ? 'r' : 'p';
    MyBlue.write(cc);
  }
  if(MyBlue.available()){
    char c = MyBlue.read();
    Serial.println(c);
    switch (c) {
      case 'w':
        vibrate();
        break;
      case 'a':
        vibrate();
        vibrate();
        vibrate();
        vibrate();
        break;
      case 's':
        vibrate();
        vibrate();
        vibrate();
        break;
      case 'd':
        vibrate();
        vibrate();
        break;
      case 'f':
        play_tune(tune1, 2);
        break;
      case 'e':
        play_tune(tune2, 4);
        break;
      case 'b':
        long_vibrate();
        break;
      default:
        // statements
        break;
    }
  }
  //Serial.println("ww");
  sensors_event_t a, g, temp;
  ultrasound_buzz();
  mpu.getEvent(&a, &g, &temp);
  Serial.println("outer loop acc");
  lastTime = millis();
  float acx = a.acceleration.x - cal_x, acy = a.acceleration.y - cal_y;
  float vx = 0.0, vy = 0.0, dx = 0.0, dy = 0.0;
  unsigned long loopstart = millis();
  if((abs(acx) >= 1.5 || abs(acy) >= 1.5 || abs(vx) >= 0.20 || abs(vy) >= 0.20) && (abs(dx) <= move_dist && abs(dy) <= move_dist && millis() - loopstart < 2000)){
    while((abs(acx) >= 1.5 || abs(acy) >= 1.5 || abs(vx) >= 0.20 || abs(vy) >= 0.20) && (abs(dx) <= move_dist && abs(dy) <= move_dist && millis() - loopstart < 2000)){
      ultrasound_buzz();
      mpu.getEvent(&a, &g, &temp);
      Serial.println("inner loop acc");
      unsigned long curTime = millis();
      float acx_new = a.acceleration.x - cal_x, acy_new = a.acceleration.y - cal_y;
      unsigned long diff = curTime - lastTime;
      float vx_new = vx + (acx + acx_new) / 2 * diff / 1000.0, vy_new = vy + (acy + acy_new) / 2 * diff / 1000.0;
      float dx_new = dx + (vx + vx_new) / 2 * diff / 1000.0, dy_new = dy + (vy + vy_new) / 2 * diff / 1000.0;
      lastTime = curTime;
      acx = acx_new;
      acy = acy_new;
      vx = vx_new;
      vy = vy_new;
      dx = dx_new;
      dy = dy_new;
      /*Serial.print(acx);
      Serial.print(" ");
      Serial.print(acy);
      Serial.print(" ");
      Serial.println(a.acceleration.z);*/
    }
    stable_num = 0;
  }
  else{
    if(stable_num < 1){
      calx_arr[stable_num] = acx;
      caly_arr[stable_num] = acy;
      stable_num += 1;
    }
    else if(stable_num == 1){
      for(int i = 0; i < 4; ++i){
        calx_arr[i] = calx_arr[i + 1];
        caly_arr[i] = caly_arr[i + 1];
      }
      calx_arr[4] = acx;
      caly_arr[4] = acy;
      cal_x += avg(calx_arr, 1);
      cal_y += avg(caly_arr, 1);
      stable_num += 1;
      Serial.println("Calibrate complete");
    }
  }
  cal_x = 0.0;
  cal_y = 0.0;
  if(game_running){
    boolean print_d = true;
    /*
    //Accelerometer sending movement
    if(dx >= move_dist){
      MyBlue.write('d');
    }
    else if(dx <= -1 * move_dist){
      MyBlue.write('a');
    }
    else if(dy >= move_dist){
      MyBlue.write('w');
    }
    else if(dy <= -1 * move_dist){
      MyBlue.write('s');
    }
    else{
      print_d = false;
    }
    if(print_d){
      Serial.print(" acx: ");
      Serial.print(acx);
      Serial.print(" acy: ");
      Serial.print(acy);
      Serial.print(" vx: ");
      Serial.print(vx);
      Serial.print(" vy: ");
      Serial.print(vy);
      Serial.print(" dx: ");
      Serial.print(dx);
      Serial.print(" dy: ");
      Serial.println(dy);
      
    }*/
  }
}

void ultrasound_buzz(){
  raw_distance = grab_distance();
  //Serial.println(raw_distance);
  if((int)raw_distance < 100){
    if(!ultrasound_ongoing){
      //Serial.println("start playing");
      ultrasound_last = micros();
      ultrasound_ongoing = true;
      ultrasound_phase = 1;
      ultrasound_frequency = (int)(50 + 8 * (100 - raw_distance));
      ultrasound_period = 500000 / ultrasound_frequency;
      digitalWrite(BUZZER, HIGH);
    }
    else{
      unsigned long ultrasound_cur = micros();
      if(ultrasound_cur - ultrasound_last >= ultrasound_period){
        if(ultrasound_phase == 1){
          digitalWrite(BUZZER, LOW);
          ultrasound_last = ultrasound_cur;
          ultrasound_phase = 0;
        }
        else{
          //Serial.println("AAA");
          ultrasound_last = micros();
          ultrasound_phase = 1;
          ultrasound_frequency = (int)(50 + 8 * (100 - raw_distance));
          ultrasound_period = 500000 / ultrasound_frequency;
          digitalWrite(BUZZER, HIGH);
        }
      }
    }
  }
  else{
    ultrasound_ongoing = false;
    digitalWrite(BUZZER, LOW);
  }
}

void clear_ongoing_var(){
  ultrasound_ongoing = false;
  digitalWrite(BUZZER, LOW);
}

void vibrate(){
  clear_ongoing_var();
  for(int i = 0; i < vibrate_delay / 2; ++i){
    digitalWrite(LRA, HIGH);
    delay(1);
    digitalWrite(LRA, LOW);
    delay(1);
  }
  delay(vibrate_delay);
}

void long_vibrate(){
  clear_ongoing_var();
  for(int i = 0; i < vibrate_delay * 4; ++i){
    digitalWrite(LRA, HIGH);
    delay(1);
    digitalWrite(LRA, LOW);
    delay(1);
  }
}

float avg(float arr[], int len){
  float sum = 0.0;
  for(int i = 0; i < len; ++i){
    //Serial.println(arr[i]);
    sum += arr[i];
  }
  return sum / len;
}

void play_tune(int arr[], int len){
  clear_ongoing_var();
  for(int ind = 0; ind < len; ++ind){
    long d_tot = tune_delay * 1000, d_once = 500000 / arr[ind];
    for(int i = 0; i < d_tot / d_once; ++i){
      delayMicroseconds(d_once);
      digitalWrite(BUZZER, HIGH);
      delayMicroseconds(d_once);
      digitalWrite(BUZZER, LOW);
    }
  }
}
float distance_filter(float raw_distance) {
  if (raw_distance > 25) {
    raw_distance = lastDistance;
  }
  FilteredDistance.Filter(raw_distance);
  float smooth_distance = FilteredDistance.Current();
  lastDistance = raw_distance;
  return smooth_distance;
}

float grab_distance() {
  // Clears the trigPin condition
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float duration = pulseIn(ECHO, HIGH);
  // Calculating the distance
  float distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}
