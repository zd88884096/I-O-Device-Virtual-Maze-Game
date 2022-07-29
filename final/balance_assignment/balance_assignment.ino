// ultrasonic sensor
#define ECHO 5 // attach pin D5 Arduino to pin Echo of HC-SR04
#define TRIG 6 //attach pin D6 Arduino to pin Trig of HC-SR04
float lastDistance = 0;
float raw_distance = 0;

// filter
#include "Filter.h" // install "MegunoLink" library
ExponentialFilter<float> FilteredDistance(10, 0);
// 20 is the weight (20 => 20%) the higher the close to raw data, the lower the smoother but with more delay
// 0 is the initial value of the filter
// https://www.megunolink.com/articles/coding/3-methods-filter-noisy-arduino-measurements/

// servo
#include <Servo.h>
#define SERVO 2
Servo myservo;
int servo_zero = 1400;             // servo pos when flat
int servo_pos = servo_zero;        // current servo position
int servo_min = 1300;              // minimun usable servo position
int servo_max = 1700;              // maximun usable servo position

float ball_pos = 0;                // current ball position
float target = 10;                 // target ball position in cms

// PID constants: tune these!
float proportion = 7;             // constant for proportional factor of PID controller
float integral = 2;                // constant for integral factor of PID controller
float derivative = 3;              // constant for derivative factor of PID controller
// other PID variables for computation
float error, cumError, rateError, lastError, elapsedTime;
unsigned long currentTime, previousTime;
int counter = 0;

void setup() {
  // setup serial, note that Serial.print takes a long time (~50ms)
  Serial.begin(9600);
  Serial.setTimeout(10);
  Serial.flush();
  delay(500);
  
  myservo.attach(SERVO);         // initilize servo pin
  myservo.write(servo_zero);     // set servo to flat position
  pinMode(TRIG, OUTPUT);         // set ultrasonic sensor trigger pin
  pinMode(ECHO, INPUT);          // set ultrasonic sensor echo pin

  lastDistance = grab_distance();
  previousTime = millis();
}

void loop() {        
  raw_distance = grab_distance();                         // grab distance from ultrasonic sensor
  ball_pos = distance_filter(raw_distance);               // raw sensor value can be noisy, so put it into a filter 
  
  Serial.println(ball_pos);
  if(Serial.available()){
    char c = Serial.read();
    if(c == 'p'){
      proportion = Serial.parseFloat();
    }
    else if(c == 'i'){
      integral = Serial.parseFloat();
    }
    else if(c == 'd'){
      derivative = Serial.parseFloat();
    }
    else if(c == 't'){
      target = Serial.parseFloat();
    }
    Serial.println(proportion);
    Serial.println(integral);
    Serial.println(derivative);
    Serial.println(target);
  }
  currentTime = millis();                                 //get current time to compute error rate/cumulation
  elapsedTime = float(currentTime - previousTime) / 1000; //compute time elapsed from previous computation in seconds
  error = target - ball_pos;                              // compute error (used for proportion)
  cumError += error * elapsedTime ;                       // compute cumulated error (used for integral)
  rateError = (error - lastError) / elapsedTime;          // compute error rate (used for derivative)

  // compute the adjuste postion for the servo
  servo_pos = servo_zero - (proportion * error + integral * cumError + derivative * rateError);

  // write to the servo using writeMicroseconds(), which gives us more resolution than just write()
  // keep this, this rejects sensor values outside oft the usable servo range
  if (servo_pos < servo_min) {
    myservo.writeMicroseconds(servo_min);
  } else if (servo_pos > servo_max) {
    myservo.writeMicroseconds(servo_max);
  } else {
    myservo.writeMicroseconds(servo_pos);
  }

  lastError = error; //remember current error
  previousTime = currentTime; //remember current time
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
