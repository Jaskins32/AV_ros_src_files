//#include <MCP492X.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
////////////////// Tick Data Publishing Variables and Constants ///////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
//#define ENC_IN_LEFT_A 2
#define ENC_RPM 3 //was enc_in_right_A

#define FEEDBACKPIN A0
unsigned char minDegrees;
unsigned char maxDegrees;
unsigned char maxFeedback;
unsigned char minFeedback;
unsigned char tolerance = 1;
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
//#define ENC_IN_LEFT_B 4
//#define ENC_IN_RIGHT_B 11

Servo ESC;
Servo steer;

//#define PIN_SPI_CHIP_SELECT_DAC 9
//MCP492X myDac(PIN_SPI_CHIP_SELECT_DAC); 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks

std_msgs::Int16 drive_motor_tick_count;
ros::Publisher drivePub("drive_ticks", &drive_motor_tick_count);
std_msgs::Int16 drive_motor_velo;
ros::Publisher veloPub("drive_velo", &drive_motor_velo);
//std_msgs::Int16 left_wheel_tick_count;
//ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 
////////////////// Motor Controller Variables and Constants ///////////////////
 
// Motor A connections
//const int enA = 9;
//const int in1 = 5;
//const int in2 = 6;
  
// Motor B connections
//const int enB = 10; 
//const int in3 = 7;
//const int in4 = 8;

 
// How much the PWM value can change each cycle
const double PWM_INCREMENT = 0.2;
 
// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 3;
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.1063625;
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.29591;
//0.365428 meters per revolution **linear distance per wheel rev** 
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 2.7365; // Originally 2880
 
// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 2;
 
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 67;
 
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;
 
// Turning PWM output (0 = min, 255 = max for PWM values)
const int STEER_TURN_RIGHT = 92;
const int STEER_TURN_LEFT = 80;
 
// Set maximum and minimum limits for the PWM values
const int ESC_MIN = 60; // about 0.1 m/s
const int ESC_MAX = 130; // about 0.172 m/s
const unsigned char STEER_MAX = 150;
const unsigned char STEER_MIN = 30;
 
// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velDriveWheel = 0;
double steerReq = 87.4;
double steerAngle = 87.4;
double ESCReq = 64;
 
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
 
/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks
void drive_motor_tick() {
  
  drive_motor_tick_count.data++;
  //Serial.println(drive_motor_tick_count.data);
}
 
// Increment the number of ticks

 
/////////////////////// Motor Controller Functions ////////////////////////////
 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is rpublished on the /left_ticks topic. 

 
// Calculate the right wheel linear velocity in m/s every time a 
// tick count message is published on the /right_ticks topic. 
void calc_vel_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + drive_motor_tick_count.data - prevRightCount) % 65535;
 
  if (numOfTicks > 25000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velDriveWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
  drive_motor_velo.data = numOfTicks;
 
  prevRightCount = drive_motor_tick_count.data;
   
  prevTime = (millis()/1000);
 
}
 
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  //Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
   
  // Calculate the PWM value given the desired velocity 
  //dacLeftReq = K_P * cmdVel.linear.x + b;
  ESCReq = K_P * cmdVel.linear.x + b;
 
  // Check if we need to turn 
  //ackerman icr code//
  if (cmdVel.angular.z != 0.0) {
 
    // Turn left
    if (cmdVel.angular.z > 0.0) {
      
      steerReq = STEER_TURN_LEFT;
    }
    // Turn right    
    else {
      //dacLeftReq = DAC_TURN;
      steerReq = STEER_TURN_RIGHT;
    }
  }
  // Go straight
  else {
    steerReq = 87.4; //straight 
    // Remove any differences in wheel velocities 
    // to make sure the robot goes straight
    /*static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velLeftWheel - velDriveWheel; 
    double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;*/
 
    // Correct PWM values of both wheels to make the vehicle go straight
    /*dacLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
    ESCReq += (int)(avgDifference * DRIFT_MULTIPLIER);*/
    
  }
 
  // Handle low PWM values
  /*if (abs(dacLeftReq) < ESC_MIN) {
    dacLeftReq = 0;
  }*/
  if (abs(ESCReq) < ESC_MIN) {
    ESCReq = 64;  
  }
}
 
void set_pwm_values() {
 
  //These variables will hold our desired PWM values
  static double steerOut = 87.4;
  static double ESCOut = 64;
 
  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if (ESCReq * velDriveWheel < 0 && ESCOut != 0) {
    //pwmLeftReq = 0;
    ESCReq = 64;
  }
 
  // Set the direction of the motors
  /*
  if (pwmLeftReq > 0) { // Left wheel forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  else { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); 
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  else { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW); 
  }
 */
  // Increase the required PWM if the robot is not moving
  /*if (steerReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }*/
  /*if ((ESCReq < 60 || ESCReq > 67) && velDriveWheel == 0) {
    ESCReq += 1;
  }*/
 
  // Calculate the output PWM value by making slow changes to the current value
  
  if (abs(steerReq) > steerOut) {
    steerOut += PWM_INCREMENT;
  }
  else if (abs(steerReq) < steerOut) {
    steerOut -= PWM_INCREMENT;
  }
  else{}
  
  if (abs(ESCReq) > ESCOut) {
    ESCOut += PWM_INCREMENT;
  }
  else if(abs(ESCReq) < ESCOut) {
    ESCOut -= PWM_INCREMENT;
  }
  else{}

 
  // Conditional operator to limit PWM output at the maximum 
  steerOut = (steerOut > STEER_MAX) ? ESC_MAX : steerOut;
  ESCOut = (ESCOut > ESC_MAX) ? ESC_MAX : ESCOut;
 
  // PWM output cannot be less than 0
  steerOut = (steerOut < STEER_MIN) ? STEER_MIN : steerOut;
  ESCOut = (ESCOut < ESC_MIN) ? ESC_MIN : ESCOut;
 
  // Set the PWM value on the pins
  ESC.write(ESCOut); 
  steer.write(steerOut);
  
   
  //ESC.write(67);
}

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);
 
void setup() {
 
  // Set pin states of the encoder
  //pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  //pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_RPM , INPUT_PULLUP);
  //pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  //attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  pinMode(ENC_RPM, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_RPM), drive_motor_tick, RISING);
   
  // Motor control pins are outputs
  //pinMode(enA, OUTPUT);
  //pinMode(enB, OUTPUT);
  //pinMode(in1, OUTPUT);
  //pinMode(in2, OUTPUT);
  //pinMode(in3, OUTPUT);
  //pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, LOW);
  //digitalWrite(in3, LOW);
  //digitalWrite(in4, LOW);
  
  // Set the motor speed
  //analogWrite(enA, 0); 
  //analogWrite(enB, 0);

//  myDac.begin();
//  pinMode(6, OUTPUT);
//  digitalWrite(6, LOW);
  //Serial.begin(115200);
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(drivePub);
  //nh.advertise(leftPub);
  nh.advertise(veloPub);
  nh.subscribe(subCmdVel);
  ESC.attach(9);
  steer.attach(10);
  //calibrate(steer, FEEDBACKPIN, 50, 130);
  ESC.write(64);
  steer.write(86.4);
  wait(1000);
  ESC.write(68);
  steer.write(86.4);
  wait(1000);
  ESC.write(64);
  drive_motor_tick_count.data = 0;
  //Serial.println("Setup complete");
 

}
 
void loop() {
   
  nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
    //leftPub.publish( &left_wheel_tick_count );
    drivePub.publish( &drive_motor_tick_count );
    veloPub.publish( &drive_motor_velo );
    
 
    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    //calc_vel_left_wheel();
     
  }
   
  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 1) {
    steerReq = 87.4;
    ESCReq = 64;
  }
 
  set_pwm_values();
}
void wait(int x)
{
  int currentTime = millis();
  while (millis() - currentTime <x)
  {
    ;
  }
}
/*void calibrate(Servo servo, int feedbackPin, int minPos, int maxPos)
{
  servo.write(minPos);
  minDegrees = minPos;
  wait(1000);
  minFeedback = analogRead(feedbackPin);

  servo.write(maxPos);
  maxDegrees = maxPos;
  wait(1000);
  maxFeedback = analogRead(feedbackPin);
}*/
