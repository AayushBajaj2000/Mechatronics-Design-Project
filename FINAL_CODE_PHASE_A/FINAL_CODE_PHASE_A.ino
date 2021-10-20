#include "Adafruit_VL53L0X.h"
#include <PID_v1.h>

char startWall = 'L'; // Variable for Which wall to follow
int turns = 6; // Specify when to switch from left wall to right wall. These are right turns. Used for positioning

// The right motor
// Backwards -> 1 HIGH, 2 LOW, Forwards -> 1 -> LOW, 2 -> HIGH
int motor1pin1 = 24; 
int motor1pin2 = 25;
int motorEnable = 9;

// The left motor
// Backwards -> 1 HIGH, 2 LOW, Forwards -> 1 -> LOW, 2 -> HIGH
int motor2pin1 = 26;
int motor2pin2 = 27;
int motorEnableTwo = 10;

int irPin = 53; // IR obstacle avoidance sensor

const byte encoder0pinA = 2; //A pin -> the left motor
int encoder0pinB = 4;   
const byte encoder1pinA = 3; //B pin -> The right motor
int encoder1pinB = 5; 

//Variables used
byte encoder0PinALast; //encoderPinA last pulse
int pulseCount = 0; //the number of the pulses
boolean Direction; //the rotation direction

//Variables used
byte encoder1PinALast; //encoderPinA last pulse
int pulseCount1 = 0; //the number of the pulses
boolean Direction1; //the rotation direction

int ppr = 2940; // pulse per rotation
int speedT = 255; // Speed
float pivotD = 9.2; // Pivot Diameter cm
float pi = 3.1415; // Pi
float C = pi * pivotD; // Circumference of the base diameter
float revDistance = 2 * pi * 6; // distance travelled in 1 revolution

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 8

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1; // Left
VL53L0X_RangingMeasurementData_t measure2; // Right

int baseSpeed = 100; // BaseSpeed for the wallfollow algorithm
int speedRight, speedLeft;
int counter = 0; // counter for the positioning of the robot

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=10, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:

  // Motor left pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motorEnable, OUTPUT);
  
  // Motor right pins
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motorEnableTwo, OUTPUT);  

  // Begin the serial port
  Serial.begin(57600); 
  
  // Encoders
  pinMode(encoder0pinA,INPUT_PULLUP);
  pinMode(encoder1pinA,INPUT_PULLUP);

  // Interrupt pins
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), motorOnePulse, CHANGE); //attach the interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1pinA), motorTwoPulse, CHANGE); //attach the interrupt

    // IR sensor pin
  pinMode(irPin, INPUT);

  //Shutdown pins for the LOX sensors
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  // Both in reset mode
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  setID(); // Setting the ID's for the TOF sensors to work both at the same time.

  if(startWall == 'R'){
    // If we are following the right wall then input is the right TOF sensor
    Input = measure2.RangeMilliMeter;
  } else if(startWall == 'L'){
    // If we are following the left wall then input is the right TOF sensor
    Input = measure1.RangeMilliMeter;
  }

  // Setpoint distance from the wall for the PID.
  Setpoint = 80;
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // Delay the robot start
  delay(1000);
}

void loop() {
    // Read if there is nothing in the front
    int frontVal = digitalRead(irPin);
  
    // If there is then check if there is something on the sides
    if(frontVal == 0){
      checkWall(); // Checks the wall according to which wall it is following
    } else {
        // Set the speeds according to the PID.
        setSpeeds();
        
        // Writing the speed to the motors
        analogWrite(motorEnableTwo, speedRight);
        analogWrite(motorEnable, speedLeft);
    }
}

// Check for walls on the right or left and make turns accordingly
void checkWall(){
      read_dual_sensors(); // Read both the TOF sensors
      
      if(startWall == 'R'){
        // If following the right wall
        if(measure1.RangeMilliMeter < 150){
          // If there is any wall on the left then turn 180 degrees
          moveLeft(180);
        } else {
          // If not then turn left 90 degrees
          moveLeft(70);
          counter++; // Increase the counter which keeps track of turns
        }

        //---------Robot Positioning in the Maze ---------------------//
        // If the robot is on the last turn before getting to the miner
        if(counter == turns + 1){
          // The robot has reached its last turn switch to Left wall now
          moveLeft(20);
          moveForward(0.5);
          startWall = 'L';
        } else if(counter == turns + 2){
          // The robot has reached the miner stop the robot
          startWall = 'E';
        }
        //------------------------------------------------------------//
        
      } else if(startWall == 'L' ){
        // If following the left wall
        if(measure2.RangeMilliMeter < 150){
          // If there is any wall on the right then turn 180 degrees
          moveRight(180);
        } else {
          // If not then turn right 90 degrees
          moveRight(70);
          counter++; // Increase the counter which keeps track of turns
        }

        //---------Robot Positioning in the Maze ---------------------//
        // If the robot needs to switch to right wall follow then switch
        if(counter == turns){
         moveForward(0.5);
         startWall = 'R';
        } else if(counter == turns + 2){
          // The robot has reached the miner stop the robot
          startWall = 'E';
        }
        //-----------------------------------------------------------//
      }
}

// Function that sets the speeds for the 2 motors
void setSpeeds(){
        // Measure the distance
        measureDistance();
          
        // Run the PID controller
        myPID.Compute();
        
        if(startWall == 'R'){
           // Set the speeds of both motors according to the PID. Experimentally determined
           speedLeft = baseSpeed + 200 - Output; // Right speed should be more as we want to follow the left wall.
           speedRight = baseSpeed  - 50 + Output; // Left speed should be higher when it is closer to the wall.
        } 
        else if(startWall == 'L'){
           // Set the speeds of both motors according to the PID. Experimentally determined
           speedRight = baseSpeed + 200 - Output; // Right speed should be more as we want to follow the left wall.
           speedLeft = baseSpeed  - 50 + Output; // Left speed should be higher when it is closer to the wall.
         } else {
           // Stop the robot
           speedRight = 0;
           speedLeft = 0;
         }

        // Check the range of Output as it is from 0 to 255.
        if(speedRight > 255){
          speedRight = 255;
        } else if(speedRight < 0){
          speedRight = 0;
        }
        
        // Check the range of Output as it is from 0 to 255.
        if(speedLeft > 255){
          speedLeft = 255;
        }else if(speedLeft < 0){
          speedLeft = 0;
        }

        // Set the motors to move straight and writing the speed to the motors.
        motorForward();
}

// Function for measuring the distance of the TOF sensor
void measureDistance(){
    // Take reading from both the sensors
    read_dual_sensors();

    // Set the input to the PID according to which wall we are following
    if(startWall == 'R'){
       Input = measure2.RangeMilliMeter;
    }else if(startWall == 'L'){
       Input = measure1.RangeMilliMeter;
    }
}

void motorForward(){
    // Motor direction pins
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
}

void moveForward(int turns){
    // Motor direction pins
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    checkPulse(turns);
}

void moveBackward(int turns){
    // Motor direction pins
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    checkPulse(turns);
}

void moveRight(float angle){
    // Motor direction pins
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);

    pulseCount = 0;
    
    float arc = (2 * angle / 360 ) * pi * pivotD;
    float turns = arc / revDistance;
    
    checkPulse(turns);
}

void moveLeft(float angle){
    // Motor direction pins
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);

    // Reset the pulseCount
    pulseCount = 0;
    
    float arc = (2 * angle / 360 ) * pi * pivotD;
    float turns = arc / revDistance;
    
    checkPulse(turns);
}

void checkPulse(float turns){
  int pulses = turns * ppr;

  while(abs(pulseCount) < pulses){
  Serial.print(pulseCount);    
  analogWrite(motorEnable,speedT);
  analogWrite(motorEnableTwo,speedT);
  }
  
  analogWrite(motorEnable,0);
  analogWrite(motorEnableTwo,0);
}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
}

//Function which gets the number of pulses of the motor
void motorOnePulse()
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;

  if(!Direction)  pulseCount++;
  else  pulseCount--;
}

//Function which gets the number of pulses of the second motor
void motorTwoPulse()
{
  int Lstate = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if(val == LOW && Direction1)
    {
      Direction1 = false; //Reverse
    }
    else if(val == HIGH && !Direction1)
    {
      Direction1 = true;  //Forward
    }
  }
  encoder1PinALast = Lstate;

  if(!Direction1)  pulseCount1++;
  else  pulseCount1--;
}
