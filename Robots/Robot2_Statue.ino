///////////////
///LIBRARIES///
///////////////

// Robot that grabs statutette


// WHEELS
#include <Wire.h>                                             // Calls for I2C bus library

// ULTRASONIC SENSORS
#include <NewPing.h>

//VOLTAGE READER
#include <CheapStepper.h>

//GRIPPER
#include <Servo.h>

///////////
///VARIABLES///
///////////

// Motors bytes
#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

    
// Ultrasonic sensor pins
#define USPIN1 2
#define USPIN2 3
#define USPIN3 4
#define USPIN4 5
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
unsigned int uS1;
unsigned int uS2;
unsigned int uS3;
unsigned int uS4;

NewPing sonar[4] = {NewPing(USPIN1, USPIN1, MAX_DISTANCE), NewPing(USPIN2, USPIN2, MAX_DISTANCE), NewPing(USPIN3, USPIN3, MAX_DISTANCE), NewPing(USPIN4, USPIN4, MAX_DISTANCE)};

#define PI 3.1415926535897932384626433832795
#define len(arr) sizeof (arr)/sizeof (arr[0])


// MOTOR MOVEMENT VARIABLES
int DualSpeedValue = 0;                                       // Combined motor speed variable
int Mode = 2;                                                 // MODE in which the MD25 will operate selector value 
float Wheel_1_Distance_CM = 0;                                // Wheel 1 travel distance variable
float Wheel_2_Distance_CM = 0;                                // Wheel 2 travel distance variable
float Distance_Wheels = 26.7;                                   // Distance between the two wheels is 20 cm
float radius = Distance_Wheels / 2;
float angle;                                                    // Angle of rotation of the robot. Negative angle corresponds to right turn.
float rads;
float distance;                                               //  Ditance - cm - to be moved forwards or backwards
//float adjuster_angle = 1.077;
//float adjuster_angle = 1.08675;
float adjuster_angle = 1.0;                                // The encoders aren't perfect. Multiply this value to the distance to get more accurate results.
float adjuster_distance = 1.076;
bool backwards = false;                                       // If backwards = true, wheels move in opposite direction.
float encoder1_variable;
float encoder2_variable;
float encoder1_prev;
float encoder2_prev;

// POSE VARIABLES
float x_pos;                                              // X position - cm - of the robot in the playing table
float y_pos;                                              // Y position - cm - of the robot in the playing table
float orientation;                                         // Initial orientation is 90 degrees. 
float wheel_1_x_pos;                                    // Store value moved by wheel 1
float wheel_1_y_pos;                                    // Store value moved by wheel 1
float wheel_2_x_pos;                                    // Store value moved by wheel 2
float wheel_2_y_pos;   


//VOLTAGE READER
CheapStepper stepper (5,6,7,8);
bool moveClockwise = true;
unsigned long moveStartTime = 0; // this will save the time (millis()) when we started each new move
int move_degrees;
int stepsLeft;
float ref_resistance = 1000;                  // Value of the reference resistance in ohms
float Rx;

// COLOR
bool yellow = true;                                           // true if we are in the yellow side, false if we are in the purple side

// MAPPING VARIABLES
float x_pos_yr1[] = {7.5  + 14, 40.0, 38.586, 90, 25, 25, 140.0, 70.0, 30, 7.5  + 14};                    // Yellow Robot 1, robot cabinet, x coordinates
// float x_pos_yr1[] = {7.5  + 14, 30.0, 60, 140.0, 120.0, 50.0, 37.0, 50.0, 19.5, 19.5, 19.5};                    // Yellow Robot 1, robot cabinet, x coordinates
//float y_pos_yr1[] = {-85, -85, -105.0, -105.0, -120.0, -150.0, -163.0, -70.0, -30.0, -18.0, -60.0};      // Yellow Robot 1, robot cabinet, y coordinates
float y_pos_yr1[] = {-85.5, -160.0, -161.414, -75, -30.0, -27.0, -130.0, -130.0, -170, -90};      // Yellow Robot 1, robot cabinet, y coordinates
//int actions[] = {1, 0, 0, 0, 0, 3, 0, 2, 0, 0, 4}; // Actions to be performed before every position Robot 1.
int actions[] = {0, 0, 0, 2, 6, 0 , 4, 0, 0, 5};// Actions to be performed before every position Robot 1.


//float x_pos_yr1[] = {6.5, 30, 130, 130, 120, 30, 65, 65, 66.75, 66.75 + 18.5, 66.75 + 2*18.5, 66.75 + 3*18.5, 66.75 + 4*18.5, 66.75 + 5*18.5, 66.75 + 5*18.5, 97.5}; // Yellow Robot 2, robot voltage, x coordinates
//float x_pos_yr1[] = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
//float y_pos_yr1[] = {-51, -51, -30.0, -15.0, -70, -75.0, -150, -185.0, -185.0, -185.0, -185.0, -185.0, -185.0, -185.0, -185.0, -137.5};                          // Yellow Robot 2, robot voltage, y coordinates
//float y_pos_yr1[] = {-125, -125, -125, -125, -125, -125, -125, -125, -125, -125};
//int actions[] = {0, 0, 0, 0, 0, 4, 0, 5, 6, 6, 6, 6, 6, 6, 0};       
//int actions[] = {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float x_next;
float y_next;
float x3;
float y3;
float x4;
float y4;
float dot;
float det;


//GRIPPER
Servo servo_test1;        //initialize a servo object for the connected servo     
Servo servo_test2;        // Servo 2 LDX 218 

int pos = 0;
int pos2 = 70;


// TIMING
unsigned long myTime;

// SWITCH
const int DIN_PIN = A0;


///////////
///SETUP///
///////////

void setup(){
  Wire.begin();                                               // Begin I2C bus
  Serial.begin(9600);                                         // Begin serial

  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // Read switch state
  pinMode(DIN_PIN, INPUT ); // Set pin to read state of switch
  int value;
  value = digitalRead( DIN_PIN );
  //value = 0;

  //Mapping
  if (value == 1){
    yellow = true;

    // Pose variables
    x_pos = x_pos_yr1[0];                                              // X position - cm - of the robot in the playing table
    y_pos = y_pos_yr1[0];                                              // Y position - cm - of the robot in the playing table
    orientation = 0;                                         // Initial orientation is 90 degrees. 

    Serial.println("yellow-----");
  }
  else {
    yellow = false;

    for (int i = 0; i < len(x_pos_yr1); i++){
      x_pos_yr1[i] = 300 - x_pos_yr1[i];
    }

    //x_pos_yr1[4] = 300 - 11;
    //x_pos_yr1[5] = 300 - 11;
    x_pos = x_pos_yr1[0];                                              // X position - cm - of the robot in the playing table
    y_pos = y_pos_yr1[0];                                              // Y position - cm - of the robot in the playing table
    orientation = 180;                                         // Initial orientation is 90 degrees. 
  }
  

  stepper.setRpm(60);

  // Gripper Setup
  servo_test1.write(pos);
  servo_test2.write(pos2);
  servo_test1.attach(6);      // attach rear gripper to signal 6
  servo_test2.attach(7);     // attach front gripper to signal 7  
  

  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation MODE
  Wire.write(MODE_SELECTOR);
  Wire.write(Mode);                                           
  Wire.endTransmission();
  

  encodeReset();                                              // Cals a function that resets the encoder values to 0 

 
} // setup()

//////////
///LOOP///
//////////

void loop(){

  Mapp();
  delay(100000000);

}



///////////////
///FUNCTIONS///
///////////////


void action1(){
  // Move forwards and backwards to make sure we are in the correct position
  distance = 16;
  backwards = true;
  Linear();
  backwards = false;

  if (yellow == false){
    x_pos = x_pos_yr1[0] + 14;
    y_pos = y_pos_yr1[0];
  } else if (yellow == true){
    x_pos = x_pos_yr1[0] - 14;
    y_pos = y_pos_yr1[0];

  }

  
  // Grab the replica with the gripper
  //myservo1.write(80);
}

void action2(){

  // Move forward and collide with the workshed 
//
//  distance = 8;
//  backwards = true;
//  Linear();
//  backwards = false;
  
  angle = 180;
  Rotation();

  distance = 7;
  backwards = true;
  Linear();
  backwards = false;
//
//  if (yellow == false){
//    x_pos = 300 - (25 + 5.3);
//    y_pos = -175 + 5.3;
//    orientation = 135 - 0;
//  }else{
//    x_pos = 25 + 5.3;
//    y_pos = -175 + 5.3;
//    orientation = 45 + 0;
//  }

  grab_rear_statuette();

  backwards = false;
  distance = 12;
  Linear();

  angle = 180;
  Rotation();

  distance = 12;
  Linear();

//  if (yellow == false){
//    x_pos = 300 - (25 + 5.3 + 1);
//    y_pos = -175 + (5.3 + 1);
//    orientation = -45 - 0;
//  }else{
//    x_pos = 25 + ( 5.3 + 1 );
//    y_pos = -175 + ( 5.3 + 1 );
//    orientation = -135 + 0;
//  }

  drop_front_statuette();

  distance = 20;
  backwards = true;
  Linear();
  backwards = false;

}

void action3(){
 
}

void action4(){
  // Open gripper (leave statuette in display cabinet)
  // Code here:
  //myservo1.write(0);


  if (yellow == false){
    angle = 180;
    Rotation();
  } else{
    angle = -180;
    Rotation();
  }

  distance = 24;    
  backwards = true;
  Linear();
  backwards = false;

  drop_rear_statuette();

  if (yellow == false){
    x_pos = 300 - 25;
    y_pos = -7.5;
    orientation = -90;
  }else{
    x_pos = 25;
    y_pos = -7.5;
    orientation = - 90;
  }
  
  distance = 10;   
  Linear();


}

void action5(){

  // Move forward and collide with the workshed 
  if (yellow == false){
    x_pos = 300 - (25 + 5.3 + 1);
    y_pos = -175 + (5.3 + 1);
    orientation = -45;
  }else{
    x_pos = 25 + (5.3 + 1);
    y_pos = -175 + (5.3 + 1);
    orientation = -135;
  }

  distance = 12;
  backwards = true;
  Linear();
  backwards = false;
}

void action7(){
  // Move the voltage sensing arm
  sequence_movement_voltage();
}

void action6(){
  // Read voltage
  //sequence_movement();
  delay(3500);
}




void Mapp(){

  for (int i = 0; i < len(x_pos_yr1); i++){

    Serial.print("Orientation: ");
    Serial.println(orientation);
    Serial.print("x_pos: ");
    Serial.println(x_pos);
    Serial.print("y_pos: ");
    Serial.println(y_pos);
  
    x_next = x_pos_yr1[i];
    y_next = y_pos_yr1[i];

    
    if (actions[i] == 1){                                          // Find action that correpsonds to number
      action1();                 
    }else if (actions[i] == 2){
      action2();
    }else if (actions[i] == 3){
      action3();
    }else if (actions[i] == 4){
      action4();
    }else if (actions[i] == 5){
      action5();
    }else if (actions[i] == 6){
      action6();
    }

    distance_angle(x_pos, y_pos, x_next, y_next);

    Rotation();
    Linear();                                                    // Calls a function that moves the platform forward

    myTime = millis();

    delay(50);
    
    if (myTime > 90000){

      if (actions[i + 1] == 5){
        action5();
      }

      Serial.println("90 seconds played. Go back home.");
      x_next = x_pos_yr1[len(x_pos_yr1) - 1];
      y_next = y_pos_yr1[len(y_pos_yr1) - 1];

      distance_angle(x_pos, y_pos, x_next, y_next);

      Rotation();
      Linear();        // Calls a function that moves the platform forward

      delay(100000);
      
    }

  }  
  
}

void distance_angle(float x1_in, float y1_in, float x2_in, float y2_in){          
  // Given two data points, get the distcance and angle required to go from one to the other
  x3 = x2_in - x1_in;
  y3 = y2_in - y1_in;

  x4 = cos(orientation * PI / 180);
  y4 = sin(orientation * PI / 180);

  dot = x4 * x3 + y4 * y3;
  det = x4 * y3 - y4 * x3;
  angle = atan2(det, dot) * 180 / PI;
  distance = sqrt(sq(x2_in - x1_in) + sq(y2_in - y1_in));

  Serial.print("Angle: ");
  Serial.println(angle);
  Serial.print("Distance: ");
  Serial.println(distance);
  
}


void sequence_movement_voltage(){
  moveClockwise = true;
  move_degrees = 330;
  move_arm();

  // Code measure resistance
  delay(500);

  float Rx_comb = 0;

  for (int i = 0; i < 6; i++){
    // Read resistance
    Rx = resistance_reader();
    
    if (Rx > Rx_comb){
      // if resistance value is larger than the previous recorded value set Rx_comb to the largest
      Rx_comb = Rx;

    }
    delay(5);

  }

  if (Rx >= 100 and Rx <= 735 and yellow == false){
    // Turn purple excavation squares
    moveClockwise = true;
    move_degrees = 100;
    move_arm();

    moveClockwise = false;
    move_degrees = 430;
    move_arm();
  }

  else if (Rx > 735 and Rx <= 2850 and yellow == true){
    // Turn yellow excavation squares
    moveClockwise = true;
    move_degrees = 100;
    move_arm();

    moveClockwise = false;
    move_degrees = 430;
    move_arm();
  }
  else{
    // Not turn other squares
    moveClockwise = false;
    move_degrees = 330;
    move_arm();
  }

  
}

float resistance_reader(){
    // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1024.0);

  float I = voltage / ref_resistance;
  float VRx = 5 - voltage;
  Rx = VRx / I;
  Rx = (5 - voltage) / I;

  return Rx;

}

void move_arm(){
  
  stepper.newMoveDegrees(moveClockwise, move_degrees);
  
  bool __continue_arm__ = true;
  
  while (__continue_arm__){
    
    stepper.run();
    int stepsLeft = stepper.getStepsLeft();
    
    if (stepsLeft == 0){
      __continue_arm__ = false;
    }
  }
}


void Movement(){
  
  bool __continue__ = true;                               // If true continue moving wheel

  encoder1_prev = 0;
  encoder2_prev = 0;

  while (__continue__){

    // Anything that has to be done constantly, like avoidance or communication has to go here

    encoder1_variable = encoder1();
    encoder2_variable = encoder2();

    Serial.print("encoder1_variable: ");
    Serial.println(encoder1_variable);
    Serial.print("encoder2_variable: ");
    Serial.println(encoder2_variable);

    //time_prop_linear(encoder1_variable - encoder1_prev, encoder2_variable - encoder2_prev);

    encoder1_prev = encoder1_variable;
    encoder2_prev = encoder2_variable; 
 
    if (abs(encoder1_variable) <= abs(Wheel_1_Distance_CM) && abs(encoder2_variable) <= abs(Wheel_2_Distance_CM)){     // If statement to check the status of the traveled distance

      // Set velocity
      sense_and_avoid();                                       // Set correct speed of motor. Deccelarating? Obstacle? Backwards?
    
      Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();
  
      Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
      Wire.write(SPEED1);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();

      //encoder1_prev = encoder1_variable;
      
    }
    else {
      __continue__ = false;                                   // Stop while loop
    }
  }
}

void time_prop_linear(float dvec1, float dvec2){
  wheel_1_x_pos = x_pos + radius * cos((orientation + 90) * PI / 180.0);
  wheel_1_y_pos = y_pos + radius * sin((orientation + 90) * PI / 180.0);

  wheel_2_x_pos = x_pos + radius * cos((orientation - 90) * PI / 180.0);
  wheel_2_y_pos = y_pos + radius * sin((orientation - 90) * PI / 180.0);

  wheel_1_x_pos += dvec1 * cos(orientation * PI / 180.0);
  wheel_1_y_pos += dvec1 * sin(orientation * PI / 180.0);

  wheel_2_x_pos += dvec2 * cos(orientation * PI / 180.0);
  wheel_2_y_pos += dvec2 * sin(orientation * PI / 180.0);

  x_pos = wheel_2_x_pos + ((wheel_1_x_pos - wheel_2_x_pos) / 2);
  y_pos = wheel_2_y_pos + ((wheel_1_y_pos - wheel_2_y_pos) / 2);
  
  orientation = (angle_two_vectors((wheel_1_x_pos - wheel_2_x_pos), (wheel_1_y_pos - wheel_2_y_pos), 2.0, 0) - 90);
}

float angle_two_vectors(float x_1, float y_1, float x_2, float y_2){
  dot = x_2 * x_1 + y_2 * y_1;
  det = x_2 * y_1 - y_2 * x_1;
  angle = atan2(det, dot) * 180 / PI;
  return angle;
}

void reset_position(float distance_in){                                            // Cals a function that resets the encoder values to 0
  if (backwards){
    x_pos -= distance_in * cos(orientation * PI / 180);               // After movement reset x position according to orientation
    y_pos -= distance_in * sin(orientation * PI / 180);               // After movement reset y position according to orientation
  }
  else{
    x_pos += distance_in * cos(orientation * PI / 180);               // After movement reset x position according to orientation
    y_pos += distance_in * sin(orientation * PI / 180);               // After movement reset y position according to orientation
  }

  stopMotor();                                                // Cals a function that stops the platform 
  encodeReset();
}

void Linear(){
  //DualSpeedValue = 129;                                       // Sets a combined motor speed value for upcoming forward motion function (150mm/s). Below 128, backward
  Wheel_1_Distance_CM = distance * adjuster_distance;         // Sets wheel 1 travel distance value - cm, for upcoming motion function
  Wheel_2_Distance_CM = distance * adjuster_distance;                             // Sets wheel 2 travel distance value - cm, for upcoming motion function
  Serial.print("Wheel_1_Distance_CM: ");
  Serial.println(Wheel_1_Distance_CM);
  Serial.print("Wheel_2_Distance_CM: ");
  Serial.println(Wheel_2_Distance_CM);
  Movement();
  //reset_position(distance);

  correction_linear();
  reset_position(distance);
  stopMotor();                                                      // Cals a function that stops the platform 
  encodeReset();
}

void correction_linear(){
  if (backwards == true){
    DualSpeedValue = 128 + 3;
  } else{
    DualSpeedValue = 128 - 3;
  }

  Serial.println("correction_linear");

  Wheel_1_Distance_CM = abs(encoder1_variable) - abs(Wheel_1_Distance_CM);
  Wheel_2_Distance_CM = abs(encoder2_variable) - abs(Wheel_2_Distance_CM);

  stopMotor();   
  encodeReset();

  delay(10);

  bool __continue__ = true;  
  
  encoder1_prev = 0;
  encoder2_prev = 0;
  
  while (__continue__){

    // Anything that has to be done constantly, like avoidance or communication has to go here

    encoder1_variable = encoder1();
    encoder2_variable = encoder2();

   

    //time_prop_linear(encoder1_variable - encoder1_prev, encoder2_variable - encoder2_prev);

    encoder1_prev = encoder1_variable;
    encoder2_prev = encoder2_variable;

    if (abs(encoder1_variable) <= abs(Wheel_1_Distance_CM) && abs(encoder2_variable) <= abs(Wheel_2_Distance_CM)){     // If statement to check the status of the traveled distance
    
      Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();
  
      Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
      Wire.write(SPEED1);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();
    }
    else {
      __continue__ = false;                                   // Stop while loop
    }
  }
}

void Movement_Turn(){
  
  bool __continue__ = true;                               // If true continue moving wheel

  encoder1_prev = 0;
  encoder2_prev = 0;
  
  while (__continue__){

    // Anything that has to be done constantly, like avoidance or communication has to go here

    encoder1_variable = encoder1();
    encoder2_variable = encoder2();

    //time_prop_rot(encoder1_variable - encoder1_prev, encoder2_variable - encoder2_prev);

    encoder1_prev = encoder1_variable;
    encoder2_prev = encoder2_variable;

 
    if (abs(encoder1_variable) <= abs(Wheel_1_Distance_CM) && abs(encoder2_variable) <= abs(Wheel_2_Distance_CM)){     // If statement to check the status of the traveled distance

      // Set velocity
      sense_and_avoid();                                      // Set correct speed of motor. Deccelarating? Obstacle? Backwards?
      
      Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();
  
      Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
      Wire.write(SPEED2);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();

    }
    else {
      __continue__ = false;                                   // Stop while loop
    }
  }
}

void time_prop_rot(float dvec1, float dvec2){

  wheel_1_x_pos = x_pos + radius * cos(((orientation + 90) * PI / 180.0) - (dvec1 / radius));
  wheel_1_y_pos = y_pos + radius * sin(((orientation + 90) * PI / 180.0) - (dvec1 / radius));

  wheel_2_x_pos = x_pos + radius * cos(((orientation - 90) * PI / 180.0) - (dvec1 / radius));
  wheel_2_y_pos = y_pos + radius * sin(((orientation - 90) * PI / 180.0) - (dvec1 / radius));

  x_pos = wheel_2_x_pos + ((wheel_1_x_pos - wheel_2_x_pos) / 2);
  y_pos = wheel_2_y_pos + ((wheel_1_y_pos - wheel_2_y_pos) / 2);
  
  orientation = angle_two_vectors((wheel_1_x_pos - wheel_2_x_pos), (wheel_1_y_pos - wheel_2_y_pos), 2.0, 0) - 90;

}


void reset_orientation(float angle_in){                                            // Cals a function that resets the encoder values to 0
  orientation += angle_in;
}

void Rotation() {
  //DualSpeedValue = 129;  
  if (angle > 0){
    backwards = true;
  }
                                              // Sets a combined motor speed value for upcoming right turn function
  rads = (angle*adjuster_angle) * PI / 180;
  Wheel_1_Distance_CM = - rads * radius;                                // Sets wheel 2 travel distance value - cm, for upcoming turn motion function
  Wheel_2_Distance_CM = - Wheel_1_Distance_CM;                        // Sets wheel 2 travel distance value - cm, for upcoming turn motion function

  Movement_Turn();

  reset_orientation(angle);                                           // Cals a function that resets the encoder values to 0 
  correction_rotation();

  stopMotor();                                                      // Cals a function that stops the platform 
  encodeReset();
  backwards = false;
    
}

void correction_rotation(){
  
  if (backwards == true){
    DualSpeedValue = 128 + 3;
  } else{
    DualSpeedValue = 128 - 3;
  }

  Wheel_1_Distance_CM = abs(encoder1_variable) - abs(Wheel_1_Distance_CM);
  Wheel_2_Distance_CM = abs(encoder2_variable) - abs(Wheel_2_Distance_CM);

  stopMotor();   
  encodeReset();

  delay(10);

  bool __continue__ = true;  
  
  encoder1_prev = 0;
  encoder2_prev = 0;
  
  while (__continue__){

    // Anything that has to be done constantly, like avoidance or communication has to go here

    encoder1_variable = encoder1();
    encoder2_variable = encoder2();

    //time_prop_rot(encoder1_variable - encoder1_prev, encoder2_variable - encoder2_prev);

    encoder1_prev = encoder1_variable;
    encoder2_prev = encoder2_variable;

 
    if (abs(encoder1_variable) <= abs(Wheel_1_Distance_CM) && abs(encoder2_variable) <= abs(Wheel_2_Distance_CM)){     // If statement to check the status of the traveled distance

      
      Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();
  
      Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
      Wire.write(SPEED2);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();

      
    }
    else {
      __continue__ = false;                                   // Stop while loop
    }
  }
}


void ultrasonic_distance(){
  uS1 = sonar[0].ping_cm(); // Send ping, get ping time in microseconds (uS).
  uS2 = sonar[1].ping_cm(); // Send ping, get ping time in microseconds (uS).
  uS3 = sonar[2].ping_cm(); // Send ping, get ping time in microseconds (uS).
  uS4 = sonar[3].ping_cm(); // Send ping, get ping time in microseconds (uS).
  
  if (uS1 == 0){
    uS1 = 200;
  }

  if (uS2 == 0){
    uS2 = 200;
  }
  
  if (uS3 == 0){
    uS3 = 200;
  }

  if (uS4 == 0){
    uS4 = 200;
  }

  if (pos2 > 25){
    uS3 = 200;
  }

  if (y_pos < -150){
    uS3 = 200;
  }

  if (y_pos > -30 && x_pos < 50){
    uS1 = 200;
    uS2 = 200;
    uS3 = 200;
    uS4 = 200;
  }
  if (y_pos > -30 && x_pos > (300 - 50)){
    uS1 = 200;
    uS2 = 200;
    uS3 = 200;
    uS4 = 200;
  }

}

// judge distance and adjust velocity to prevent collision.
void sense_and_avoid(){
  ultrasonic_distance();

  myTime = millis();

  if (myTime > 99000) {
    stopMotor();                                                      // Cals a function that stops the platform
    encodeReset();
    delay(10000000);
  }

  if(uS1 < 15 || uS2 < 15 || uS3 < 15 || uS4 < 15){             // within 10cm, slow down a lot (50%? 25%?)
    // Speed if obstacle very close
    
    DualSpeedValue = 128;
    Serial.println("STOP");
  }
  
  else if (uS1 <= 20 || uS2 <= 20 || uS3 <= 20 || uS4 <= 20){                    // within 20cm, slow down
    // Speed if obstacle relatively close

    if (backwards){
      DualSpeedValue = 128 - 15;
    }
    else{
      DualSpeedValue = 128 + 15;
    }   
        
    Serial.println("SLOW");
    // if stopped for more than x seconds, reverse back?
  }
  
  else{                                          // above 20cm, carry on at full speed
    if ((abs(Wheel_1_Distance_CM) - abs(encoder1_variable)) <= 24){
      // Speed close to target, no obstacle
      if (backwards){
        DualSpeedValue = 128 - 2 - (abs(Wheel_1_Distance_CM) - abs(encoder1_variable));
      }
      else {
        DualSpeedValue = 128 + 2 + (abs(Wheel_1_Distance_CM) - abs(encoder2_variable));
      }
    } 
    
    else{
      // Speed far from target, no obstacle
      if (backwards){
        DualSpeedValue = 128 - 50;
      }
      else{
        DualSpeedValue = 128 + 50;
      }
    }

    //Serial.println("Normal");
  }
}


void encodeReset(){                                         // This function resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);                                         
  Wire.endTransmission(); 
  delay(50);
}



float encoder1(){                                           // Function to read and display value of encoder 1 as a long
  Wire.beginTransmission(MD25ADDRESS);                      // Send byte to get a reading from encoder 1
  Wire.write(ENCODERONE);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1  +=Wire.read();                                     // Fourth byte for encoder 1, LLalue
  delay(5);                                                 // Wait for everything to make sure everything is sent
  return(poss1*0.091);                                       // Convert encoder value to cm
}

float encoder2(){                                            // Function to read and display velue of encoder 2 as a long
  Wire.beginTransmission(MD25ADDRESS);           
  Wire.write(ENCODERTWO);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to become available
  long poss2 = Wire.read();                                 // First byte for encoder 2, HH
  poss2 <<= 8;
  poss2 += Wire.read();                                     // Second byte for encoder 2, HL             
  poss2 <<= 8;
  poss2 += Wire.read();                                     // Third byte for encoder 2, LH             
  poss2 <<= 8;
  poss2  +=Wire.read();                                     // Fourth byte for encoder 2, LLalue
  delay(5);                                                 // Wait to make sure everything is sent
  return(poss2*0.091);                                       // Convert encoder value to cm
}


void stopMotor(){                                           // Function to stop motors
  
  Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 10 (0.65s)
  Wire.write(ACCELERATION);
  Wire.write(10);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
  Wire.write(SPEED1);
  Wire.write(128);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Stops motors motor 2 when operated in MODE 0 or 1 and Stops both motors while in turning sequence if operated in MODE 2 or 3
  Wire.write(SPEED2);
  Wire.write(128);
  Wire.endTransmission();
  delay(50);

  //Serial.print("Encoder 1 Distance CM - ");                  // Displays last recorded traveled distance
  //Serial.print(encoder1());
  //Serial.print("   ");
  //Serial.print("Encoder 2 Distance CM - ");
  //Serial.print(encoder2());
  //Serial.println(" ");
}  

void grab_rear_statuette(){
  for (pos = 0; pos<=80; pos+=1){
  servo_test1.write(pos);
  delay(20);
  }
}

void drop_rear_statuette(){
  for (pos = 80; pos>=0; pos-=1){
    servo_test1.write(pos);
    delay(20);
  }
}

void grab_front_statuette(){
  for (pos2 = 25; pos2<=70; pos2+=1){
    servo_test2.write(pos2);
    delay(20);
  }
}

void drop_front_statuette(){
  for (pos2 = 70; pos2>=25; pos2-=1){
    servo_test2.write(pos2);
    delay(20);
  }
}
