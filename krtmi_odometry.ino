#include "RotaryEncoder.h"

/* CONSTANTS and VARIABLES */
#define TRACK_WIDTH_CM 20.0
#define CENTER_WHEEL_OFFSET_CM -10.0
#define ODO_WHEEL_DIAMETER_CM 5.8
#define ENCODER_TICKS_PER_REVOLUTION 20.0
#define TICKS_TO_CM (ENCODER_TICKS_PER_REVOLUTION / (ODO_WHEEL_DIAMETER_CM * PI))

// Variables for odometry
float x = 0.0; // x position in centimeters
float y = 0.0; // y position in centimeters
float theta = 0.0; // heading in radians

int CounterB = 0; 
void RotaryBack(); 

int CounterL = 0; 
void RotaryLeft(); 

int CounterR = 0;
void RotaryRight(); 

RotaryEncoder Rotary_Back(&RotaryBack, 2, 3, 1); // Pins 2 (DT), 3 (CLK), 4 (SW)
RotaryEncoder Rotary_Left(&RotaryLeft, 19, 18, 1); 
RotaryEncoder Rotary_Right(&RotaryRight, 20, 21, 1); 

void RotaryBack()
{
  const unsigned int state = Rotary_Back.GetState();
  
  if (state & DIR_CW)  
    CounterB++;
    
  if (state & DIR_CCW)  
    CounterB--;    
}

void RotaryLeft()
{
  const unsigned int state = Rotary_Left.GetState();
  
  if (state & DIR_CW)  
    CounterL++;
    
  if (state & DIR_CCW)  
    CounterL--;    
}

void RotaryRight()
{
  const unsigned int state = Rotary_Right.GetState();
  
  if (state & DIR_CW)  
    CounterR++;
    
  if (state & DIR_CCW)  
    CounterR--;    
}


void updateOdometry() {
  // Calculate the distance traveled by each wheel
  float leftDistance   = CounterL / TICKS_TO_CM;
  float rightDistance  = CounterR / TICKS_TO_CM;
  float centerDistance = CounterB / TICKS_TO_CM;
  
  // Reset encoder values
  CounterL = 0;
  CounterR = 0;
  CounterB = 0;
  
  // Calculate the change in position and heading
  float deltaDistance = (leftDistance + rightDistance + centerDistance) / 3.0;
  float deltaTheta    = (rightDistance - leftDistance) / TRACK_WIDTH_CM;
  
  // Update the position and heading
  theta += deltaTheta;
  x += deltaDistance * cos(theta);
  y += deltaDistance * sin(theta);
}

unsigned long previousMillis = 0;
const long interval = 1; 

void setup()
{
  Rotary_Back.setup();  
  Rotary_Left.setup();  
  Rotary_Right.setup();  
  Serial.begin(9600);  
  Serial.println("Rotary Encoder Tests");  
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis += interval;
    
    updateOdometry();
  
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" cm, Y: ");
    Serial.print(y);
    Serial.print(" cm, Heading: ");
    Serial.print(theta * 180.0 / PI);
    Serial.println(" degrees");  
  }
  
}
