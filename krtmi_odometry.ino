#include "RotaryEncoder.h"
//#include <SimpleKalmanFilter.h>

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

//byte measureUncertainty = 1;
//byte estimateUncertainty = 1;
//float processVariance = 0.01;

/*  Source: https://github.com/denyssene/SimpleKalmanFilter/
 *  e_mea: Measurement Uncertainty - How much do we expect to our measurement vary
 *  e_est: Estimation Uncertainty - Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
 *  q: Process Variance - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01. Should be tunned to your needs.
 */
//SimpleKalmanFilter kalmanX(measureUncertainty, estimateUncertainty, processVariance);  // Process noise, Measurement noise, Initial estimate
//SimpleKalmanFilter kalmanY(measureUncertainty, estimateUncertainty, processVariance);
//SimpleKalmanFilter kalmanTheta(measureUncertainty, estimateUncertainty,processVariance);

int CounterC = 0; 
void RotaryCenter(); 

int CounterL = 0; 
void RotaryLeft(); 

int CounterR = 0;
void RotaryRight(); 

RotaryEncoder Rotary_Center(&RotaryCenter, 2, 3, 1); // Pins 2 (DT), 3 (CLK), 4 (SW)
RotaryEncoder Rotary_Left(&RotaryLeft, 19, 18, 1); 
RotaryEncoder Rotary_Right(&RotaryRight, 20, 21, 1); 

void RotaryCenter()
{
  const unsigned int state = Rotary_Center.GetState();
  
  if (state & DIR_CW)  
    CounterC++;
    
  if (state & DIR_CCW)  
    CounterC--;    
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
  float centerDistance = CounterC / TICKS_TO_CM;
  
  // Reset encoder values
  CounterL = 0;
  CounterR = 0;
  CounterC = 0;
  
  // Calculate the change in position and heading
  float deltaDistance = (leftDistance + rightDistance + centerDistance) / 3.0;
  float deltaTheta    = (rightDistance - leftDistance) / TRACK_WIDTH_CM;
  
  // Update the position and heading
  theta += deltaTheta;
  x += deltaDistance * cos(theta);
  y += deltaDistance * sin(theta);

//  // kalman filter shit
//  float filteredX = kalmanX.updateEstimate(x);
//  float filteredY = kalmanY.updateEstimate(y);
//  float filteredTheta = kalmanTheta.updateEstimate(theta);
//  x = filteredX;
//  y = filteredY;
//  theta = filteredTheta;
}

unsigned long previousMillis = 0;
const long interval = 1; 

void setup()
{
  Rotary_Center.setup();  
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
