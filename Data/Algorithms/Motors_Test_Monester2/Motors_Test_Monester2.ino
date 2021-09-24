

#include <Encoder.h>
#define BRAKEVCC 0
#define CW   1
#define CCW  2
    long newLeft, newRight;

long positionLeft  = -999;
long positionRight = -999;
long hamood=0;
long hamood1=0;
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input+
int pwmpin[2] = {5, 6}; // PWM input
 
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);Serial.println("TwoKnobs Encoder Test:");
   // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);pinMode(inBpin[i], OUTPUT);pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);digitalWrite(inBpin[i], LOW);
  }

  //right
  motorGo(0, CW, 90);
  motorGo(1, CCW, 90);
  delay(3000);

  //Left
  motorGo(0, CCW, 90);
  motorGo(1, CW, 90);
  delay(3000);
  
   motorGo(0, CCW, 90);
  motorGo(1, CCW, 90);
  delay(3000);
    motorGo(0, CW, 90);
  motorGo(1, CW, 90);
  delay(3000);
  motorOff(0);      motorOff(1);

  }


void loop() {
}

void motorOff(int motor)
{
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}


void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
