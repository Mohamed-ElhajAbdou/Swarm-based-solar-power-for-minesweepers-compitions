#include <Encoder.h>
#define BRAKEVCC 0
#define CW   1
#define CCW  2
    long newLeft, newRight;

long positionLeft  = -999;
long positionRight = -999;
int counter =0;
char pulse;
/*float hamood=0;
float hamood1=0;*/
 float hamood1; float hamood2;
 float Xpos=0; float Ypos=0; float Xpos1=0;
 float Ypos1=0; float Xpos2=0; float Ypos2=0; int   theta; float Distance1; float Distance2;
float hamoodtot=0;
int Dir[2] = {10, 7};  // INA: Clockwise input
int pwmpin[2] = {9, 5}; // PWM input
Encoder knobLeft(2, 3);
Encoder knobRight(18, 19);
//   avoid using pins with LEDs attached

void setup() {

  Serial.begin(9600);Serial.println("TwoKnobs Encoder Test:");
    pinMode(47, INPUT);                           // Trigger pin set to output

   // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(Dir[i], OUTPUT);pinMode(pwmpin[i], OUTPUT);
    
  }
    for (int i=0; i<2; i++)
  {
    digitalWrite(Dir[i], 0);digitalWrite(pwmpin[i], 0);
  }


  newLeft = knobLeft.read();
  newRight = knobRight.read();
  delay(1000);


  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
 



void loop() {
  long newLeft, newRight;
    //motorGo(0, CCW, 100); motorGo(1, CCW, 100);
   do
  {
    
        motorGo(0, CW, 70); motorGo(1, CW, 70);//Move_Forword
    /*if ( hamoodtot >=10)
    {
      motorOff(0);      motorOff(1);
      delay(100000);
    }*/
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  
  if (newLeft != positionLeft || newRight != positionRight) {
    //Serial.print("hamoodtot = "); Serial.print("\t"); Serial.print(hamoodtot);
    /*Serial.print("\t");Serial.print("abs = ");  Serial.print(Distance1);
    Serial.print("\t");Serial.print("abs = ");  Serial.print(Distance2);
    Serial.println();*/
    //Xpos=2;
    
    positionLeft = newLeft;
    positionRight = newRight;
  /* Distance1=7*((0.2*(positionLeft))/333);
    Distance2=7*((0.2*(positionRight))/333);
        hamoodtot=(Distance1+Distance2)/2;*/
         Ultra_sonic();
               Distance1= 0.25*((positionLeft*0.2)/(333));  Distance2= 0.25*((positionRight*0.2)/(333));  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((26*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((26*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((26*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((26*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=   float(((Ypos1+Ypos2)/2));
           counter=counter+1;
      
    
  
       Serial.print("\t");Serial.print("Xpos = ");  Serial.print(Xpos);Serial.print("\t");Serial.print("Ypos = ");  Serial.print(Ypos);Serial.print("Counter = ");  Serial.println(counter);
}
  }//while(hamoodtot<10);
  while(Xpos < 10 || Ypos <0.5);
   motorOff(0);      motorOff(1);
      delay(100000);
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
//  motorOff(0);      motorOff(1);
}
/*
void loop() 
{
  motorGo(0, CCW, 50); motorGo(1, CW, 50);
  delay(2000);
  motorOff(0);      motorOff(1);
  motorGo(0, CW, 50); motorGo(1, CW, 50);

delay(10000);
motorGo(0, CW, 50); motorGo(1, CCW, 50);
delay(10000);
motorGo(0, CCW, 50); motorGo(1, CCW, 50);
delay(10000);
  
   if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }

}
*/

void motorOff(int motor)
{
  
    digitalWrite(pwmpin[motor], LOW);
  
}


void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(Dir[motor], 1);
      else
        digitalWrite(Dir[motor], 0);


      analogWrite(pwmpin[motor], pwm);
    }
  }
}

void Ultra_sonic()
{
   pulse=digitalRead(47);
          if ( pulse== HIGH)
          {
             motorOff(0);      motorOff(1);
             delay(2000);
              
                for ( int LED=0 ;LED<4;LED++)
                    {
                        digitalWrite(12, HIGH);
                          delay(500);
                              digitalWrite(12, LOW);
                                delay(500);

                                                }
            
            
          }

}
