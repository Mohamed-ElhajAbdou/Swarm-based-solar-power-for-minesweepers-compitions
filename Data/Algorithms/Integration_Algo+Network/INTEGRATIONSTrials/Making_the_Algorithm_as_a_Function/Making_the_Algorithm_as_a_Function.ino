#include <Wire.h>
#include <MatrixMath.h>
#include "Kalman.h" 
#include <Encoder.h>
Encoder knobLeft(3, 2);
Encoder knobRight(19, 18);
#define RESTRICT_PITCH 
#define PI   3.14159265358979323846
#define BRAKEVCC 0
#define CW   1
#define CCW  2
int Dir[2] = {10, 7};  // INA: Clockwise input
int pwmpin[2] = {9, 5}; // PWM input
#include <RF24.h>
#include <RF24_config.h>
#include<SPI.h>
#include <MatrixMath.h>
#include<SoftReset.h>
 #include <stdio.h>
 //Shaft_Encoder_Data
#define CHA1 2 
#define CHA2 18 
#define CHB1 3
#define CHB2 19


//ce,csn pins
//*****************NRF VARIABLES******************
RF24 radio(48,49);
  String inString = "";    // string to hold input
  char my_var[32]={0};
  float Matix[32];
  int k=0;
  int arrSize;
  int Counter_Matrix_element=0;
  int Counter_while_Loop=0;
float Mines_Loc[32][2];
float Mines_Loc2[32][2];

int Algorithm_Counter=0;
float d=0;float s=0;float i=0;float Toleto;long positionLeft  = -999;long positionRight = -999;
volatile int master_count1=0;
volatile int master_count2=0;

 float newLeft, newRight; float hamood1; float hamood2;
 float Xpos=0; float Ypos=0; float Xpos1=0;
 float Ypos1=0; float Xpos2=0; float Ypos2=0; int   theta; float Distance1; float Distance2;

int kalo;
float val=180/PI;
float Prob[16];
float help[16];
float clep[16];
float Diff[16];
float Angle[16];
float Atano[16];
float Axeo [16];
float Ayeo [16];
float X_poos[16];
float Y_poos[16];
float Ambo[16];
float Bmbo[16];
float ATata[16];
float Robot_Loc[1][2]={ {0,0} };

Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

// put your setup code here, to run once:


const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
const uint8_t HMC5883L = 0x1E; // Address of magnetometer

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
int16_t tempRaw;

 
double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556

float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];
void setup(void) 
{
    
//H-bridge
  for (int i=0; i<2; i++)
  {
    pinMode(Dir[i], OUTPUT);pinMode(pwmpin[i], OUTPUT);
    
  }
    for (int i=0; i<2; i++)
  {
    digitalWrite(Dir[i], 0);digitalWrite(pwmpin[i], 0);
  }
    delay(100); // Wait for sensors to get ready


  while(!Serial);
  Serial.begin(115200);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(0x76);
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  const uint64_t pipe=0xE8E8F0F0E1LL;
  radio.openReadingPipe(1,pipe);
  radio.enableDynamicPayloads();
  radio.powerUp();

  for(int k=0;k<31;k++){Matix[k]=0;}
  for(int i=0;i<16;i++)
  {Prob[i]=0;help[i]=0;clep[i]=0; Diff[i]=0;
   Atano[i]=0; X_poos[i]=0; Y_poos[i]=0;Ambo[i]=0;Bmbo[i]=0;ATata[i]=0;}
  Wire.begin();

  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ??250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ??2g
  while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(MPU6050, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(MPU6050, 0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    
    while (1);
  }

  while (i2cWrite(HMC5883L, 0x02, 0x00, true)); // Configure device for continuous mode
  calibrateMag();

  delay(100); // Wait for sensors to stabilize

  /* Set Kalman and gyro starting angle */
  updateMPU6050();updateHMC5883L();updatePitchRoll();updateYaw();
  kalmanX.setAngle(roll); // First set roll starting angle 
  gyroXangle = roll; compAngleX = roll;

  kalmanY.setAngle(pitch); // Then pitch
  gyroYangle = pitch;compAngleY = pitch;

  kalmanZ.setAngle(yaw); // And finally yaw
  gyroZangle = yaw;compAngleZ = yaw;

  timer = micros(); // Initialize the timer
}
  
  
//Matrix.Print((float*)Matix,8,2,"Mines Locations  Of X ");



void loop() {
                       
/*################################################______ Update all the IMU values______######################################################################################################*/
  updateMPU6050();updateHMC5883L();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();


  /* Roll and pitch estimation */
  updatePitchRoll();double gyroXrate = gyroX / 131.0; double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
  {
    kalmanX.setAngle(roll);
    compAngleX = roll;kalAngleX = roll;gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);compAngleY = pitch;kalAngleY = pitch;gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif


  /* Yaw estimation */
  updateYaw();
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(yaw);compAngleZ = yaw;kalAngleZ = yaw;gyroZangle = yaw;
  } else
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


  /* Estimate angles using gyro only */
  gyroXangle += gyroXrate * dt; gyroYangle += gyroYrate * dt;gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
  //gyroYangle += kalmanY.getRate() * dt;
  //gyroZangle += kalmanZ.getRate() * dt;

  /* Estimate angles using complimentary filter */
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angles when they has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)gyroZangle = kalAngleZ;

recivingdata();




Matrix.Print((float*)Mines_Loc,Algorithm_Counter,2,"Mines Locations  Of X ");
Serial.print("Counter_Matrix_element   ");Serial.print(Algorithm_Counter/2);Serial.println();
//FOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR

for (int i=0;i<=((Algorithm_Counter/2)-1);)  
{ 
  
  Serial.print((Algorithm_Counter/2)-1);Serial.println();
  if(i>((Algorithm_Counter/2)-1))
{
  
  motorOff(0);      motorOff(1);Serial.println("Finsh_________2");
         Matrix.Print((float*)Mines_Loc,Algorithm_Counter,2,"Mines Locations  Of X ");
  Serial.print("Counter_Matttttttttttttttttttttttttttrix_element   ");Serial.print(Algorithm_Counter/2);Serial.println();

         Matrix.Print((float*)Axeo,1,(Algorithm_Counter+1),"Axeo Locations  Of X ");
         Matrix.Print((float*)Ayeo,1,(Algorithm_Counter+1),"Ayeo Locations  Of X ");
         Matrix.Print((float*)ATata,1,(Algorithm_Counter+1),"ATata Locations  Of X ");
}
while ((kalAngleZ!=ATata[i]))
//while(1)
{

  if(i>((Algorithm_Counter/2)-1))
{
  motorOff(0);      motorOff(1);
  delay(100000);
  kalAngleZ=0;
  ATata[i]=0;
  Serial.println("Finsh_________3");
}

  //motorOff(0);      motorOff(1);
  //Serial.println("WwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww");
  Axeo[i]=(((Mines_Loc[i][0] - Xpos))) ;                X_poos[i]=Mines_Loc[i][0] ;      Ayeo[i]=(((Mines_Loc[i][1] - Ypos )) );        Y_poos[i]=Mines_Loc[i][1];
  ATata[i]=abs((atan(Ayeo[i]/Axeo[i])) * val);

Serial.println();
//imposoble for Atata No Coodinates with negative
if (Axeo[i] <0 && Ayeo[i] <0 )
  { 
          ATata[i]=(abs(ATata[i]))-180;

          Serial.print("__case_0___");

       }
  if( Axeo[i] >0 &&Ayeo[i]<0)
    {
      Serial.print("__case_1111___");
      ATata[i]=-1*(abs(ATata[i]));
         }
  if (Axeo[i] <0 && Ayeo[i] >0)
      {
              Serial.print("__case_2__");

      //ATata[i]=ATata[i]*-1;
      ATata[i]=180-(abs(ATata[i]));
        }
  if(Axeo[i]>0&&Ayeo[i]>0)
      {
        //error
              Serial.print("__case_3__");

        ATata[i]=abs(ATata[i]);
        }
  if( Axeo[i]==0 && Ayeo[i]<0)
   {
           Serial.print("__case_4__");

     ATata[i]=-90;
     }
   if (Axeo[i]==0 && Ayeo[i]>0)
   {
           Serial.print("__case_5__");

     ATata[i]=90;
     }
   if(Ayeo[i]==0 && Axeo[i]>0)
   {
           Serial.print("__case_6__");

     ATata[i]=0;
     }
   if(Ayeo[i]==0 && Axeo[i]<0)
   {
           Serial.print("__case_7__");

     ATata[i]=180;
     }
       hamood();
     kalo=kalAngleZ;
     Serial.print("ATata___");Serial.print(ATata[i]   );Serial.print("__X_Next__");Serial.print(X_poos[i]  );Serial.print("__Y_Next__");Serial.print(Y_poos[i]  );

     //Serial.print("\t");Serial.print("KKKKKAAAAALOOOO___");Serial.print(kalo);Serial.print("___\t___");Serial.print(i);Serial.print("__ATata[i]");Serial.print("\t");Serial.print(ATata[i]);Serial.println();
     
//#####################################################____________________FIRST____________________########################################################################################

if((kalo<=-90&&kalo>=-180)&&(Atano[i]>=0&& Atano[i]<90))

{
      if (kalo < -135 )
  {
        if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
       {       
               do
                        {
                          
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();

  if (newLeft != positionLeft || newRight != positionRight) 

        {

           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2=0.085*((positionRight*0.2)/(333));  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=   float(((Ypos1+Ypos2)/2));
           hamood();
  }
           Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");
           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
      else
        { 
              Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");
              Serial.println("Move_Right_First_third"); Serial.println();       //right
              motorGo(0, CCW, 70);  motorGo(1, CW, 70); 
             }
      }
   if(kalo > -135)
       {
          if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
              {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);
  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
   }
                 Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

          Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
         else
            {
          Serial.print("   Xpos   ");Serial.print(Xpos);Serial.print("     Ypos   ");Serial.print(Ypos);Serial.print("     KKAAALLLO   ");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_First_third");  //Left
  motorGo(0, CW, 70); motorGo(1, CCW, 70);
       
                 }
                 }
  if(kalo == -135)
   {
       if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
         {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
       else
            { 
                        Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Enyway_Move_Right_First_third");        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
             }
                 }
                        }
  
  

 if (  (kalo<=-90 &&kalo>=-180)  &&  (Atano[i]>=90&&Atano[i]<=180))
  {
          if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
            {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
         else
             {  
                         Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

               Serial.println("Move_Right_First_Fourth");        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 

                   }
                      }
      


  if (  (kalo<=-90 &&kalo>=-180) &&  (Atano[i]<=-90 && Atano[i]>=-180))
  {
        if(kalo>Atano[i])
          {
             if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
            else
                { 
                  
                            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_First_First");        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
                    }
           }
       if(kalo<Atano[i])
        {
            if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
             {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    //error detection first first or fourth first
           else
             { 
  motorGo(0, CCW, 70); motorGo(1, CW, 70);
                         Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_First_First");  //Left
                    }     
                         }
                             }
                             

  if (  (kalo<=-90 &&kalo>=-180)  &&  (Atano[i]<0 && Atano[i]>-90))
         {
            if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
              {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
          else
              { 
  motorGo(0, CCW, 70); motorGo(1, CW, 70);
                          Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_First_Second");  //Left

                   }
                     }
                     
//#####################################################____________________SECOND____________________#############################################################################################

if (  (kalo<=0 &&kalo>-90)  &&  (Atano[i]<=0 && Atano[i]>=-90))
 {
    if(kalo>Atano[i])
      {
                  if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                       {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

                              newLeft = knobLeft.read();                    newRight = knobRight.read();
                         if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
                else
                      {  
                                  Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Second_Second");        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
                           }
                             }
       
   if(kalo<Atano[i])
     {
        if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
               {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

                              newLeft = knobLeft.read();                    newRight = knobRight.read();
                         if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }

       else
           { 
               motorGo(0, CCW, 70); motorGo(1, CW, 70);

                       Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Second_Second");  //Left
                }
                }
                  }
        
  
                   



  if (  (kalo<=0 &&kalo>-90)  &&  (Atano[i]<-90 && Atano[i]>=-180))
   {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
               {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

                              newLeft = knobLeft.read();                    newRight = knobRight.read();
                         if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();

           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
         else
                 {  
                             Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Second_First");        //right

             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
                     }
                        }




  if ( (kalo<=0 &&kalo>-90)  &&  (Atano[i]>=90 && Atano[i]<=180))
       {   
          if(kalo<-45)
            {
        
               if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                   {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

                              newLeft = knobLeft.read();                    newRight = knobRight.read();
                         if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    //*****************
               else
                   {
                               Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Second_Fourth");
        //right*********************
                   motorGo(0, CW, 70);  motorGo(1, CCW, 70);                          }  
                                }
         if(kalo>-45)
               {
              if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                  {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

                              newLeft = knobLeft.read();                    newRight = knobRight.read();
                         if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    //eeror detected
             else
                    {  
                                Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Second_Fourth");
        //left
                    motorGo(0, CCW, 70); motorGo(1, CW, 70);
                        }   
                              }
         if(kalo==-45)
                {   
                     if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                         {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

                              newLeft = knobLeft.read();                    newRight = knobRight.read();
                         if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
                    else
                          {  
                                      Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("enyWay_Move_Right_Second_Fourth");
        //right
                   motorGo(0, CW, 70);  motorGo(1, CCW, 70);                                  }
                                        }
                                               }
                                               //###################
                                               


   if ( (kalo<=0 &&kalo>-90)  &&  (Atano[i]>0 && Atano[i]<90))
      { 
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
          {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

                              newLeft = knobLeft.read();                    newRight = knobRight.read();
                         if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    { 
  motorGo(0, CCW, 70); motorGo(1, CW, 70);
                 Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Second_Third");  //Left

      }  
    }
        
//#####################################################________________________________________#############################################################################################


//#####################################################____________________THIRD____________________#############################################################################################
 
 


    if ( (kalo>0 &&kalo<=90)  &&  (Atano[i]>=0 && Atano[i]<=90) )
    {
      if ( Atano[i]> kalo )
      {
        
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
                             long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

                              newLeft = knobLeft.read();                    newRight = knobRight.read();
                         if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
              else
    { 
  motorGo(0, CCW, 70); motorGo(1, CW, 70);
                Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Third_Third");  //Left

        
     }
   }
        
      if (Atano[i]< kalo )
      {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
          else
    {
                Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Third_Third");
        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
     }   
        }
     
              }
        
       //################### 


  if ( (kalo>0 &&kalo<=90)  && (Atano[i]<0 && Atano[i]>=-90) )
      {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    {  
                Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Third_Second");
        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
     }   
      }
      //testerror

  if ( (kalo>0 &&kalo<=90) &&  (Atano[i]<-90 && Atano[i]>=-180) )
     {
       if(kalo>45)
       {
         
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    {
                Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Third_First");
        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
     }
     }
       if (kalo<45)
       {
         
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    {    motorGo(0, CCW, 70); motorGo(1, CW, 70);

              Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Third_First");  //Left


           }
               }    
       else
       {
               if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
 if ( kalo==45)  
 { 

             Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Enyway_Move_Right_Third_First");           //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 

     }    
         }
         
              }
      
  //******************************  



    if ( (kalo>0 &&kalo<=90) &&  (Atano[i]>90 && Atano[i]<=180) )
    {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
        
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");Serial.println();
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
      else
    {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {  
      do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    
    else
    {    
  motorGo(0, CCW, 70); motorGo(1, CW, 70);
              Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Third_Fourth");
         }

     
   
               hamood();

 } 
      
      }
   kalo=kalAngleZ;
   
//fortttttt#####################################################____________________FOURTH____________________#############################################################################################
 
 


 
    if ( (kalo>90 &&kalo<=180) && (Atano[i]>=90 && Atano[i]<=180) )
    {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);
  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));       
  
           
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    {
      if(Atano[i]>kalo)
      {  
  motorGo(0, CCW, 70); motorGo(1, CW, 70);
           
                     Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Fourth_Fourth");
  //Left

     }   
        }
        
     if(Atano[i]<kalo)
      {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);
   newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {

           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));       
      
           
           hamood();
           
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    { 
                Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Fourth_Fourth");
         //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); }
        }
      else
      {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));       
     
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    

        }
     
      }
   

if ( (kalo>90 &&kalo<=180) && (Atano[i]<-90 &&Atano[i]>-180))
     {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));       
      
           
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    //error first first 
    {  
  motorGo(0, CCW, 70); motorGo(1, CW, 70);
              Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Fourth_First");
  //Left
 
     }  
       }


   if ( (kalo>90 &&kalo<=180) && (Atano[i]<=0 &&Atano[i]>=-90))
     {
       
        if ( kalo>135 )
          {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));       
     
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    { 
  motorGo(0, CCW, 70); motorGo(1, CW, 70);

          Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Fourth_Second");
  //Left
}
           } 
          if ( kalo<135)
          {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));       
         
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    { 
                Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Fourth_Second");
        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
}
            }
          else
          {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                               Ypos=float(((Ypos1+Ypos2)/2));       
       
           
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
          if ( kalo==135)
    { 
          Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Enyway_Move_Right_Fourth_Second");
        //right
             motorGo(0, CW, 70);  motorGo(1, CCW, 70); 
     
   }
   }
         }
         


if (  (kalo>90 &&kalo<=180) && (Atano[i]>0 && Atano[i]<90))
    {
      
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));       
       
           
           hamood();
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    { 
                Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Right_Fourth_Third");
        //right
             motorGo(0, CW, 70);  motorGo(1, CCW,70 ); 
     }
     }
     
     
   if ( (kalo>=90 &&kalo<=180) && (Atano[i]<=-90 &&Atano[i]>=-180))
   {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 70); motorGo(1, CW, 70);

  newLeft = knobLeft.read();                    newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) 
        {
           positionLeft = newLeft;              positionRight = newRight;
           theta =( kalAngleZ )   ;  
           Distance1=0.085*((positionLeft*0.2)/(333));  Distance2= 0.7*((positionRight*0.2)/(333));;  
           //le
           Xpos1 = float(Xpos1+( Distance1*(cos((ATata[i]*PI)/180))));           Xpos2 = float(Xpos2+( Distance2*(cos((ATata[i]*PI)/180))));
           Ypos1 = float(Ypos1+( Distance1*(sin((ATata[i]*PI)/180))));           Ypos2 = float(Ypos2+( Distance2*(sin((ATata[i]*PI)/180))));
           Xpos  = float(((Xpos1+Xpos2)/2));                                     Ypos=float(((Ypos1+Ypos2)/2));    
       
           d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;

           hamood();
           
  }
            Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("You Arrived_____Move_Forword");
           delay(60);        d=abs(Xpos-X_poos[i]);      s=abs(Ypos-Y_poos[i]);      Toleto=0.5;      }while ((d>Toleto)||(s>Toleto));
           knobLeft.write(0);      knobRight.write(0);      i=i+1;motorOff(0);      motorOff(1);delay(3000);
       
    }
    else
    {  
  motorGo(0, CCW, 70); motorGo(1, CW, 70);
              Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

             Serial.println("Move_Left_Fourth_First");
  //Left

     }
     }

  


//###########################################################________________________________________#############################################################################################
     kalo=kalAngleZ;

//#####################################################________________________________________#############################################################################################
//End of While Loop
      }
      

//End of Foor Loop
         }
}
void recivingdata()
{
  int Q=5;
  
    delay(1000);

  while(Counter_while_Loop<Q)
 {
 radio.startListening();
 Serial.println("Now_Loooading......");Serial.print("\t               Counter_while_Loop");Serial.print(Counter_while_Loop);Serial.println();
 if(radio.available())
  {
    radio.read(my_var,sizeof(my_var));
 
       String Message(my_var);
for (int i=0;i<31;i++)
    {
      if (my_var[i]=='A')
       {
         my_var[i]= 0;
         Matix[Counter_Matrix_element]=my_var[i];
         
         Serial.println("yeees_______A____");
         Serial.print(i);
         Serial.println();
        Serial.println();
        Counter_Matrix_element=Counter_Matrix_element+1;
       }
      if (my_var[i]=='B')
       {
         my_var[i]= 1;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______B");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
    }
      if (my_var[i]=='C')
       {
         my_var[i]= 2;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______C");
         Serial.print(i);
         Serial.println();  
          Counter_Matrix_element=Counter_Matrix_element+1;
     }
      if (my_var[i]=='D')
       {
         my_var[i]= 3;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______D");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
    }
      if (my_var[i]=='E')
       {
         my_var[i]= 4;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______E");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
    }
      if (my_var[i]=='F')
       {
         my_var[i]= 5;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______F");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
    }
      if (my_var[i]=='G')
       {
         my_var[i]= 6;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______G");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
    } 
       if (my_var[i]=='H')
       {
         my_var[i]= 7;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______H");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
    }
       if (my_var[i]=='I')
       {
         my_var[i]= 8;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______I");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
    }
       if (my_var[i]=='J')
       {
         my_var[i]= 9;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______J");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
    }  
          if (my_var[i]=='K')
       {
         my_var[i]= 10;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______K");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
           
 
    }
             if (my_var[i]=='L')
       {
         my_var[i]= 11;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______L");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
           
 
    } 
    
              if (my_var[i]=='M')
       {
         my_var[i]= 12;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______M");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
           
 
    }
              if (my_var[i]=='N')
       {
         my_var[i]= 13;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______N");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
           
 
    }
              if (my_var[i]=='O')
       {
         my_var[i]= 14;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______O");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
           
 
    }
              if (my_var[i]=='P')
       {
         my_var[i]= 15;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______P");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
           
 
    }
              if (my_var[i]=='Q')
       {
         my_var[i]= 16;
         Matix[Counter_Matrix_element]=my_var[i];
         Serial.println("yeees_______Q");
         Serial.print(i);
         Serial.println();   
           Counter_Matrix_element=Counter_Matrix_element+1;
           
 
    }
       }
    Serial.print("Counter_Matrix_element   ");Serial.print(Algorithm_Counter/2);Serial.println();
    
        Algorithm_Counter=Counter_Matrix_element;
       Counter_Matrix_element=0;
             /* while(
       {
       Serial.print("the Buffer ");Serial.print(my_var);
           
       */
Matrix.Print((float*)Matix,1,32,"Recevied in one array ");
int counter;
/*#ifdef RESTRICT_PITCH
for(int i =0;i<32;i++)
{
  if((Matix[i]!=0&&Matix[i+1]!=0))
  {
     counter =counter+1;
}
}  Serial.print("couuuuuuuuuuuuuuuuuuuuuunt  ");Serial.println(counter);
#endif
*/
for (int i=0;i<31;i++)
{
    for(int j=0;j<1;j++)
      {
        Mines_Loc[j][i]=Matix[k];
        k=k+1;
      }
}
k=0;
//just add this to debugfloat Mines_Loc[8][2] = { {5,3},{2,9},{10,2},{7,8},{10,12},{14,9},{16,5},{18,8}  };
arrSize = sizeof( Mines_Loc ) / sizeof( float );
//Serial.print("Size of the Matrix Mines LOC ");Serial.println(arrSize/2);

Matrix.Print((float*)Mines_Loc,(Algorithm_Counter*2),2,"Mines Locations  Of X ");

    radio.stopListening();


  }
 /* else
  //Cheak if the Radio is not avaliable we need to reset the Arduino 
  {
    Serial.println("No input");
    //soft_restart();

    
  }*/
  /*if (Counter_while_Loop==10&&my_var)
  //***NOTE THAT****[if(my_var)] is Equal in [if(my_var==NULL)] cheacking if the buffer is empty 
  //Cheacking the the while loop is outof range and and the buffer is empty we need to reset the Arduino 
  {
    Serial.println("no inpuuuuut");
    soft_restart();
  }*/
  
  //end of if_Radio_avaliable 
delay(1000);
Counter_while_Loop=Counter_while_Loop+1;
radio.stopListening();
}

if (Counter_while_Loop==Q&&Matix[0]==0&&Matix[1]==0)
  {
    Serial.println("Sorry we need to RESET the Arduino ");
    soft_restart();
  }

  
}

void hamood()
{
  
  
  /* Update all the IMU values */
  updateMPU6050();updateHMC5883L();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();


  /* Roll and pitch estimation */
  updatePitchRoll();
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif


  /* Yaw estimation */
  updateYaw();
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(yaw);
    compAngleZ = yaw;
    kalAngleZ = yaw;
    gyroZangle = yaw;
  } else
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


  /* Estimate angles using gyro only */
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;

  /* Estimate angles using complimentary filter */
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angles when they has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;
  }

void updateMPU6050() {
  while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
  accX = ((i2cData[0] << 8) | i2cData[1]);accY = -((i2cData[2] << 8) | i2cData[3]);accZ = ((i2cData[4] << 8) | i2cData[5]);tempRaw = (i2cData[6] << 8) | i2cData[7];gyroX = -(i2cData[8] << 8) | i2cData[9];gyroY = (i2cData[10] << 8) | i2cData[11];gyroZ = -(i2cData[12] << 8) | i2cData[13];
}

void updateHMC5883L() {
  while (i2cRead(HMC5883L, 0x03, i2cData, 6)); magX = ((i2cData[0] << 8) | i2cData[1]);magZ = ((i2cData[2] << 8) | i2cData[3]);magY = ((i2cData[4] << 8) | i2cData[5]);
}
void updatePitchRoll() {

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll = atan2(accY, accZ) * RAD_TO_DEG;pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}
void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
  magX *= -1; /*Invert axis - this it done here, as it should be done after the calibration*/magZ *= -1;magX *= magGain[0];magY *= magGain[1];magZ *= magGain[2];
  magX -= magOffset[0];magY -= magOffset[1];magZ -= magOffset[2];double rollAngle = kalAngleX * DEG_TO_RAD;double pitchAngle = kalAngleY * DEG_TO_RAD;
  double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;yaw *= -1;
}
void calibrateMag() { // Inspired by: https://code.google.com/p/open-headtracker/
  i2cWrite(HMC5883L, 0x00, 0x11, true);
  delay(100); /* Wait for sensor to get ready*/updateHMC5883L(); /* Read positive bias values*/ int16_t magPosOff[3] = { magX, magY, magZ };i2cWrite(HMC5883L, 0x00, 0x12, true);
  delay(100); /* Wait for sensor to get ready*/updateHMC5883L(); /* Read negative bias values*/int16_t magNegOff[3] = { magX, magY, magZ };i2cWrite(HMC5883L, 0x00, 0x10, true); // Back to normal
  magGain[0] = -2500 / float(magNegOff[0] - magPosOff[0]);magGain[1] = -2500 / float(magNegOff[1] - magPosOff[1]);magGain[2] = -2500 / float(magNegOff[2] - magPosOff[2]);

}      


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
