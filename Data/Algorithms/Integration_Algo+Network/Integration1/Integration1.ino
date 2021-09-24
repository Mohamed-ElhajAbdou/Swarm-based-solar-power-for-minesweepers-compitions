#include <Wire.h>
#include <MatrixMath.h>
#include "Kalman.h" 
#include <Encoder.h>
Encoder knobLeft(2, 3);
Encoder knobRight(18, 19);
#define RESTRICT_PITCH 
#define PI   3.14159265358979323846
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
//Shaft_Encoder_Data
#define CHA1 2 
#define CHA2 18 
#define CHB1 3
#define CHB2 19
#define CW_LED 8
#define CCW_LED 7
float d=0;float s=0;float i=0;float Toleto;long positionLeft  = -999;long positionRight = -999;
volatile int master_count1=0;
volatile int master_count2=0;

 float newLeft, newRight; float hamood1; float hamood2;
 float Xpos=0; float Ypos=0; float Xpos1=0;
 float Ypos1=0; float Xpos2=0; float Ypos2=0; int   theta; float Distance1; float Distance2;
 Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances
  // put your setup code here, to run once:
const uint8_t MPU6050 = 0x68; /* If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69*/const uint8_t HMC5883L = 0x1E; // Address of magnetometer

/* IMU Data */
double accX, accY, accZ;double gyroX, gyroY, gyroZ;double magX, magY, magZ;int16_t tempRaw;
/*Algorithm Data*/
int kalo;float val=180/PI;float Prob[8];float help[8];float clep[8];float Diff[8];float Angles[8];float Atano[8];float Axeo [8];float Ayeo [8];
float X_poos[8];float Y_poos[8];float Ambo[8];float Bmbo[8];float ATata[8];
//Original              
//5//float Mines_Loc[8][2] = { {2,3},{7,6},{8,4},{5,6},{6,8},{2,9},{5,10} ,{10,10} };
float Mines_Loc[8][2] = { {5,3},{2,9},{10,2},{7,8},{10,12},{14,9},{16,5},{18,8}  };
//float Mines_Loc[8][2] = { {5,3},{16,5},{12,2},{18,10},{14,9},{13,12},{2,9},{7,4}  };
//float Mines_Loc[8][2] = { {5,3},{5,6},{2,10},{12,2},{9,9},{4,15},{16,5},{18,10}  };
//float Mines_Loc[8][2] = { {1,3},{1,5},{1,7},{1,10},{1,13},{1,15},{1,17},{1,21}  };2

float Robot_Loc[1][2]={ {0,0} };
double   RADTODEG=180/PI;
//#########################################################################################################################################################
double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer
double gyroXangle, gyroYangle, gyroZangle; /* Angle calculate using the gyro only*/double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter
uint32_t timer;uint8_t i2cData[14]; // Buffer for I2C data

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556

float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };double magGain[3];
void setup() {
  
//H-bridge

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

  delay(100); // Wait for sensors to get ready
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
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
  
  
  
  //####################################################################################
         	Matrix.Print((float*)Mines_Loc,8,2,"Location  of each  Mine");
                Matrix.Print((float*)Robot_Loc,1,2,"Location  of       ROBOT");



 for(int i=0;i<8;i++)
  {
   Prob[i]=0;help[i]=0;clep[i]=0;Diff[i]=0;Angles[i]=0;Atano[i]=0;X_poos[i]=0;Y_poos[i]=0;Ambo[i]=0;Bmbo[i]=0;ATata[i]=0;
}
   Matrix.Print((float*)Prob,8,1,"Function Of X ");int arrSize = sizeof( Mines_Loc ) / sizeof( float );
 delay(5000);
 }




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

//#############################################################################################################################################################################################
if(i>7)
{
  motorOff(0);      motorOff(1);Serial.println("Finsh          1");
}
  //Serial.println("FOOOOOOOOOOOOOOOOOOOOOOOOooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOR");
for (int i=0;i<=7;)  //FOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR
{ 
  if(i>7)
{
  motorOff(0);      motorOff(1);Serial.println("Finsh           2");
}
while ((kalAngleZ!=Atano[i]))
{

  if(i>7)
{
  motorOff(0);      motorOff(1);
  delay(100000);
  kalAngleZ=0;
  Atano[i]=0;
  Serial.println("Finsh            3");
}

  //motorOff(0);      motorOff(1);
  //Serial.println("WwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww");
  Axeo[i]=(((Mines_Loc[i][0] - Xpos))) ;                X_poos[i]=Mines_Loc[i][0] ;      Ayeo[i]=(((Mines_Loc[i][1] - Ypos )) );        Y_poos[i]=Mines_Loc[i][1];
  Atano[i]=abs((atan(Ayeo[i]/Axeo[i])) * val);     //   ATata[i]=(atan(Ayeo[i]/Axeo[i])) * val;
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

if( (kalo<=-90 &&kalo>=-180) &&  (Atano[i]>=0 && Atano[i]<=90)  )
{
      if (kalo < -135 )
  {
        if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
       {       
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
             }
      }
   if(kalo > -135)
       {
          if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
              {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);
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
           motorGo(0, CCW, 160);motorGo(1, CW, 160);
       
                 }
                 }
  if(kalo == -135)
   {
       if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
         {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
             }
                 }
                        }
  
  
  
  
  if (  (kalo<=-90 &&kalo>=-180)  &&  (Atano[i]>=90 && Atano[i]<=180))
  
  {
          if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
            {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 

                   }
                      }
    
  if (  (kalo<=-90 &&kalo>-180) &&  (Atano[i]<-90 && Atano[i]>-180))
  {
        if(kalo>Atano[i])
          {
             if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
                    }
           }
       if(kalo<Atano[i])
        {
            if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
             {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
               motorGo(0, CCW, 160); motorGo(1, CW, 160);
                         Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_First_First");  //Left
                    }     
                         }
                             }
  if (  (kalo<=-90 &&kalo>=-180)  &&  (Atano[i]<0 && Atano[i]>=-90))
         {
            if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
              {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
                motorGo(0, CCW, 160); motorGo(1, CW, 160);
                          Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_First_Second");  //Left

                   }
                     }
                     
//#####################################################____________________SECOND____________________#############################################################################################
if (  (kalo<=0 &&kalo>=-90)  &&  (Atano[i]<=0 && Atano[i]>=-90))
 {
    if(kalo>Atano[i])
      {
                  if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                       {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
                           }
                             }
       
   if(kalo<Atano[i])
     {
        if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
               {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CCW, 160); motorGo(1, CW, 160);
                       Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Second_Second");  //Left
                }
                }
                  }
        
        
  if (  (kalo<=0 &&kalo>=-90)  &&  (Atano[i]<=-90 && Atano[i]>=-180))
   {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
               {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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

             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
                     }
                        }
  if ( (kalo<0 &&kalo>=-90)  &&  (Atano[i]>90 && Atano[i]<180))
       {   
          if(kalo<-45)
            {
        
               if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                   {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
                   motorGo(0, CW, 160);  motorGo(1, CCW, 160);                          }  
                                }
         if(kalo>-45)
               {
              if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                  {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
                   motorGo(0, CCW, 160);  motorGo(1, CW, 160);                         }   
                              }
         if(kalo==-45)
                {   
                     if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
                         {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
                   motorGo(0, CW, 160);  motorGo(1, CCW, 160);                                  }
                                        }
                                               }
                                               //###################
   if ( (kalo<=1 &&kalo>=-90)  &&  (Atano[i]>=1 && Atano[i]<=90))
      { 
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
          {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
       motorGo(0, CCW, 160); motorGo(1, CW, 160);
                 Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Second_Third");  //Left

      }  
    }
        
//#####################################################________________________________________#############################################################################################


//#####################################################____________________THIRD____________________#############################################################################################
      
    if ( (kalo>1 &&kalo<=90)  &&  (Atano[i]>=0 && Atano[i]<=90) )
    {
      if ( Atano[i]> kalo )
      {
        
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
                             long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
      motorGo(0, CCW, 160); motorGo(1, CW, 160);
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
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
     }   
        }
      /*else
      {
        
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
 
         }*/
              }
        
       //################### 
  if ( (kalo>=0 &&kalo<90)  && (Atano[i]<0 && Atano[i]>=-90) )
      {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
     }   
      }
      //testerror
  if ( (kalo>=0 &&kalo<=90) &&  (Atano[i]<=-90 && Atano[i]>=-180) )
     {
       if(kalo>45)
       {
         
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
     }
     }
       if (kalo<45)
       {
         
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
    {  motorGo(0, CCW, 160); motorGo(1, CW, 160);
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
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 

     }    
         }
         
              }
      
  //******************************    
    if ( (kalo>=0 &&kalo<=90) &&  (Atano[i]>=90 && Atano[i]<=180) )
    {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
        
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
    {      motorGo(0, CCW, 160); motorGo(1, CW, 160);
              Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Third_Fourth");
         }

     
   
               hamood();

 } 
      
      }
   kalo=kalAngleZ;
   
//fortttttt#####################################################____________________FOURTH____________________#############################################################################################
 
    if ( (kalo>=90 &&kalo<=180) && (Atano[i]>=90 && Atano[i]<=180) )
    {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);
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
           motorGo(0, CCW, 160); motorGo(1, CW, 160);
           
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
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);
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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); }
        }
      else
      {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
      
     if ( (kalo>=90 &&kalo<=180) && (Atano[i]<-90 &&Atano[i]>-180))
     {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
      motorGo(0, CCW, 160); motorGo(1, CW, 160);
              Serial.print(" Xpos___");Serial.print(Xpos);Serial.print("___Ypos___");Serial.print(Ypos);Serial.print("__KKAAALLLO___");Serial.print(kalo);Serial.print("\t");

           Serial.println("Move_Left_Fourth_First");
  //Left
 
     }  
       }
       
   if ( (kalo>=90 &&kalo<=180) && (Atano[i]<=0 &&Atano[i]>=-90))
     {
       
        if ( kalo>135 )
          {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
      motorGo(0, CCW, 160); motorGo(1, CW, 160);

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
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
}
            }
          else
          {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
     
   }
   }
         }
         
         
   if (  (kalo>=90 &&kalo<=180) && (Atano[i]>=0 && Atano[i]<90))
    {
      
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
             motorGo(0, CW, 160);  motorGo(1, CCW, 160); 
     }
     }
     
     
   if ( (kalo>=90 &&kalo<=180) && (Atano[i]<=-90 &&Atano[i]>=-180))
   {
           if ( ((kalo>(Atano[i]-5) && kalo<(Atano[i]+5)) )||(Atano[i]==kalo))
    {
               do
                        {
   long newLeft, newRight;   motorGo(0, CW, 160); motorGo(1, CW, 160);

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
      motorGo(0, CCW, 160); motorGo(1, CW,160 );
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
void yarab()
{

    long newLeft, newRight;

  newLeft = knobLeft.read();
  newRight = knobRight.read();

  if (newLeft != positionLeft || newRight != positionRight) 
  {

           positionLeft = newLeft;positionRight = newRight;theta = kalAngleZ    ;  
           Distance1=0.085*((positionLeft*0.2)/(333));                         Distance2=0.085*((positionRight*0.2)/(333));  
           Xpos1 = int(Xpos1+( Distance1*(cos((theta*PI)/180))));           Xpos2 = int(Xpos2+( Distance2*(cos((theta*PI)/180))));
           Ypos1 = int(Ypos1+( Distance1*(sin((theta*PI)/180))));           Ypos2 = int(Ypos2+( Distance2*(sin((theta*PI)/180))));
           Xpos=int((Xpos1+Xpos2)/2);                                       Ypos=int((Ypos1+Ypos2)/2);

           
  }
  
      knobLeft.write(0);
      knobRight.write(0);
 
}
//hamood
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
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);digitalWrite(inBpin[i], LOW);
  }analogWrite(pwmpin[motor], 0);
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {if (direct <=4)
    {if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
        else
        digitalWrite(inApin[motor], LOW);
      if ((direct==0)||(direct==2))digitalWrite(inBpin[motor], HIGH);else digitalWrite(inBpin[motor], LOW);analogWrite(pwmpin[motor], pwm);
    }
  }
}
