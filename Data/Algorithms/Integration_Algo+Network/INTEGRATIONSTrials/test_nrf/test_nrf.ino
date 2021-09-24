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
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
#define CS_THRESHOLD 100
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
RF24 radio(49,48);
  String inString = "";    // string to hold input
  char my_var[32]={0};
  float Matix[32];
  int k=0;
  int arrSize;
  int Counter_Matrix_element=0;
  int Counter_while_Loop=0;
  float Mines_Loc[32][2];
  int Algorithm_Counter=0;
long positionLeft  = -999;
long positionRight = -999;

volatile int master_count1=0;
volatile int master_count2=0;

 float newLeft, newRight; float hamood1; float hamood2;
 float Xpos=0; float Ypos=0; float Xpos1=0;
 float Ypos1=0; float Xpos2=0; float Ypos2=0; int   theta; float Distance1; float Distance2;
/*Algorithm Data*/
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


void setup(void) 
{
    


  while(!Serial);
  Serial.begin(115200);
  radio.begin();
    Wire.begin();

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

  
  
}


void loop() {
recivingdata();
Matrix.Print((float*)Mines_Loc,Algorithm_Counter,2,"Mines Locations  Of X ");
Serial.print("Counter_Matrix_element   ");Serial.print(Algorithm_Counter/2);Serial.println();
//End of Foor Loop
         }




void recivingdata()
{
  
    delay(1000);

  while(Counter_while_Loop<10)
 {
 radio.startListening();
 Serial.println("Now_Loooading......");Serial.print("\t               Counter_while_Loop");Serial.print(Counter_while_Loop);Serial.println();
 if(radio.available())
  {
     Serial.print("Available ......");      


    radio.read(my_var,sizeof(my_var));
 
       String Message(my_var);
       Serial.println(my_var);
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
       }
    Serial.print("Counter_Matrix_element   ");Serial.print(Algorithm_Counter/2);Serial.println();
        Algorithm_Counter=Counter_Matrix_element;
       Counter_Matrix_element=0;
             /* while(
       {
       Serial.print("the Buffer ");Serial.print(my_var);
           
       */
Matrix.Print((float*)Matix,1,32,"Recevied in one array ");

for (int i=0;i<Algorithm_Counter;i++)
{
    for(int j=0;j<1;j++)
      {
        Mines_Loc[j][i]=Matix[k];
        k=k+1;
      }
}
k=0;
arrSize = sizeof( Mines_Loc ) / sizeof( float );
//Serial.print("Size of the Matrix Mines LOC ");Serial.println(arrSize/2);

Matrix.Print((float*)Mines_Loc,Algorithm_Counter,2,"Mines Locations  Of X ");

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

if (Counter_while_Loop==10&&Matix[0]==0&&Matix[1]==0)
  {
    Serial.println("Sorry we need to RESET the Arduino ");
    soft_restart();
  }

  

}
