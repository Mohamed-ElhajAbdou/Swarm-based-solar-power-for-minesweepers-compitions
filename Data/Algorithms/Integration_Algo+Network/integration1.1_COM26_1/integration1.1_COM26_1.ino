#include <RF24.h>
#include <RF24_config.h>
#include<SPI.h>
#include <MatrixMath.h>
#include<SoftReset.h>
 #include <stdio.h>

//ce,csn pins
//NRF
RF24 radio(48,49);
  String inString = "";    // string to hold input
  char my_var[32]={0};
  float Matix[32];
  float s=0;
  int k=0;
  int arrSize;
  int Counter_Matrix_element=0;
  int Counter_while_Loop=0;
  float Mines_Loc[32][2];
  int Algorithm_Counter=0;
void setup(void) 
{
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

  for(int k=0;k<32;k++)
  {(Matix[k])=0;
}
Matrix.Print((float*)Matix,8,2,"Mines Locations  Of X ");

}

void loop() {
  delay(1000);

  while(Counter_while_Loop<100)
 {

       //soft_restart();

  radio.startListening();
  Serial.println("Now_Loooading......");
 Serial.print("\t               Counter_while_Loop");Serial.print(Counter_while_Loop);Serial.println();
 if(radio.available())
  {
    radio.read(my_var,sizeof(my_var));
 
       //Serial.println(my_var);
       String Message(my_var);
       //Serial.println(my_var[2]);
for (int i=0;i<32;i++)
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

for (int i=0;i<8;i++)
{
    for(int j=0;j<1;j++)
      {
        Mines_Loc[j][i]=Matix[k];
        k=k+1;
      }
}
k=0;
arrSize = sizeof( Mines_Loc ) / sizeof( float );
Serial.print("Size of the Matrix Mines LOC ");Serial.println(arrSize/2);

Matrix.Print((float*)Mines_Loc,8,2,"Mines Locations  Of X ");

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

if (Counter_while_Loop==15&&Matix[0]==0&&Matix[1]==0)
  {
    Serial.println("Sorry we need to RESET the Arduino ");
    soft_restart();
  }

}


      
    
 
