
import processing.serial.*;

int linefeed = 10;       // linefeed in ASCII
int carriageReturn = 13; // carriage return in ASCII
Serial myPort;           // The serial port
int sensorValue = 0;     // the value from the sensor
float latitude = 0.0;    // the latitude reading in degrees
String northSouth;       // north or south?
float longitude = 0.0;   // the longitude reading in degrees
String eastWest;         // east or west?
char fix;
float degWhole1;
float degDec;
float deglong;
float degWhole2;
float deglat;
int earthRadius=6367000;
float inlat=4732174.5;
float inlong=2688106.0;
float finlat;
float finlong;
int factor = 800;
void setup() {
  
      size(800,800);


}

void draw() {
  back();
  
  move();

  
}

   
  
  
  
  
  
 
  
 

 
    
   


  
void move()
{ 
      stroke(100);
fill(145);
        rect(0,0,10,10);

    float x =latitude;
    float y =  longitude;
    //finlat=x-inlat;
   // finlong=y-inlong;
    println(x+","+y );


   // println("the latitude is :"+finlat+"thelongtiude is:"+finlong);
    println("_________________\n");
    fill(250,5,5);

          stroke(200);
    fill(0,5,255);

   /* ellipse(200,120,20,20);
    ellipse(80,360,20,20);
    ellipse(400,120,20,20);
    ellipse(280,320,20,20);
    ellipse(400,480,20,20);
    ellipse(560,360,20,20);
    ellipse(640,200,20,20);
    ellipse(720,320,20,20);*/
    //scaling factor is (600 / Location)___... 600 is the scale of the window 
    
    ellipse(factor/5,factor/3,20,20);
    ellipse((factor/2),(factor/9),20,20);
    ellipse((factor/10),(factor/2),20,20);
    ellipse((factor/7),(factor/8),20,20);
    ellipse((factor/10),(factor/12),20,20);
    ellipse((factor/14),(factor/9),20,20);
    ellipse((factor/16),(factor/5),20,20);
    ellipse((factor/16),(factor/10),20,20);

        fill(250,5,5);

    ellipse(x*40,y*40,20,20);
    fill(255);

}
void back()
{

// Draw points
for (int i = 0; i < 100; i++) 
{
  for (int j = 0; j < 100; j++)
  {
    stroke(100);
rect(i*40,j*40,100,100);
}
}
}
 

