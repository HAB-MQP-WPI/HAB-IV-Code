/* N. Chantre, J. Lopez 
 *  ECE MQP Team 5
 *  Base Station Arduino Code
 */

//Include Statements
#include <AccelStepper.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//Stepper Motor Global Variables
float coords[3] = {0.0, 0.0, 0.0};
int unitsToRotate = 0;
float oldDegs = 0.0;
float diff = 0;
float offset = 0;

//Change to actual coordinates
float bs_lon = -71.7829575;
float bs_lat = 42.281262833;
float bs_alt = 200;

//Constant Declaration
float pi = 3.1415;

/* Port Declaration
*  Stepper Motor will use pins 6 and 7
*  Linear Actuator will use pins 10 and 11
*/

// Define a stepper and the pins it will use
AccelStepper stepper(1,7,6);

//Linear Actuator Globals
byte Speed = 0; // Intialize Varaible for the speed of the motor (0-255);
int RPWM = 10;  //connect Arduino pin 10 to IBT-2 pin RPWM
int LPWM = 11;  //connect Arduino pin 11 to IBT-2 pin LPWM
int position;

void setup() 
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    
    //Stepper Setup: Setting speed and acceleration for our motor
    stepper.setMaxSpeed(1000.0);
    stepper.setAcceleration(1000.0);
    stepper.setCurrentPosition(0);

    //Linear Actuator Setup
    pinMode(10, OUTPUT); // Configure pin 10 as an Output
    pinMode(11, OUTPUT); // Configure pin 11 as an Output
    Speed = 100;
    position = 1;
    //reset position down.
    analogWrite(RPWM, 0);
    analogWrite(LPWM, Speed);
    delay(13000); // 6.5 Seconds is how long it take to go down

     // Stop Actuator
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    delay(2000); // 10 Seconds

}

// ------ BEGINNING OF FUNCTIONS ------

//Returns angle between base station & payload
float computeDegs(float bs_lon, float bs_lat, float payl_lon, float payl_lat)
{

    //NOTE: base station calculations
    float bs_lat_rad = bs_lat * pi / 180;
    float bs_lon_rad = bs_lon* pi / 180;

    //NOTE: payload calculations
    float payl_lat_rad = payl_lat * pi / 180;
    float payl_lon_rad = payl_lon * pi / 180;

    float lonDiff = payl_lon_rad - bs_lon_rad;
    float latDiff = payl_lat_rad - bs_lat_rad;

    float degToRotate = atan2(latDiff, lonDiff) * (180/pi);

    Serial.println("Computed Degs:");
    Serial.println(degToRotate);

    return degToRotate;
}


float bearing(float lat1, float lon1, float lat2, float lon2)
{
    //lat1&lon1 is our base station
    float lat1_rad = lat1 * pi / 180;
    float lon1_rad = lon1 * pi / 180;
    float lat2_rad = lat2 * pi / 180;
    float lon2_rad = lon2 * pi / 180;

    float y = sin(lon2_rad - lon1_rad) * cos(lat2_rad);
    float x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(lon2_rad - lon1_rad);

    float brng = atan2(y, x) * 180 / pi;
    if(brng < 0)
    {
        brng = brng + 360;
    }

    return brng;
}

//This function takes in the degrees(angle) between the base station and the payload to map this value to a motor value for the correct rotation
int mapToMotor(float degs)
{
  int retUnits;
  float unitPerDeg = 18.056;

  retUnits = floor(degs*unitPerDeg);

  return retUnits;
}

//This function receives parsed GPS coordinates being sent from the PC to our Arduino Uno
void recvGPSCoords()
{
  float longitude, latitude, altitude;
  String recvData;

  while(Serial.available()==0)
  {
    //wait
  }

  //longitude
  recvData = Serial.readStringUntil(',');
  longitude = recvData.toFloat();
  coords[0] = longitude;

  //latitude
  recvData = Serial.readStringUntil(',');
  latitude = recvData.toFloat();
  coords[1] = latitude;

  //altitude
  recvData = Serial.readStringUntil('\n');
  altitude = recvData.toFloat();
  coords[2] = altitude;

}

void rotate()
{
  stepper.runToNewPosition(unitsToRotate);
  //change to variable that will change with each call of the point function
  stepper.stop();
  delayMicroseconds(1000);
}

float computeDiff(float currDegs)
{
  float retVal = 0.0;
  
  if(currDegs > offset)
  {
    retVal = currDegs - offset;
  }
  else if(currDegs < offset)
  {
    retVal = offset - currDegs;
  }

  offset = currDegs;

  return retVal;
}

// 10in stroke
// 90deg/10in = 9deg per in
// 17 second up
// 13 seconds down
// 17/10 = 1.7s/in up
// 13/10 = 1.3 s/in down
// 3ft offset from the ground
float pointer(float pay_alt, float pay_lat, float pay_lon){
  float lonDiff1 = pay_lon - bs_lon;
  float latDiff1 = pay_lat - bs_lat;

  //Hypotenuse
  float Hypotenuse1 = (pow(latDiff1,2)) + (pow(lonDiff1,2));
  float diagdist=sqrt(Hypotenuse1); //adjacent
  float angle = tan((pay_alt-bs_alt)/diagdist); //returns angles in radians
  float angledegree = angle * (180/pi); //conversion to degrees
  return angledegree;
}
  int split(float angledegrees, float pay_alt){
    int position;
    if (angledegrees <= 9){
      position = 1;
    }
    else if (angledegrees <= 10 && angledegrees >= 18){
      position = 2;
    }
    else if (angledegrees <= 19 && angledegrees >= 27){
      position = 3;
    }
    else if (angledegrees <= 28 && angledegrees >= 36){
      position = 4;
    }
    else if (angledegrees <= 37 && angledegrees >= 45){
      position = 5;
    }
    else if (angledegrees <= 46 && angledegrees >= 54){
      position = 6;
    }
    return position;
  }

// ------ END OF FUNCTIONS ------

void loop()
{
  //Read serial data from Base Station Computer
  recvGPSCoords();
  float pl_lon = coords[0];
  float pl_lat = coords[1];
  float pl_alt = coords[2];

  Serial.println("ARD RECV: " + String(pl_lon) + " " + String(pl_lat));

  //Calculate bearing
  float retDegs = bearing(bs_lat, bs_lon, pl_lat, pl_lon);

//TO PREVENT TURNS GREATER THAN 180 degrees
  if(retDegs>180)
  {
    retDegs = -1*(360-retDegs);
  }

  //Compute value and determine direction to rotate motor
  unitsToRotate = mapToMotor(retDegs);
  
  //Rotate Motor
  stepper.runToNewPosition(unitsToRotate);
  stepper.stop();
  
  //Assign offset
  offset = retDegs;//offset will always be equal to retDegs

  int oldposition = position;
  //Linear Actuator
  float angledegs = pointer(pl_alt, pl_lat, pl_lon);
  position = split(pl_alt, angledegs);

  if (position > oldposition){
    //Extend Actuator
    analogWrite(RPWM, Speed);
    analogWrite(LPWM, 0);
    delay(1700); //1.7 seconds to go up 1in
    // Stop Actuator
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    delay(1000);
  } else if(position < oldposition){
    // Retract Actuator
    analogWrite(RPWM, 0);
    analogWrite(LPWM, Speed);
    delay(1300); //1.3 rate doing down 1in
    //Stop Actuator
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    delay(1000);
  } 
}
