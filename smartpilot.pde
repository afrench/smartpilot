
#include <SoftwareSerial.h>
#include <Wire.h>
//#include "smartpilot.h"

int  youreOK = 1;     //a flag accessible by several functions. If it ever goes low alarm should sound;
int alarmPin = 1;    //location of alarm pin
int heading[10];
int desired_heading;      //the heading we'd like to hold
int heading_data[3];
int course_delta[3]; //subtract Present - Past
int course_derivative[2]; //subtract Present - Past
float ndderivative;
float spoke_proportional;   //number of spokes from proportional
float spoke_derivative;    //number of spokes fromderivative
float spoke_ndderivative;  //number of spokes from second derivative
float k_proportional;  //constnat for proportional
float k_derivative;    //constant for derivative 
float k_ndderivative;      //constant for second derivative
float k_spoke_time;    //constant defines a spoke
float spoke_total;          //sum of all spoke times in miliseconds
int period;          //how frequently you sample, after 3 samples you run spokulator. 
int on_time;

void setup (){
  Serial.begin(9600); 
  youreOK = 1;		       //initialize sate as all ok
  period = 3000;                //time between samples. after 3 samples we run spokulator
  k_proportional = 1;     //constant applied to course_delta in spokulator
  k_derivative = 1;        //constant applied to rate of course change in spokulator
  k_ndderivative = .5;     //constant applied to rate of change of rate of change in spokulator
  k_spoke_time = 25;      //time in miliseconds to run the motor for every spoke.
  spoke_total = 0;         //initialize at zero  
  on_time = 0;
  setup_motor();
  setup_compass();
  delay(1000);  //wait for the first bit of spurious data to pass
  desired_heading = read_compass();
}

void loop (){  
    //Serial.print("desired_heading: ");
    //Serial.println(int (desired_heading));
    for (int i=0; i<2; i++){
      heading_data[i]= read_compass();
      delay (period);
    }
    //Serial.print ("current_heading: ");
    //Serial.println (heading_data[0]);  
    course_delta[0] = heading_data[0] - desired_heading;
    Serial.print ("course_delta[0]: ");
    Serial.println (course_delta[0]);
    course_delta[1] = heading_data[1] - desired_heading;
    course_delta[2] = heading_data[2] - desired_heading;
    course_derivative[0] = (heading_data[0] - heading_data[1])/(period/100);
    course_derivative[1] = (heading_data[1] - heading_data[2])/(period/100);
    ndderivative = (course_derivative[0] - course_derivative[1])/(period/100);
    spokulator ();
    turn (on_time);
  }


//end of main section
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//begin definition of Motor functions
  
int port_pin = 2;   //pin on Arduino board that will turn the boat to port when energized
int starboard_pin = 3;
  
int setup_motor(){
 //set initial state of motor to off
 pinMode(2, OUTPUT);
 pinMode(3, OUTPUT);
 digitalWrite(2, LOW); 
 digitalWrite(3, LOW); 
}

int turn(int on_time){
  
  Serial.print("  Starting a ");
  Serial.print(int(on_time));
  Serial.println(" turn. ");
  if (on_time < 0){
    digitalWrite(port_pin, HIGH); 
  }
   else if (on_time > 0){
     digitalWrite(starboard_pin, HIGH); 
  }
  else{
    return 0;
  }
  delay (abs(on_time));
  digitalWrite(port_pin, LOW);
  digitalWrite(starboard_pin, LOW);
  return 0;
  Serial.println("Turn completed.");
}
  
//end motor section

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Spoke calculations;
int spokulator(){  //define function to deal with course corrections
  spoke_proportional =  -(float)course_delta[0] * k_proportional;    //number of spokes from proportional 
  Serial.print ("spoke_proportional: ");
  Serial.print ((int)spoke_proportional);  
  spoke_derivative = (float)course_derivative[0] * k_derivative;    //number of spokes from derivative
  Serial.print ("  spoke_derivative: ");
  Serial.print ((int)spoke_derivative);  
  spoke_ndderivative = ndderivative * k_ndderivative;    //number of spokes from second derivative
  Serial.print ("  spoke_ndderivative: ");
  Serial.print ((int)spoke_ndderivative);   
  spoke_total = spoke_proportional + spoke_derivative + spoke_ndderivative; 
  Serial.print ("  spoke_total: ");
  Serial.print ((int)spoke_total);  
  on_time = (int)(spoke_total * k_spoke_time/10);                                         //not sure this is how it's done
  Serial.print ("  on_time: ");
  Serial.print (on_time);  

} 
//END COURSE CORRECTION SECTION
 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//WORKING WITH THE COMPASS

int HMC6352Address = 0x42; 
int slaveAddress;             // This is calculated in the setup() function 
int ledPin = 13; 
boolean ledState = false; 
byte headingData[2]; 
int i, headingValue; 

int setup_compass(){
  //begin setup of compass
  slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI 
  pinMode(ledPin, OUTPUT);      // Set the LED pin as output 
  Wire.begin(); 
  //end setup compass
}

int get_average_heading(){
  int sum=0;
  for (int i=0; i<=9; i++){
    sum += heading[i];
  }
  return sum/10;
}

int update_heading_stack(){
  for (int i=9; i>=0; i--){
    heading[i]=heading[i-1];
  }
  heading[0] = read_compass();
}

int read_compass(){  
  int offset=0;   //offset is the ammount to adjust compass readings by to match installed angle
  // Flash the LED on pin 13 just to show that something is happening 
  // Also serves as an indication that we're not "stuck" waiting for TWI data 
  ledState = !ledState; 
  if (ledState) { 
    digitalWrite(ledPin,HIGH); 
  } 
  else { 
    digitalWrite(ledPin,LOW);
  } 
  // Send a "A" command to the HMC6352 
  // This requests the current heading data 
  Wire.beginTransmission(slaveAddress);
  Wire.send("A");              // The "Get Data" command 
  Wire.endTransmission(); 
  delay(10);                   // The HMC6352 needs at least a 70us (microsecond) delay 
                               // after this command.  Using 10ms just makes it safe 
  // Read the 2 heading bytes, MSB first 
  // The resulting 16bit word is the compass heading in 10th's of a degree 
  // For example: a heading of 1345 would be 134.5 degrees
  Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
  i = 0;
  while(Wire.available() && i < 2) {
    headingData[i] = Wire.receive();
    i++;
  }
  headingValue = (headingData[0]*256 + headingData[1]) + offset;  // Put the MSB and LSB together 
//  Serial.print("Current heading: "); 
//  Serial.print(int (headingValue / 10));     // The whole number part of the heading 
//  Serial.print("."); 
//  Serial.print(int (headingValue % 10));     // The fractional part of the heading 
//  Serial.println(" degrees"); 
  return headingValue;
  //delay(200); 
}
//end WORKING WITH THE COMPASS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* CUTTING ROOM FLOOR

  while (youreOK){
    for (int i=0; i<9; i++){
      update_heading_stack();
      delay(200);
      
    Serial.print(": Current heading: ");
    Serial.print(int (average_heading));
    Serial.print("\tcourse_delta: ");
    Serial.print(int (course_delta));

    */
