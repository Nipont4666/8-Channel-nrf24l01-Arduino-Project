/*
 2.4G Transmitter nRF24L01 module Arduino
 8 Channel Radio Control + L298N dual motor control + Gripper arm for Tracked robot
 Written by: Pason Tanpaiboon
 November 2016
 Version.1.3
 Transmitter sketch  Tx_8CH_V2
 
This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ 
or send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 */
 
#include <Servo.h> 
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(9,53);//9,53 for MEGA2560

const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };//2 pipes communication
uint8_t received_data[9];
uint8_t num_received_data = sizeof(received_data);

//Motor Right
const int enableR = 3;
const int MotorR1 = 4;
const int MotorR2 = 5;

//Motor Left
const int enableL = 6;
const int MotorL1 = 7;
const int MotorL2 = 8;

int enableRval ;
int enableLval ;
int MotorR1val ;
int MotorR2val ;
int MotorL1val ;
int MotorL2val ;

int MotorRspeed ;
int MotorLspeed ;

int MotorMaxspeed;//Varaiable max motor speed
int SpeedMultiplier;//Varaiable multiplier

/////////////////////////////////////////////////////////
int LXaxis_Val ;
int LYaxis_Val ;
int LSwitch_Val;

int RXaxis_Val ;//Servo arm2
int RYaxis_Val ;//Servo arm1
/////////////////////////////////////////////////////////
int Aux1pot_Val ;//Servo arm1
int Aux2pot_Val ;//Servo arm2
int Aux3pot_Val ;//Servo gripper(Mpot_Val)
int EmerSw_Val ;//Th. Hold Switch value
int Arm_servoVaL3t ;

int Arm_servoVaL1 = 0;  
int Arm_servoVaL2 = 0;  
int Arm_servoVaL3 = 0;  

Servo Armservo1;
Servo Armservo2;
Servo Armservo3;

void setup(void)
{
  Serial.begin(115200);
 
  pinMode (enableR, OUTPUT);
  pinMode (MotorR1, OUTPUT);
  pinMode (MotorR2, OUTPUT);  

  pinMode (enableL, OUTPUT);
  pinMode (MotorL1, OUTPUT);
  pinMode (MotorL2, OUTPUT);  
  
  digitalWrite(enableR , HIGH);
  digitalWrite(enableL , HIGH);
 ////////////////////////////////////////////////////////////////////// 
  Armservo1.attach(11);  // attaches the servo on pin 3 to the servo object 
  Armservo2.attach(12);
  Armservo3.attach(13); 
  
  Armservo1.write(90);//neutral
  Armservo2.write(90);
  Armservo3.write(90);
  
  radio.begin();

  radio.setDataRate(RF24_250KBPS); 
  radio.setPALevel(RF24_PA_MAX); 
  radio.setChannel(108);
  radio.setCRCLength(RF24_CRC_8);
  
  radio.setRetries(15,15);
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();
  radio.printDetails();
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
  radio.startListening();
 

}//void setup

void loop(void)
{
  
  ///////////////////////////////Radio reading///////////////////////////////////////////////////////     
  if ( radio.available() )
  {
    bool done = false;
    while (!done)
    {
      done = radio.read(&received_data, num_received_data);
      delay(10);
    }// while (!done)

    Serial.print (received_data[0]); 
    Serial.print ("---"); 
    Serial.print (received_data[1]); 
    Serial.print ("---"); 
    Serial.print (received_data[2]); 
    Serial.print ("---"); 
    Serial.print (received_data[4]); 
    Serial.print ("---"); 
    Serial.print (received_data[5]); 
    Serial.print ("---"); 
    Serial.print (received_data[6]); 
    Serial.print ("---");
    Serial.print (received_data[7]); 
    Serial.print ("---"); 
    Serial.print (received_data[8]); 
    Serial.print (" "); 
 
    radio.stopListening();

    radio.startListening();


if (received_data[0] == 0 && received_data[1] == 0  &&  received_data[2] == 0 &&  received_data[4] == 0  && received_data[5] == 0 && received_data[6] == 0 && received_data[8] == 0  )// radio error

  {
    
      MotorRspeed = 0;
      MotorLspeed = 0;
 
      MotorR1val = 0;
      MotorR2val = 0;
      MotorL1val = 0;
      MotorL2val = 0;
      Engine();
      Serial.print("Transmission error 0!");
  }

if (received_data[0] == 255 && received_data[1] == 255  &&  received_data[2] == 255 &&  received_data[4] == 255 && received_data[5] == 255 && received_data[6] == 255 && received_data[8] == 255 )// radio error

  {
      
      MotorRspeed = 0;
      MotorLspeed = 0;
 
      MotorR1val = 0;//off 
      MotorR2val = 0;
      MotorL1val = 0;
      MotorL2val = 0;
      Engine();
      Serial.print("Transmission error 255!");
  }

/****************************Emergency Stop********************/
EmerSw_Val = received_data[8];
delay(10);
if ( EmerSw_Val == 1 )
 {
      
      MotorRspeed = 0;
      MotorLspeed = 0;
 
      MotorR1val = 0;
      MotorR2val = 0;
      MotorL1val = 0;
      MotorL2val = 0;
      Engine();
      Armservo1.write(135);
      Armservo2.write(0);
      Armservo3.write(90);
      Serial.println("Emergency Stop!");
     } 
else {
 /**************************************************************Left X axis > Aileron*********************************************************************/
    LXaxis_Val = received_data[0];
    delay(10);
    LXaxis_Val = LXaxis_Val*10;

 /**************************************************************Left Y axis > Elevator*********************************************************************/
    LYaxis_Val = received_data[1];
    delay(10);
    LYaxis_Val = LYaxis_Val *10;
    
/**************************************************************Right Y axis > Aux1 axis =Servo arm 1*********************************************************************/    
    RYaxis_Val = received_data[4];
    delay(10);
    RYaxis_Val = RYaxis_Val*10;
    Arm_servoVaL1 = RYaxis_Val;
    Arm_servoVaL1 = map(Arm_servoVaL1,0, 1020,0, 180);
    
    if (Arm_servoVaL1 > 135 )
    {
      Armservo1.write(135);
   }
    
  else 
  { 
    Armservo1.write(Arm_servoVaL1);
  }    
  
/**************************************************************Right X axis >  Aux2 axis=Servo arm 2*********************************************************************/    
    RXaxis_Val = received_data[5];
    delay(10);
    RXaxis_Val = RXaxis_Val*10;
     Arm_servoVaL2 = RXaxis_Val;
     Arm_servoVaL2 = map(Arm_servoVaL2, 0 , 1020, 0, 180);
     Armservo2.write(Arm_servoVaL2);

/**************************************************************Potentiometer >  Aux4 axis = Gripper*********************************************************************/        
    Aux3pot_Val = received_data[7];
    delay(10);
    Aux3pot_Val = Aux3pot_Val*10;
    Arm_servoVaL3t = Aux3pot_Val; 
    Arm_servoVaL3t = map(Arm_servoVaL3t, 0 , 1020, 180, 0);
  
 if (Arm_servoVaL3t < 90)
 {
 Armservo3.write(90);
 }
  if (Arm_servoVaL3t > 150)
 {
 Armservo3.write(150);
 }
 else 
 {
    Armservo3.write(Arm_servoVaL3t);
 }
 
/**************************************************************Potentiometer >  Aux4 ********************************************************************/   
    MotorMaxspeed = received_data[2];
      delay(10);
    
    MotorMaxspeed = map(MotorMaxspeed, 85 , 23, 255, 0);


 /*************************************************************Stopping***************************************************************/      
      if (LXaxis_Val >= 550  && LXaxis_Val <= 590 && LYaxis_Val >= 410  && LYaxis_Val <= 460  )//Y-stick neutral + X-stick neutral 
     {
      
      MotorRspeed = 0;
      MotorLspeed = 0;
 
      MotorR1val = 0;
      MotorR2val = 0;
      MotorL1val = 0;
      MotorL2val = 0;
 
      Engine();
      Serial.print("Stop");
     } 
  
/*************************************************************Straight Forward***************************************************************/   
     if ( LYaxis_Val > 450 && LXaxis_Val < 600  &&  LXaxis_Val > 550 )//Move Y-stick forward + X-stick neutral
     {
    
     MotorRspeed =  map(LYaxis_Val,780,110,MotorMaxspeed,0);

     MotorLspeed =  MotorRspeed*1.1 ;
       
      MotorR1val = 1;
      MotorR2val = 0;
      MotorL1val = 1;
      MotorL2val = 0;
      Engine();
      Serial.print("Forward");
     }      

/*************************************************************Straight Backward***************************************************************/        
  if (   LYaxis_Val < 370 &&  LXaxis_Val <= 600   &&  LXaxis_Val >= 550  )//Move Y-stick backward + X-stick nutral    
   {
      
     MotorRspeed = map(LYaxis_Val,110,780,MotorMaxspeed,0);
         
     MotorLspeed = MotorRspeed*1.5;
        
      MotorR1val = 0;
      MotorR2val = 1;
      MotorL1val = 0;
      MotorL2val = 1;   
      Engine();
      Serial.print("Backward");
   }
       
/********************************************************************Forward Right Direction***************************************************************/   
  if (LYaxis_Val > 450 && LXaxis_Val > 600 )//Move Y-stick forward + X-stick right    
     {
       
     MotorRspeed = 0;
           
     MotorLspeed = map(LYaxis_Val,110,780,0,MotorMaxspeed)*1.3 ;
      
      MotorR1val = 1;
      MotorR2val = 0;
      MotorL1val = 1;
      MotorL2val = 0;
      Engine();
      Serial.print("Forward Right");
     }
   /********************************************************************Forward Left Direction***************************************************************/      
     
  if ( LYaxis_Val >= 0 &&  LYaxis_Val > 450 && LXaxis_Val >= 0 && LXaxis_Val < 550 )//Move Y-stick forward + X-stick left    
     {
          
     MotorRspeed = map(LYaxis_Val,110,780,0,MotorMaxspeed) ;
     
     MotorLspeed = 0;
   
      MotorR1val = 1;
      MotorR2val = 0;
      MotorL1val = 1;
      MotorL2val = 0;   
      Engine();
      Serial.print("Forward Left");
     }
     /********************************************************************Backward Right Direction***************************************************************/      
   if (LYaxis_Val < 430 && LXaxis_Val > 600 )//Move Y-stick forward + X-stick right    
     {
       
     MotorRspeed = 0;
          
     MotorLspeed = map(LYaxis_Val,110,780,MotorMaxspeed,0)*1.3 ;
          
      MotorR1val = 0;
      MotorR2val = 1;
      MotorL1val = 0;
      MotorL2val = 1;
      Engine();
      Serial.print("Backward Right");
     }
     
   /************************************************Backward Left Direction***************************************************************/   
  if (LYaxis_Val < 400 && LXaxis_Val < 520 )//Move Y-stick backward + X-stick left    
     {
         
     MotorRspeed = map(LYaxis_Val,110,780,MotorMaxspeed,0);
         
     MotorLspeed = 0;
     
      MotorR1val = 0;
      MotorR2val = 1;
      MotorL1val = 0;
      MotorL2val = 1;   
      Engine();
      Serial.print("Backward Left");
     }   
  /*************************************************************Rotate Right***************************************************************/ 
   if ( LYaxis_Val > 420 && LYaxis_Val < 460 && LXaxis_Val > 600)//Move Y-stick nutral + X-stick right    
   {
         
     MotorRspeed = map(LXaxis_Val,290,870,0,MotorMaxspeed);
      
     MotorLspeed = MotorRspeed*1.3 ;
        
      MotorR1val = 0;
      MotorR2val = 1;
      MotorL1val = 1;
      MotorL2val = 0;   
      Engine();
      Serial.print("Rotate Right");
   } 
 /*************************************************************Rotate Left***************************************************************/    
   if ( LYaxis_Val > 420 && LYaxis_Val < 460 && LXaxis_Val < 560)//Move Y-stick nutral + X-stick right  
   {
     
     MotorRspeed = map(LXaxis_Val,290,870,MotorMaxspeed,0);
        
     MotorLspeed = MotorRspeed*1.3 ;
    
      MotorR1val = 1;
      MotorR2val = 0;
      MotorL1val = 0;
      MotorL2val = 1;  
      Engine();
      Serial.print("Rotate Left"); 
   }     
/////////////////////////////////////////////Serial Print/////////////////////////////////////////////////////////////////////////// 
     
    Serial.print (">>>"); 
    Serial.print(MotorRspeed);
    Serial.print ("---"); 
    Serial.print(MotorLspeed);
    Serial.print ("---"); 
    Serial.print(MotorR1val);
    Serial.print ("---"); 
    Serial.print(MotorR2val);
    Serial.print ("---"); 
    Serial.print(MotorL1val);
    Serial.print ("---"); 
    Serial.print(MotorL2val);
    Serial.print ("---"); 
    Serial.println(MotorMaxspeed);
 
  }//Emergency stop
  
  }// if ( radio.available() )

  else //No signal condition for safety
  {

   Serial.println ("No signal>>>Stop"); 
   MotorRspeed = 0;
   MotorLspeed = 0;
   MotorR1val = 0;
   MotorR2val = 0;
   MotorL1val = 0;
   MotorL2val = 0;
   Engine();
   
  }
 
}//void loop(void)

/*****************Driving motor *********************************************************************/

void Engine() {
//Right Motor 
  analogWrite(enableR , MotorRspeed);// 
  digitalWrite (MotorR1,MotorR1val);
  digitalWrite (MotorR2,MotorR2val);  
  delay (100);
  
 //Left Motor 
  analogWrite(enableL , MotorLspeed);// 
  digitalWrite (MotorL1,MotorL1val);
  digitalWrite (MotorL2,MotorL2val);  
  delay (100);
}

