/*
 2.4G Transmitter nRF24L01 module Arduino
 8 Channel Radio Control 
 Written by: Pason Tanpaiboon
 November 2016
 Version.1.3
 Receiver sketch  Rx_8CH_gripper_arm_robot_V1

 This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
 To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ 
 or send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(9,10);

uint8_t data[9] ; //8 data ptt
const uint8_t buffer_size = sizeof(data);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };//two pipes communication 

const int Ail_pot = A1;//Aileron
const int Ele_pot= A2;//Elevator
const int Thr_pot = A3;//Throttle
const int Rud_pot = A4;//Rudder
const int Th_hold = 7;//Throttle hold
const int Th_hold_LED = 3;//Th Hold LED

const int Aux1_pot = A5;// Auxillary
const int Aux2_pot = A6;// Auxillary
const int Aux3_pot = A7;// Auxillary
const int Aux4_pot = A0;// Auxillary

int Ail_value = 0; 
int Ele_value = 0;    
int Thr_value = 0;
int Rud_value = 0;
int Th_hold_value = 0;

int Aux1_value = 0; 
int Aux2_value = 0; 
int Aux3_value = 0; 
int Aux4_value = 0; 

void setup(void)
{
   Serial.begin(115200);
   pinMode(Ail_pot,INPUT );
   pinMode(Ele_pot,INPUT );
   pinMode(Thr_pot,INPUT );
   pinMode(Rud_pot,INPUT );
   pinMode(Th_hold,INPUT );
   
   pinMode(Aux1_pot,INPUT );
   pinMode(Aux2_pot,INPUT );
   pinMode(Aux3_pot,INPUT );
   pinMode(Aux4_pot,INPUT );
   
   pinMode(Th_hold_LED,OUTPUT );
   
  radio.begin();
  
  radio.setDataRate(RF24_250KBPS); 
  radio.setPALevel(RF24_PA_MAX); 
  radio.setChannel(108);
  radio.setCRCLength(RF24_CRC_8);
 
  radio.setRetries(15,15);
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();
  radio.printDetails();
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.stopListening();
   
}

void loop(void)
{
//////////////////////////////////////////////////////////////////////// 
  Ail_value = analogRead(Ail_pot); 
  Ail_value = Ail_value/10;
  data[0] = Ail_value;
  
//////////////////////////////////////////////////////////////////////// 
  Ele_value = analogRead(Ele_pot); 

  data[1] = Ele_value/10;//scaling escval
  
//////////////////////////////////////////////////////////////////////// 
  Thr_value = analogRead(Thr_pot); 
  

  data[2] = Thr_value/10;//scaling escval
  
//////////////////////////////////////////////////////////////////////// 
  Rud_value = analogRead(Rud_pot); 
  
 
  data[3] = Rud_value/10;//scaling escval
     
////////////////////////////////////////////////////////////////////////      
  Aux1_value =  analogRead(Aux1_pot);    
  
  data[4] = Aux1_value/10;//scaling escval    
 
////////////////////////////////////////////////////////////////////////      
  Aux2_value =  analogRead(Aux2_pot);    
  
  data[5] = Aux2_value/10;//scaling escval   
 
////////////////////////////////////////////////////////////////////////      
  Aux3_value =  analogRead(Aux3_pot);    
  
  data[6] = Aux3_value/10;//scaling escval    
  
////////////////////////////////////////////////////////////////////////      
  Aux4_value =  analogRead(Aux4_pot);    
  
  data[7] = Aux4_value/10;//scaling escval  
 
/////////////////////////////////////////////////////////////////////////
 Th_hold_value = digitalRead(Th_hold);
 delay(15); 
 
  if (Th_hold_value == HIGH){
     
  data[8] = 1;//Th Hold ON
  digitalWrite(Th_hold_LED,HIGH);
  
  }
else {
  data[8] = 0;
  digitalWrite(Th_hold_LED,LOW);
}
 
/////////////////////////////////////////////////////////////////////////

  radio.stopListening();

  bool ok = radio.write(  data ,sizeof(data) );
  delay(30);
  radio.startListening();
  delay(20);
  if (ok){
    Serial.print("data[0]=");
    Serial.print(data[0]);
    Serial.print(" data[1]=");
    Serial.print(data[1]);
    Serial.print(" data[2]=");
    Serial.print(data[2]);
    Serial.print(" data[3]=");
    Serial.print(data[3]);
    Serial.print(" data[4]=");
    Serial.print(data[4]);
    Serial.print(" data[5]=");
    Serial.print(data[5]);    
    Serial.print(" data[6]=");
    Serial.print(data[6]);
    Serial.print(" data[7]=");
    Serial.print(data[7]);  
    Serial.print(" data[8]=");
    Serial.println(data[8]);  
  
  }
  else
    Serial.println("failed\n\r");

}//void loop()
 
 
 
    

  
















