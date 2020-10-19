/*
  Exercise 1: This exercise is to turn on an LED built into the UNO using the Port connected to digital pin 13. 
  You will need to write a program that can turn on this LED and the rewrite the program turn off this LED.
  For this exercise you cannot use the Arduino standard library. 
  Code written by:
          Member 1 SID xxxxxxxx
          Member 2 SID xxxxxxxx
          Member 3 SID xxxxxxxx
  Date: xx/xx/xxxx
*/

#include <avr/io.h> //Include the standard library required for programming the avr microprocessor

/* 
    Once you have chosen the Port and Pin you will be using, you would need to set a data direction for that port. This can be 
    done by configuring the data direction register for that port to output{set the bit to 1}
    {configuring a port for input is to set it to 0}. To turn on the LED you will need to toggle the specific pin on, 
*/
  
ISR(TIMER2_OVF_vect){
  int gfy = 0;
}

void setup()
{
  //<-------- Here, set the Data direction register for specifc Portx Pin x to output. 
  DDRB |= (1<<DDB1);
  DDRB |= (1<<DDB2);

}

void loop() {
    //<------- Here, set the specific Portx Pinx to high{1} to turn on and set to low{0} to turn off.
}
