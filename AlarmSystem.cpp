/**********************************************************************
* Filename    : AlarmSystem.c
* Description : System on the RPi that simulates an alarm system
* Author      : Christopher Milian and Aaron Braun
* modification: 7/20/2020
**********************************************************************/

// How to Compile: g++ AlarmSystem.cpp -o AlarmSystem -lwiringPi -lpthread -lwiringPiDev

// Libraries 
#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <pcf8574.h>
#include <lcd.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// For RBG LED
#define ledPinRed    0
#define ledPinGreen  1
#define ledPinBlue   2

//interrupt pin
#define isrPin 29 

// For Ultrasonic Sensor
#define trigPin 4
#define echoPin 5
#define MAX_DISTANCE 220        
#define timeOut MAX_DISTANCE*60 

// For LCD Screen
#define pcf8574_address 0x27    
#define BASE 64                 

//Define the output pins of the PCF8574, which are directly connected to the LCD1602 pin.
#define RS      BASE+0
#define RW      BASE+1
#define EN      BASE+2
#define LED     BASE+3
#define D4      BASE+4
#define D5      BASE+5
#define D6      BASE+6
#define D7      BASE+7

using namespace std;

int lcdhd, redLED, greenLED, blueLED;
bool status;

/*
* Calculates the pulse time of selected pin.
* Parameters: pin, level, timeout
* Return: 0 or micros
*/
int pulseIn(int pin, int level, int timeout){

   struct timeval tn, t0, t1;
   long micros = 0;
   
   gettimeofday(&t0, NULL);

   while (digitalRead(pin) != level){

      gettimeofday(&tn, NULL);

      if (tn.tv_sec > t0.tv_sec){
        micros = 1000000L; 
        }else{ 
          micros = 0;
          micros += (tn.tv_usec - t0.tv_usec);
        }
      
      if (micros > timeout){
          return 0;
        } 
   }

   gettimeofday(&t1, NULL);

   while (digitalRead(pin) == level){

      gettimeofday(&tn, NULL);

      if (tn.tv_sec > t0.tv_sec){
          micros = 1000000L;
        }else {
          micros = 0;
          micros = micros + (tn.tv_usec - t0.tv_usec);
        }
      if (micros > timeout){
          return 0;
        } 
   }
   if (tn.tv_sec > t1.tv_sec){
     micros = 1000000L;
   }else{
      micros = 0;
      micros = micros + (tn.tv_usec - t1.tv_usec);
   }
    return micros;
}

/*
* --
* Parameters: -
* Return: -
*/
void setArmed()
{
	status = true;
}

/*
* Calculates the distance using the ultrasonic sensor.
* Parameters: -
* Return: distance
*/
float getSonar(){  // Get the measurement result of ultrasonic module with unit: cm
    long pingTime;
    float distance;
    digitalWrite(trigPin,HIGH); //Send 10us high level to trigPin
    delayMicroseconds(10);
    digitalWrite(trigPin,LOW);
    pingTime = pulseIn(echoPin,HIGH,timeOut);           // Read plus time of echoPin
    distance = (float)pingTime * 340.0 / 2.0 / 10000.0; // Calculate distance with sound speed 340m/s
    return distance;
}

/*
* Sets system to Armed state
* Parameters: -
* Return: distance
*/
float printDistance(){
    float distance = 0;
    distance = getSonar();
    lcdPosition(lcdhd,0,0);
    lcdPrintf(lcdhd, "Armed Away");
    return distance;
}

/*
* Creates PWM for red, green, and blue pins for LED
* Parameters: void
* Return: -
*/
void setupLedPin(void)
{
	softPwmCreate(ledPinRed,  0, 100);
	softPwmCreate(ledPinGreen,0, 100);  
	softPwmCreate(ledPinBlue, 0, 100);  
}

/*
* Sets the duty cycle for each pin
* Parameters: redLED, greenLED, blueLED
* Return: -
*/
void setLedColor(int redLED, int greenLED, int blueLED)
{
	softPwmWrite(ledPinRed,   redLED);	 
	softPwmWrite(ledPinGreen, greenLED);   
	softPwmWrite(ledPinBlue,  blueLED);    
}

int main(void)
{
  char command[50];
  long distance = 0;
  status = false;

  printf("Sensor and Screen are Initializing.\n\n");

  // LCD, LED, and Ultrasonic Sensor Set up
  wiringPiSetup(); 
  setupLedPin();

  wiringPiISR (isrPin, INT_EDGE_RISING,  &setArmed);

  pcf8574Setup(BASE,pcf8574_address);  // Initialize PCF8574

  for(int setOutput=0; setOutput<8; setOutput++){
      pinMode(BASE+setOutput,OUTPUT);  // Set PCF8574 port to output mode
  }

  digitalWrite(LED,HIGH);  // Turn on LCD backlight
  digitalWrite(RW,LOW);    // Allow writing to LCD
  lcdhd = lcdInit(2,16,4,RS,EN,D4,D5,D6,D7,0,0,0,0);  // Initialize LCD and return “handle” used to handle LCD

  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
    
  // Checking if LCD was initialized
  if(lcdhd == -1){
      printf("LCD could not be initialized. Exiting.");
      return 1;
  }

  while(1){
	  if(!status){
	    lcdPosition(lcdhd,0,0);
	    lcdPrintf(lcdhd, "Disarmed Home");
	    setLedColor(99,0,99);
	    } 
	    else{
        setLedColor(1,99,99);
	      distance = printDistance();
       	delay(500);

		if(distance < 20){
			system(command);
      /*system("curl -X POST https://textbelt.com/text \
       --data-urlencode phone='3053211749' \
       --data-urlencode message='Your home alarm system has detected movement.' \
       -d key=EEL4709CSUMMER20"); */
			while(1){
				lcdPosition(lcdhd,0,1);
				lcdPrintf(lcdhd, "Sensor Triggered");
				setLedColor(99,99,0);
		    }
      }
    }
  }
	return 1;
}

