/**********************************************************************
* Filename    : AlarmSystem.c
* Description : System on the RPi that simulates an alarm system
* Author      : Christopher Milian and Aaron Braun
* modification: 7/20/2020
**********************************************************************/

// How to Compile: gcc AlarmSystem.c -o AlarmSystem -lwiringPi -lpthread -lwiringPiDev

// Libraries 
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
#define isrPin 29 //which one doesn't work?

// For Ultrasonic Sensor
#define trigPin 4
#define echoPin 5
#define MAX_DISTANCE 220        // Define the maximum measured distance
#define timeOut MAX_DISTANCE*60 // Calculate timeout according to the maximum measured distance

// For LCD Screen
#define pcf8574_address 0x27    // default I2C address of PCF8574
#define BASE 64                 // BASE any number above 64

//Define the output pins of the PCF8574, which are directly connected to the LCD1602 pin.
#define RS      BASE+0
#define RW      BASE+1
#define EN      BASE+2
#define LED     BASE+3
#define D4      BASE+4
#define D5      BASE+5
#define D6      BASE+6
#define D7      BASE+7

int lcdhd, r, g, b, i;

bool status;

// Functions for Ultrasonic Sensor
int pulseIn(int pin, int level, int timeout)  // Function pulseIn: obtain pulse time of a pin
{
   struct timeval tn, t0, t1;
   long micros;
   gettimeofday(&t0, NULL);
   micros = 0;
   while (digitalRead(pin) != level)
   {
      gettimeofday(&tn, NULL);
      if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
      micros += (tn.tv_usec - t0.tv_usec);
      if (micros > timeout) return 0;
   }
   gettimeofday(&t1, NULL);
   while (digitalRead(pin) == level)
   {
      gettimeofday(&tn, NULL);
      if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
      micros = micros + (tn.tv_usec - t0.tv_usec);
      if (micros > timeout) return 0;
   }
   if (tn.tv_sec > t1.tv_sec) micros = 1000000L; else micros = 0;
   micros = micros + (tn.tv_usec - t1.tv_usec);
   return micros;
}

void setArmed()
{
	status = true;
}

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

// Function that displays distance from sensor onto LCD
float printDistance(){
    float distance = 0;
    distance = getSonar();
    lcdPosition(lcdhd,0,0);  // Set the LCD cursor position to (0,0)
    lcdPrintf(lcdhd,"Dist: %.2fcm", distance);  //Display system time on LCD
    printf("ARMED: Dist: %.2fcm\n", distance);
    return distance;
}


void setupLedPin(void)
{
	softPwmCreate(ledPinRed,  0, 100);	//Creat SoftPWM pin for red
	softPwmCreate(ledPinGreen,0, 100);  //Creat SoftPWM pin for green
	softPwmCreate(ledPinBlue, 0, 100);  //Creat SoftPWM pin for blue
}

void setLedColor(int r, int g, int b)
{
	softPwmWrite(ledPinRed,   r);	//Set the duty cycle 
	softPwmWrite(ledPinGreen, g);   //Set the duty cycle 
	softPwmWrite(ledPinBlue,  b);   //Set the duty cycle 
}

int main(void)
{
    char command[50];
    long distance;
    status = false;

    strcpy(command, "echo Chris is gay");

    printf("Sensor and Screen are Initializing.\n\n");
    wiringPiSetup(); //Initialize wiringPi.
    wiringPiISR (isrPin, INT_EDGE_RISING,  &setArmed) ;
    pcf8574Setup(BASE,pcf8574_address);  // Initialize PCF8574
    for(i=0;i<8;i++){
        pinMode(BASE+i,OUTPUT);  // Set PCF8574 port to output mode
    }

    digitalWrite(LED,HIGH);  // Turn on LCD backlight
    digitalWrite(RW,LOW);    // Allow writing to LCD
    lcdhd = lcdInit(2,16,4,RS,EN,D4,D5,D6,D7,0,0,0,0);  // Initialize LCD and return “handle” used to handle LCD

    pinMode(trigPin,OUTPUT);
    pinMode(echoPin,INPUT);

    setupLedPin();

    if(lcdhd == -1){
        printf("LCD could not be initialized. Exiting.");
        return 1;
    }
    while(1){
	if(status == false){
	   lcdPosition(lcdhd,0,0);
	   lcdPrintf(lcdhd, "Disarmed");
	   setLedColor(99,0,99);
	}
	else{
        	//r = random()%100;  //get a random in (0,100)
		//g = random()%100;  //get a random in (0,100)
		//b = random()%100;  //get a random in (0,100)
		r = 1;
		g = 99;
		b = 99;
		setLedColor(r,g,b);//set random as the duty cycle value
		printf("r=%d,  g=%d,  b=%d \n",r,g,b);
	        distance = printDistance();
       		delay(500);
		if(distance < 20){
			system(command);
			while(1){
				lcdPosition(lcdhd,0,0);
				lcdPrintf(lcdhd, "!INTRUDER ALERT!");
				setLedColor(99,99,0);
		}}
    }}

	return 1;
}

