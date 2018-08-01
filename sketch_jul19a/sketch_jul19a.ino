#include <math.h>
#include "Controller.h"
#include "Motors.h"

// Pins
constexpr unsigned char leftMotorControlPin = 5;
constexpr unsigned char rightMotorControlPin = 3;

constexpr unsigned char photoTranPinLeft = A0;		// Right
constexpr unsigned char photoTranPinCenter = A1;		// Center
constexpr unsigned char photoTranPinRight = A2;		// Left

constexpr unsigned char rLedPin = 6;			// Red
constexpr unsigned char gLedPin = 8;			// Green
constexpr unsigned char bLedPin = 7;			// Blue

//constexpr unsigned char leftWheelSensor = A7;
//constexpr unsigned char rightWheelSensor = A6;

//constexpr unsigned char leftWheelSensor = 2;
//constexpr unsigned char rightWheelSensor = A6;


double error(unsigned short, unsigned short, unsigned short);


Controller steeringController;
Motors mainMotors(leftMotorControlPin, rightMotorControlPin);


void setup()
{
	// PWM
	pinMode(leftMotorControlPin, OUTPUT);
	pinMode(rightMotorControlPin, OUTPUT);

	// LEDs
	pinMode(rLedPin, OUTPUT);
	pinMode(gLedPin, OUTPUT);
	pinMode(bLedPin, OUTPUT);

	// Phototransistors
	pinMode(photoTranPinLeft, INPUT);
	pinMode(photoTranPinCenter, INPUT);
	pinMode(photoTranPinRight, INPUT);

	// Wheel Speed Sensors
	//pinMode(leftWheelSensor, INPUT);
	//pinMode(rightWheelSensor, INPUT);

	//Serial.begin(9600);

/*
	digitalWrite(rLedPin, HIGH);
	digitalWrite(gLedPin, HIGH);
	digitalWrite(bLedPin, HIGH);
//*/
}

void loop()
{
	// Sensors
	unsigned short photoTranLeft = analogRead(photoTranPinLeft);
	unsigned short photoTranCenter = analogRead(photoTranPinCenter);
	unsigned short photoTranRight = analogRead(photoTranPinRight);


// ------- Debug --------
/*
	Serial.print(photoTranLeft);
	Serial.print(", ");
	Serial.print(photoTranCenter);
	Serial.print(", ");
	Serial.print(photoTranRight);
	Serial.print(", ");
	Serial.println();
//*/

/*
	unsigned short photoTranTest = analogRead(A5);
	Serial.println(photoTranTest);
//*/

/*
	unsigned short wheelTest = analogRead(rightWheelSensor);
	Serial.println(wheelTest);
//*/
/*
	int wheelTest = digitalRead(leftWheelSensor);
	Serial.println(wheelTest);
/*/

	//delay(1000);
	//delay(200);

//*
	
//*/
// ----------------------


	mainMotors.beginCalibMode();
	char carPos = static_cast<char>(mainMotors.convertControllerOutput(error(photoTranLeft, photoTranCenter, photoTranRight)));
	mainMotors.actuate();
	

	// LED Indicators
	switch (carPos) {
		case -1: {		// Car is off-track to the left -> red LED
			digitalWrite(rLedPin, HIGH);
			digitalWrite(gLedPin, LOW);
			digitalWrite(bLedPin, LOW);
			break;
		}
		case 0: {		// Car is centered -> green LED
			digitalWrite(rLedPin, LOW);
			digitalWrite(gLedPin, HIGH);
			digitalWrite(bLedPin, LOW);
			break;
		}
		case 1: {		// Car is off-track to the right -> blue LED
			digitalWrite(rLedPin, LOW);
			digitalWrite(gLedPin, LOW);
			digitalWrite(bLedPin, HIGH);
			break;
		}
	}
}

// Error function to be fed to the PD controller
double error(unsigned short l, unsigned short c, unsigned short r)
{
	return static_cast<double>(10 * ((l - r) / log(c) + 20));
}

