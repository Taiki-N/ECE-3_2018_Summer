#include "Controller.h"

// Pins
constexpr unsigned char leftMotorControlPin = 5;
constexpr unsigned char rightMotorControlPin = 3;

constexpr unsigned char photoTranPin1 = A0;		// Right
constexpr unsigned char photoTranPin2 = A1;		// Center
constexpr unsigned char photoTranPin3 = A2;		// Left

constexpr unsigned char rLedPin = 7;
constexpr unsigned char gLedPin = 9;
constexpr unsigned char bLedPin = 11;

// Other constants
constexpr unsigned char leftMotorPwmMax = 230;
constexpr unsigned char rightMotorPwmMax = 255;


double error(unsigned short, unsigned short, unsigned short);


void setup()
{
	// PWM
	pinMode(leftMotorControlPin, OUTPUT);
	pinMode(rightMotorControlPin, OUTPUT);

	// Phototransistors
	pinMode(photoTranPin1, INPUT);
	pinMode(photoTranPin2, INPUT);
	pinMode(photoTranPin3, INPUT);

	// LEDs
	// pinMode(rLedPin, OUTPUT);
	// pinMode(gLedPin, OUTPUT);
	// pinMode(bLedPin, OUTPUT);

	Serial.begin(9600);
}

void loop()
{
	unsigned char leftMotorPwm = leftMotorPwmMax;
	unsigned char rightMotorPwm = rightMotorPwmMax;

	// Sensors
	unsigned short photoTran1 = analogRead(photoTranPin1);
	unsigned short photoTran2 = analogRead(photoTranPin2);
	unsigned short photoTran3 = analogRead(photoTranPin3);


// ------- Debug --------
/*
	Serial.print("PT1: ");
	Serial.println(photoTran1);
	Serial.print("PT2: ");
	Serial.println(photoTran2);
	Serial.print("PT3: ");
	Serial.println(photoTran3);
	Serial.println();
//*/

//*
	unsigned short photoTranTest = analogRead(A5);
	Serial.println(photoTranTest);
//*/
// ----------------------

	delay(1000);

	// PWM Control
	analogWrite(leftMotorControlPin, leftMotorPwm);			// Left motor
	analogWrite(rightMotorControlPin, rightMotorPwm);		// Right motor

	// LED Indicators
	/*
	if (false) {
		digitalWrite(rLedPin, HIGH);
		digitalWrite(gLedPin, LOW);
		digitalWrite(bLedPin, LOW);
	}
	else if (false) {
		digitalWrite(rLedPin, LOW);
		digitalWrite(gLedPin, HIGH);
		digitalWrite(bLedPin, LOW);
	}
	else if (false) {
		digitalWrite(rLedPin, LOW);
		digitalWrite(gLedPin, LOW);
		digitalWrite(bLedPin, HIGH);
	}
	//*/
}

double error(unsigned short, unsigned short, unsigned short)
{
	//
}
