#include "Controller.h"

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

constexpr unsigned char leftWheelSensor = 2;
//constexpr unsigned char rightWheelSensor = A6;

// Other constants
constexpr unsigned char leftMotorPwmMax = 230;
constexpr unsigned char rightMotorPwmMax = 255;


double error(unsigned short, unsigned short, unsigned short);


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
	pinMode(leftWheelSensor, INPUT);
	//pinMode(rightWheelSensor, INPUT);

	Serial.begin(9600);

	digitalWrite(rLedPin, HIGH);
	digitalWrite(gLedPin, HIGH);
	digitalWrite(bLedPin, HIGH);
}

void loop()
{
	unsigned char leftMotorPwm = leftMotorPwmMax;
	unsigned char rightMotorPwm = rightMotorPwmMax;

	// Sensors
	unsigned short photoTranLeft = analogRead(photoTranPinLeft);
	unsigned short photoTranCenter = analogRead(photoTranPinCenter);
	unsigned short photoTranRight = analogRead(photoTranPinRight);


// ------- Debug --------
//*
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

	delay(1000);
	//delay(200);

//*
	
//*/
// ----------------------


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

