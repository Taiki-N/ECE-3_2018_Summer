#include <math.h>
#include "Controller.h"
#include "Motors.h"

// Pins
constexpr unsigned char leftMotorControlPin = 5;
constexpr unsigned char rightMotorControlPin = 6;

constexpr unsigned char photoTranPinLeft = A0;
constexpr unsigned char photoTranPinCenter = A1;
constexpr unsigned char photoTranPinRight = A2;

constexpr unsigned char rLedPin = 9;		// Red
constexpr unsigned char gLedPin = 8;		// Green
constexpr unsigned char bLedPin = 7;		// Blue

constexpr unsigned char leftWheelSensor = 2;
//constexpr unsigned char rightWheelSensor = 3;


constexpr unsigned long wheelSpeedThreshold = 1000;		// [ms]


double error(unsigned short, unsigned short, unsigned short);
void leftWheelSpeedISR();
void rightWheelSpeedISR();


Controller steeringController;
Motors mainMotors(leftMotorControlPin, rightMotorControlPin);

unsigned long leftWheelTime[2] {0};
unsigned long rightWheelTime[2] {0};


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

	//attachInterrupt(digitalPinToInterrupt(leftWheelSensor), leftWheelSpeedISR, FALLING);
	//attachInterrupt(digitalPinToInterrupt(rightWheelSensor), rightWheelSpeedISR, FALLING);

	Serial.begin(9600);

/*
	digitalWrite(rLedPin, HIGH);
	digitalWrite(gLedPin, HIGH);
	digitalWrite(bLedPin, HIGH);
//*/

	digitalWrite(rLedPin, LOW);
	digitalWrite(gLedPin, LOW);
	digitalWrite(bLedPin, LOW);
}

void loop()
{
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

	//delay(1000);
	//delay(200);

/*
	analogWrite(leftMotorControlPin, 250);
	analogWrite(rightMotorControlPin, 255);
//*/
// ----------------------

	// Controller
	steeringController.setError(error(photoTranLeft, photoTranCenter, photoTranRight));

//*
	// Motor Control
	char carPos;
	if (false) {					// End of track
		mainMotors.zeroPwm();
		carPos = 10;				// Car is at rest
	}
	else {
		//mainMotors.beginCalibMode();
		carPos = static_cast<char>(mainMotors.convertControllerOutput(steeringController.getU()));
	}
	mainMotors.actuate();

	//Serial.println(static_cast<int>(carPos));

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
		case 10: {		// Car is at rest at the end of track -> red and blue LED
			digitalWrite(rLedPin, HIGH);
			digitalWrite(gLedPin, LOW);
			digitalWrite(bLedPin, HIGH);
			break;
		}
	}
//*/

/*
	// Wheel Speed Sensing System
	if (leftWheelTime[1] - leftWheelTime[0] > wheelSpeedThreshold) {
		for (int i = 0; i < 5; ++i) {
			digitalWrite(rLedPin, HIGH);
			digitalWrite(gLedPin, HIGH);
			digitalWrite(bLedPin, HIGH);

			delay(100);

			digitalWrite(rLedPin, LOW);
			digitalWrite(gLedPin, LOW);
			digitalWrite(bLedPin, LOW);

			delay(100);
		}

		mainMotors.maxLeft();
		mainMotors.actuate();
		delay(50);
	}
	if (rightWheelTime[1] - rightWheelTime[0] > wheelSpeedThreshold) {
		for (int i = 0; i < 5; ++i) {
			digitalWrite(rLedPin, HIGH);
			digitalWrite(gLedPin, HIGH);
			digitalWrite(bLedPin, HIGH);

			delay(100);

			digitalWrite(rLedPin, LOW);
			digitalWrite(gLedPin, LOW);
			digitalWrite(bLedPin, LOW);

			delay(100);
		}

		mainMotors.maxRight();
		mainMotors.actuate();
		delay(50);
	}
//*/
}


/*	error -- Error function to be fed to the PD controller
	[Arguments]
		- l: Input from phototransistor on the left
		- c: Input from phototransistor in the middle
		- r: Input from phototransistor on the right
	[Return]
		The value of the error function
*/
double error(unsigned short l, unsigned short c, unsigned short r)
{
	return static_cast<double>(10 * ((l - r) / log(c) + 20));
}

void leftWheelSpeedISR()
{
	leftWheelTime[0] = leftWheelTime[1];
	leftWheelTime[1] = millis();
}

void rightWheelSpeedISR()
{
	rightWheelTime[0] = rightWheelTime[1];
	rightWheelTime[1] = millis();
}
