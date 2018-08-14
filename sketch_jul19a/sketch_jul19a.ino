#include <math.h>
#include "Controller.h"
#include "Motors.h"

// Pins
constexpr unsigned char leftMotorControlPin = 6;
constexpr unsigned char rightMotorControlPin = 5;

constexpr unsigned char photoTranPinLeft = A0;
constexpr unsigned char photoTranPinCenter = A3;
constexpr unsigned char photoTranPinRight = A2;

constexpr unsigned char rLedPin = 9;		// Red
constexpr unsigned char gLedPin = 8;		// Green
constexpr unsigned char bLedPin = 7;		// Blue

constexpr unsigned char wheelSensor = 2;

constexpr unsigned long wheelSpeedThreshold = 2000;		// [ms]


double error(unsigned short, unsigned short, unsigned short);
double rc(unsigned short, unsigned short);
double lc(unsigned short, unsigned short);
void wheelSpeedISR();

Controller steeringController;
Motors mainMotors(leftMotorControlPin, rightMotorControlPin);

unsigned long wheelTime = 0;		// ms
unsigned long LEDTime = 0;			// ms
unsigned char LEDCount = 0;

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
	pinMode(wheelSensor, INPUT);

	attachInterrupt(digitalPinToInterrupt(wheelSensor), wheelSpeedISR, FALLING);

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

	//delay(1000);
	//delay(200);

/*
	analogWrite(leftMotorControlPin, 75);
	analogWrite(rightMotorControlPin, 80);
//*/
// ----------------------

//*
	// Controller
	steeringController.setError(error(photoTranLeft, photoTranCenter, photoTranRight));

	//Serial.println(error(photoTranLeft, photoTranCenter, photoTranRight));

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
/*
	Serial.print("L:");
	Serial.println(static_cast<int>(mainMotors.getPwm(0)));
	Serial.print("R:");
	Serial.println(static_cast<int>(mainMotors.getPwm(1)));
//*/
	delay(1);

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
	if (millis() - wheelTime > wheelSpeedThreshold) {
		digitalWrite(rLedPin, HIGH);
		digitalWrite(gLedPin, HIGH);
		digitalWrite(bLedPin, HIGH);

		if (millis() - LEDTime > 100) {			// 100 ms delay
			digitalWrite(rLedPin, LOW);
			digitalWrite(gLedPin, LOW);
			digitalWrite(bLedPin, LOW);
		}

		if (millis() - LEDTime > 200) {			// 100 ms more delay
			++LEDCount;
			LEDTime = millis();
		}

		if (LEDCount == 5) {		// After five flashes
			mainMotors.maxLeft();
			mainMotors.actuate();
			LEDCount = 0;
		}
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
	if (l > r) {		// Car is off to the right
		return 100 * (lc(l, c) + 0.05);
	}
	else {				// Car is off to the left
		return 100 * (rc(r, c) - 0.0975) - 2.3;
	}
}

double rc(unsigned short r, unsigned short c)
{
	return 0.75 * (300 / r - 300 / (1.5 * c + 100));
}

double lc(unsigned short l, unsigned short c)
{
	return 300 / (1.2 * c + 100) - 300 / l;
}

void wheelSpeedISR()
{
	wheelTime = millis();
}
