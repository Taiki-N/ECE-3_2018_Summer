#include <math.h>
#include "Controller.h"
#include "Motors.h"

// Pins
constexpr unsigned char leftMotorControlPin = 6;
constexpr unsigned char rightMotorControlPin = 5;

constexpr unsigned char photoTranPinLeft = A2;
constexpr unsigned char photoTranPinCenter = A1;
constexpr unsigned char photoTranPinRight = A0;

constexpr unsigned char rLedPin = 9;		// Red
constexpr unsigned char gLedPin = 8;		// Green
constexpr unsigned char bLedPin = 7;		// Blue

constexpr unsigned char wheelSensor = 2;

constexpr unsigned long wheelSpeedThreshold = 2000;		// [ms]


double error(short, short, short);
double rc(short, short);
double lc(short, short);
void wheelSpeedISR();

Controller steeringController;
Motors mainMotors(leftMotorControlPin, rightMotorControlPin);

bool stop = false;		// At the end of track

int stopTime = 0;

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

	//Serial.begin(9600);

//*
	digitalWrite(rLedPin, LOW);
	digitalWrite(gLedPin, LOW);
	digitalWrite(bLedPin, LOW);
//*/

	stopTime = millis();
}

void loop()
{
	if (true) {
		// Sensors
		short photoTranLeft = analogRead(photoTranPinLeft);
		short photoTranCenter = analogRead(photoTranPinCenter);
		short photoTranRight = analogRead(photoTranPinRight);


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

		//delay(1000);
		//delay(200);

	/*
		analogWrite(leftMotorControlPin, 255);
		analogWrite(rightMotorControlPin, 255);
	//*/
	// ----------------------

	/*
		// Controller
		steeringController.setError(error(photoTranLeft, photoTranCenter, photoTranRight));

		//Serial.println(error(photoTranLeft, photoTranCenter, photoTranRight));

		// Motor Control
		int carPos;
		if (photoTranLeft > 400 && photoTranCenter > 400 && photoTranRight > 400 && millis() - stopTime > 9000) {					// End of track
			mainMotors.zeroPwm();
			carPos = 10;				// Car is at rest
		}
		else {
			carPos = mainMotors.convertControllerOutput(steeringController.getU());
		}
		mainMotors.actuate();

		//Serial.println(static_cast<int>(carPos));

		// Serial.print("L:");
		// Serial.print(static_cast<int>(mainMotors.getPwm(0)));
		// Serial.print("\t\tR:");
		// Serial.println(static_cast<int>(mainMotors.getPwm(1)));

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

				stop = true;

				break;
			}
		}
	//*/

	//*
		mainMotors.setPwm(50,50);
		mainMotors.actuate();

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

			if (LEDCount >= 5) {		// After five flashes
				mainMotors.maxLeft();
				mainMotors.actuate();
				if (millis() - LEDTime > 300)
					LEDCount = 0;
			}
		}
//*/
	}
}


/*	error -- Error function to be fed to the PD controller
	[Arguments]
		- l: Input from phototransistor on the left
		- c: Input from phototransistor in the middle
		- r: Input from phototransistor on the right
	[Return]
		The value of the error function
*/
double error(short l, short c, short r)
{
	double ret = rc(r, c);

/*
	Serial.println(ret);
//*/

	if (ret >= 0) {
		ret = lc(l, c);		// Car is off to the right
	}

	return ret;
}

double rc(short r, short c)
{
	return 0.3 * (-r - 300 / c) + 94.1;
}

double lc(short l, short c)
{
	return 0.125 * (l - c) + 50.84;
}

void wheelSpeedISR()
{
	wheelTime = millis();
}
