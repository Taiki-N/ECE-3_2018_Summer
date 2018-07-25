// Pins
constexpr unsigned char leftMotorControlPin = 3;
constexpr unsigned char rightMotorControlPin = 5;

constexpr unsigned char photoTranPin1 = A0;
constexpr unsigned char photoTranPin2 = A1;
constexpr unsigned char photoTranPin3 = A2;
constexpr unsigned char photoTranPin4 = A3;

constexpr unsigned char rLedPin = 7;
constexpr unsigned char gLedPin = 9;
constexpr unsigned char bLedPin = 11;


void setup()
{
	// PWM
	pinMode(leftMotorControlPin, OUTPUT);
	pinMode(rightMotorControlPin, OUTPUT);

	// Phototransistors
	pinMode(photoTranPin1, INPUT);

	// LEDs
	// pinMode(rLedPin, OUTPUT);
	// pinMode(gLedPin, OUTPUT);
	// pinMode(bLedPin, OUTPUT);

	Serial.begin(9600);
}

void loop()
{
	unsigned char leftMotorPwm = 230;
	unsigned char rightMotorPwm = 255;

	// Sensors
	short photoTran1 = analogRead(photoTranPin1);
	Serial.println(photoTran1);

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
