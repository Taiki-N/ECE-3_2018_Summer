// Pins
constexpr unsigned char leftMotorControlPin = 3;
constexpr unsigned char rightMotorControlPin = 5;

constexpr unsigned char photoTranPin1 = A0;
constexpr unsigned char photoTranPin2 = A1;
constexpr unsigned char photoTranPin3 = A2;
constexpr unsigned char photoTranPin4 = A3;


void setup()
{
	pinMode(leftMotorControlPin, OUTPUT);
	pinMode(rightMotorControlPin, OUTPUT);

	pinMode(photoTranPin1, INPUT);

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
}
