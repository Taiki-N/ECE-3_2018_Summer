void setup()
{
	pinMode(3, OUTPUT);
	pinMode(5, OUTPUT);
}

void loop()
{
	unsigned char leftMotorPwm = 230;
	unsigned char rightMotorPwm = 255;

	analogWrite(3, leftMotorPwm);		// Left motor
	analogWrite(5, rightMotorPwm);		// Right motor
}

