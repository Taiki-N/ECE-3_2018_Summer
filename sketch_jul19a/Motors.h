#ifndef MOTORS_H
#define MOTORS_H

constexpr unsigned char leftPwmMax = 255;		// 6V
constexpr unsigned char rightPwmMax = 230;		// 6V

//constexpr unsigned char leftPwmMax = 95;		// 9V
//constexpr unsigned char rightPwmMax = 100;		// 9V

constexpr unsigned char straightCondition = 0;


class Motors
{
private:
	const unsigned char leftPin;
	const unsigned char rightPin;

	unsigned char leftPwm;
	unsigned char rightPwm;

	bool calibMode;

public:
	Motors(unsigned char, unsigned char, unsigned char = leftPwmMax, unsigned char = rightPwmMax);
	unsigned char getPwm(unsigned char) const;
	int convertControllerOutput(double);
	void actuate() const;
	void zeroPwm();
	void maxLeft();
	void maxRight();
	void beginCalibMode();
	void endCalibMode();
};


/*	Motors -- Constructor
	[Arguments]
		- lp: Arduino pin number for left motor
		- rp: Arduino pin number for right motor
		- lpwm0: Initial pwm for left motor
		- rpwm0: initial pwm for right motor
*/
Motors::Motors(unsigned char lp, unsigned char rp, unsigned char lpwm0, unsigned char rpwm0)
: leftPin(lp), rightPin(rp), leftPwm(lpwm0), rightPwm(rpwm0), calibMode(false)
{
}

/*	getPwm -- Returns current PWM values (0-255).
	[Argument]
		- side: Specifying the motor (0 for left and 1 for right)
*/
unsigned char Motors::getPwm(unsigned char side) const
{
	switch (side) {
		case 0: return leftPwm;
		case 1: return rightPwm;
	}
}

/*	convertControllerOutput -- Takes the output from the controller and converts it to
		left and right PWMs.
	[Argument]
		- u: Return value from the controller
	[Return]
		- -1 if car is off-track to the left
		- 0 if car is centered
		- 1 if car is off-track to the right
*/
int Motors::convertControllerOutput(double u)
{
	int ret;			// Return value

/*
	Serial.println(u);
//*/

	if (u < -straightCondition) {		// Car is off-track to the left
		leftPwm = leftPwmMax;
		int right = rightPwmMax + u;		// u is negative

		if (right < 0) {
			right = 0;
		}

		rightPwm = right;

		ret = -1;
	}
	else if (u > straightCondition) {	// Car is off-track to the right
		rightPwm = rightPwmMax;
		int left = leftPwmMax - u;		// u is positive

		if (left < 0) {
			left = 0;
		}

		leftPwm = left;

		ret = 1;
	}
	else {				// Car is at the center
		rightPwm = rightPwmMax;
		leftPwm = leftPwmMax;

		ret = 0;
	}

	return ret;
}

/*	actuate -- Outputs set PWMs to motors through Arduino.
*/
void Motors::actuate() const
{
	if (!calibMode) {
		analogWrite(leftPin, leftPwm);			// Left motor
		analogWrite(rightPin, rightPwm);		// Right motor
	}
	else {		// Calibration mode
		analogWrite(leftPin, leftPwmMax);			// Left motor
		analogWrite(rightPin, rightPwmMax);			// Right motor
	}
}

/* zeroPwm -- Sets PWMs to zero, which will not be effective until actuate() is called.
*/
void Motors::zeroPwm()
{
	leftPwm = rightPwm = 0;
}

void Motors::maxLeft()
{
	leftPwm = leftPwmMax;
}

void Motors::maxRight()
{
	rightPwm = rightPwmMax;
}

/*	beginCalibMode -- Begins calibration mode (= PWMs are fixed to maximum to calibrate
		appropriate maximum PWMs)
*/
void Motors::beginCalibMode()
{
	calibMode = true;
}

/* endCalibMode -- Ends calibration mode
*/
void Motors::endCalibMode()
{
	calibMode = false;
}

#endif
