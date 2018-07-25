#ifndef CONTROLLER_H
#define CONTROLLER_H

constexpr double kp = 0;		// Proportional constant
constexpr double kd = 0;		// Derivative constant


class Controller
{
private:
	double error;
	double old_error;
	unsigned long time;		// ms
	unsigned long dt;		// ms

	double derivative() const;	// Finds the time derivative of the error

public:
	Controller();
	void setError(double);
	double getU() const;			// Returns the value of the control function
};

Controller::Controller() :
error(-1),						// old_error is set -1 when an error is first input
time(0)
{
}

void Controller::setError(double e)
{
	unsigned long newTime = millis();	// ms
	dt = newTime - time;
	time = newTime;

	old_error = error;
	error = e;
}

double Controller::getU() const
{
	return kp * error + kd * derivative();
}

double Controller::derivative() const
{
	return (error - old_error) / dt;	// -/ms
}

#endif
