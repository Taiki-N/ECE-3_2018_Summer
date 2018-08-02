#ifndef CONTROLLER_H
#define CONTROLLER_H

constexpr double kp = 0.01;			// Proportional constant
constexpr double kd = 0;		// Derivative constant


class Controller
{
private:
	double error;
	double old_error;
	unsigned long time;		// ms
	unsigned long dt;		// ms

	double derivative() const;

public:
	Controller();
	void setError(double);
	double getU() const;
};


/*	Controller -- Constructor
*/
Controller::Controller() :
error(-1),						// old_error is set -1 when an error is first input
time(0)
{
}

/*	setError -- Accepts a new return value of an error function, pushes a previous error
		out to a separate variable for derivatives. Keeps track of the time differential.
	[Argument]
		- e: Return value of an error function
*/
void Controller::setError(double e)
{
	unsigned long newTime = millis();	// ms
	dt = newTime - time;
	time = newTime;

	old_error = error;
	error = e;
}

/*	getU -- Gets a controller output
*/
double Controller::getU() const
{
	return kp * error + kd * derivative();
}

/*	derivative -- Takes the time derivative of the error
*/
double Controller::derivative() const
{
	return (error - old_error) / dt;	// -/ms
}

#endif
