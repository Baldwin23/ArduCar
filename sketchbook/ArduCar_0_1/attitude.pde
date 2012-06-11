void stabilize()
{
    // no more stablize mode
}

void crash_checker()
{
    // stub
}

void calc_throttle()
{
        Serial.print("FRONT: ");
        Serial.print(front_range);
        Serial.println();
        if(front_range > 100)
        {
            servo_out[CH_THROTTLE] = THROTTLE_CRUISE;
        }
        else
        {
            servo_out[CH_THROTTLE] = THROTTLE_MIN;
        }
}

void calc_nav_pitch()
{
    //stub
}
void calc_nav_roll()
{
    //stub
}


/*****************************************
 * Roll servo slew limit
 *****************************************/
float roll_slew_limit(float servo)
{
	static float last;
	float temp = constrain(servo, last-ROLL_SLEW_LIMIT * deltaMiliSeconds/1000.f, last + ROLL_SLEW_LIMIT * deltaMiliSeconds/1000.f);
	last = servo;
	return temp;
}

/*****************************************
 * Throttle slew limit
 *****************************************/
float throttle_slew_limit(float throttle)
{
        static float last;
	float temp = constrain(throttle, last-THROTTLE_SLEW_LIMIT * deltaMiliSeconds/1000.f, last + THROTTLE_SLEW_LIMIT * deltaMiliSeconds/1000.f);
	last = throttle;
	return temp;
}


/*****************************************
 * Proportional Integrator Derivative Control 
 *****************************************/

float PID(long PID_error, long dt, int PID_case)
{

	// PID_case is used to keep track of which PID controller is being used - e.g.	PID_servo_out[CH_STEERING]
	float output;
 
	float derivative = 1000.0f * (PID_error-last_error[PID_case]) / (float)dt;
	last_error[PID_case] = PID_error;
	output = (kp[PID_case]*PID_error);    	//Compute proportional component
											//Positive error produces positive output

	integrator[PID_case] += (float)PID_error*(float)dt/1000.0f; 
	integrator[PID_case] = constrain(integrator[PID_case],-integrator_max[PID_case],integrator_max[PID_case]);
  
	//integrator[PID_case] = reset_I(integrator[PID_case]);   
  
	output += integrator[PID_case]*ki[PID_case];        	//Add the integral component

	output += derivative*kd[PID_case];                   	//Add the derivative component

	return output; //Returns the result     

}

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void reset_I(void)
{
		integrator[CASE_NAV_ROLL] = 0;
		integrator[CASE_NAV_PITCH_ASP] = 0;
		integrator[CASE_NAV_PITCH_ALT] = 0;
		integrator[CASE_TE_THROTTLE] = 0;
		integrator[CASE_ALT_THROTTLE] = 0;

		last_error[CASE_NAV_ROLL] = 0;
		last_error[CASE_NAV_PITCH_ASP] = 0;
		last_error[CASE_NAV_PITCH_ALT] = 0;
		last_error[CASE_TE_THROTTLE] = 0;
		last_error[CASE_ALT_THROTTLE] = 0;
}

