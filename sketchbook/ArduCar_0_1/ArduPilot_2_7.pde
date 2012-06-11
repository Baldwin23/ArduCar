// ARDUPILOT 2.7.1
// September 10, 2010 // Hardware Ardupilot (not mega)

#include <avr/io.h>
#include <avr/eeprom.h>
#include <math.h>
#include "defines.h"

#include "AP_Config.h"
#include "AP_PID_settings.h"


// GENERAL VARIABLE DECLARATIONS
// --------------------------------------------
byte 	control_mode		= MANUAL;
boolean failsafe			= false;	// did our throttle dip below the failsafe value?
boolean ch3_failsafe		= false;

//byte 	config_tool_options	= 0;		// a bitmask defining config tool options such as altitude hold
byte 	crash_timer			= 0;
int 	egg_dist 			= 0;
boolean manual_assist = false;
long front_dist = 0;
/* Radio values
		Channel assignments	
			1	
			2	
			3	Throttle
			4	
*/

int16_t radio_min[] 	= {CH1_MIN, CH2_MIN, CH3_MIN, CH4_MIN};	// may be reset by init sequence
int16_t radio_trim[] 	= {0,0,0,0};							// may be reset by init sequence
int16_t radio_max[] 	= {CH1_MAX, CH2_MAX, CH3_MAX, CH4_MAX};	// may be reset by init sequence

int16_t radio_in[]		= {1500,1500,1100,1500};			// current values from the transmitter - microseconds
int16_t radio_out[]		= {1500,1500,1100,1500};			// PWM to ROLL PITCH Servos
float servo_out[] 	= {0,0,0,0};						// current values to the servos - -45 to 45 degrees, except [3] is 0 to 100

int16_t elevon1_trim 	= 1500;
int16_t elevon2_trim 	= 1500;
int16_t ch1_temp 		= 1500;			// Used for elevon mixing
int16_t ch2_temp 		= 1500;
int16_t ch3_raw;

uint16_t timer_ovf_a	= 0;
uint16_t timer_ovf_b	= 0;
uint16_t timer_ovf		= 0;

// for elevons radio_in[CH_STEERING] and radio_in[CH_THROTTLE_R] are equivalent aileron and elevator, not left and right elevon


/*	PID Control variables
		Cases
		1	Aileron servo control	(rudder if no ailerons)
		2	Elevator servo control
		3	Rudder servo control 	(if we have ailerons)
		4	Roll set-point control
		5	Pitch set-point based on airspeed error control		
		6	Pitch set-point based on altitude error control		(if we do not have airspeed sensor)
		7	Throttle based on Energy Height (Total Energy) error control
		8	Throttle based on altitude error control
*/
		
float kp[]={SERVO_ROLL_P, SERVO_PITCH_P, SERVO_RUDDER_P, NAV_ROLL_P, NAV_PITCH_ASP_P, NAV_PITCH_ALT_P, THROTTLE_TE_P, THROTTLE_ALT_P};
float ki[]={SERVO_ROLL_I, SERVO_PITCH_I, SERVO_RUDDER_I, NAV_ROLL_I, NAV_PITCH_ASP_I, NAV_PITCH_ALT_I, THROTTLE_TE_I, THROTTLE_ALT_I};
float kd[]={SERVO_ROLL_D, SERVO_PITCH_D, SERVO_RUDDER_D, NAV_ROLL_D, NAV_PITCH_ASP_D, NAV_PITCH_ALT_D, THROTTLE_TE_D, THROTTLE_ALT_D}; 		

const float integrator_max[] = {SERVO_ROLL_INT_MAX, SERVO_PITCH_INT_MAX, SERVO_RUDDER_INT_MAX, NAV_ROLL_INT_MAX, NAV_PITCH_ASP_INT_MAX, NAV_PITCH_ALT_INT_MAX, THROTTLE_TE_INT_MAX, THROTTLE_ALT_INT_MAX};

float	integrator[] = {0,0,0,0,0,0,0,0};	// PID Integrators
float	last_error[] = {0,0,0,0,0,0,0,0};	// PID last error for derivative
float 	nav_gain_scaler	= 1;			// Gain scaling for headwind/tailwind


// GPS variables
// -------------
byte 	ground_start_count	= 5;			// have we achieved first lock and set Home?
byte 	GPS_fix				= BAD_GPS;		// This variable store the status of the GPS
boolean GPS_light			= false;		// status of the GPS light
boolean GPS_new_data		= false;		// fresh data from the GPS needs to be used
const float t7				= 10000000.0;	// used to scale GPS values for EEPROM storage
long 	iTOW 				= 0; 			// GPS Millisecond Time of Week
byte	NumSats				= 0;			// number of sats
boolean imu_ok 				= false;		// the IMU is sending data correctly
float 	scaleLongUp			= 0;			// used to reverse longtitude scaling
float 	scaleLongDown		= 0;			// used to reverse longtitude scaling


// used to consruct the GPS data from Bytes to ints and longs
// ----------------------------------------------------------
union long_union {
	int32_t dword;
	uint8_t	byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t	byte[2];
} intUnion;

// Location & Navigation 
// ---------------------
byte 	wp_radius			= 20;			// meters
int 	ground_speed 		= 0;			// m/s * 100
long 	ground_course 		= 0;			// deg * 100 dir of plane
long 	hold_course 		= -1;			// deg * 100 dir of plane
long	nav_bearing			= 0;			// deg * 100 : 0 to 360 current desired bearing to navigate
long 	target_bearing		= 0;			// deg * 100 : 0 to 360 location of the plane to the target
long 	crosstrack_bearing	= 0;			// deg * 100 : 0 to 360 desired angle of plane to target
int 	climb_rate 			= 0;			// m/s * 100
byte	loiter_radius 		= LOITER_RADIUS; // meters


// Airspeed
// --------
float 	airspeed 			= 0; 							// m/s * 100
int		airspeed_cruise		= AIRSPEED_CRUISE * 100;		// m/s * 100 : target airspeed sensor value

// Location Errors
// ---------------
long 	bearing_error		= 0; 			// deg * 100 : 18000 to -18000 
long 	altitude_error		= 0;			// meters * 100 we are off in altitude
float 	airspeed_error		= 0;			// m/s * 100: 
float	crosstrack_error	= 0;			// deg * 100 : 18000 to -18000  meters we are off trackline
long 	energy_error		= 0;

// Sensors 
// --------
float 	airpressure_raw		= 511;		// Airspeed Sensor - is a float to better handle filtering
int 	airpressure_offset	= 0;		// analog air pressure sensor while still
int 	airpressure			= 0;		// airspeed as a pressure value
float 	battery_voltage 	= 1024;		// Battery Voltage
float 	filter_batt_voltage = 511; 		// Filtered Battery Voltage mjc

// Sensors -- thermopiles
long analog0				= 511;		// Thermopiles - Pitch
long analog1				= 511;		// Thermopiles - Roll
long analog2				= 511;		// Thermopiles - Z
int ir_max					= 300;		// used to scale Thermopile output to 511
int ir_max_save				= 300;		// used to scale Thermopile output to 511
long roll_sensor			= 0;		// how much we're turning in deg * 100
long pitch_sensor			= 0;		// our angle of attack in deg * 100

// Sensors -- ultrasonic range
long front_range                        = 300; // 300cm (max-distance of this particular usrf)


// Loiter management
// -----------------
long 	old_target_bearing	= 0;			// deg * 100
int		loiter_total 		= 0;			// deg : how many times to loiter * 360
int 	loiter_delta		= 0;			// deg : how far we just turned
int		loiter_sum			= 0;			// deg : how far we have turned around a waypoint


// these are the values for navigation control functions
// ----------------------------------------------------
long 	nav_roll			= 0;			// deg * 100 : target roll angle
long 	nav_pitch			= 0;			// deg * 100 : target pitch angle
int 	throttle_cruise		= THROTTLE_CRUISE;	// 0-100 : target throttle output for average speed
long	altitude_estimate	= 0;			// for smoothing GPS output


// Waypoints
// ---------
long 	wp_distance				= 0;		// meters - distance between plane and next waypoint
long 	wp_totalDistance		        = 0;		// meters - distance between old and next waypoint
byte 	wp_total				= 0;		// # of waypoints
byte 	wp_index				= 0;		// Current WP index, -1 is RTL

// 3D Location vectors
// -------------------
struct Location {
	long lat;				// Lattitude * 10**7
	long lng;				// Longitude * 10**7
	long alt;				// Altitude in centimeters (meters * 100)
};

struct Location home 			= {0,0,0};	// home location
struct Location prev_WP 		= {0,0,0};	// last waypoint
struct Location current_loc		= {0,0,0};	// current location
struct Location next_WP 		= {0,0,0};	// next waypoint
struct Location new_WP                  = {0,0,0};
long target_altitude			= 0;		// used for 
long offset_altitude			= 0;		// used for 

// IMU specific
// ------------
#if GPS_PROTOCOL == 3
	long perf_mon_timer = 0;
	byte imu_health = 0;
	int G_Dt_max = 0.0;						// Max main loop cycle time in milliseconds
	byte gyro_sat_count = 0;
	byte adc_constraints = 0;
	byte renorm_sqrt_count = 0;
	byte renorm_blowup_count = 0;
	byte gps_payload_error_count = 0;
	byte gps_checksum_error_count = 0;
	byte gps_pos_fix_count = 0;
	byte gps_nav_fix_count = 0;
	byte gps_messages_sent = 0;
	byte gps_messages_received = 0;
	int imu_messages_received = 0;
	byte imu_payload_error_count = 0;
	byte imu_checksum_error_count = 0;
#endif

// System Timers
// --------------
unsigned long fast_loopTimer		= 0;		// Time in miliseconds of main control loop
unsigned long medium_loopTimer		= 0;		// Time in miliseconds of navigation control loop
byte medium_loopCounter				= 0;		// Counters for branching from main control loop to slower loops
byte slow_loopCounter				= 0;		// 
unsigned long deltaMiliSeconds 		= 0;		// Delta Time in miliseconds
unsigned long dTnav					= 0;		// Delta Time in milliseconds for navigation computations
int mainLoop_count 					= 0;
unsigned long elapsedTime			= 0;		// for doing custom events

// Basic Initialization
//---------------------
void setup() {
	init_ardupilot();
}

void loop()
{
	// We want this to execute at 50Hz if possible
	// -------------------------------------------
	if (DIYmillis()-fast_loopTimer > 19) {
		deltaMiliSeconds 	= DIYmillis() - fast_loopTimer;
		fast_loopTimer		= DIYmillis();

		// Execute the fast loop
		// ---------------------
		//PORTD |= B10000000;
		fast_loop();
		//PORTD &= B01111111;
		
		// Execute the medium loop 
		// -----------------------
		//PORTD |= B01000000;
		medium_loop();
		//PORTD &= B10111111;

		//mainLoop_event();
	}
}



void fast_loop()
{
	// Read radio
	// ----------
	read_radio();
	update_current_flight_mode();
	set_servos_4();
}

void medium_loop()
{
	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch (medium_loopCounter){
	               
		// This case deals with the GPS
		//-------------------------------
		case 0:
			medium_loopCounter++;
			// update navigation timer
			// -----------------------			
			dTnav = DIYmillis() - medium_loopTimer;
			medium_loopTimer = DIYmillis();

			update_GPS();
			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
			medium_loopCounter++;

			// calculate the plane's desired bearing
			// -------------------------------------
			navigate();
			break;

		// unused
		//------------------------------
		case 2:
			medium_loopCounter++;
                        // This might be a good place to check our front range
                        front_range = getFrontDistanceCM();
                        Serial.println(front_range);

			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;
			send_message(MSG_ATTITUDE);
			break;
			
		// This case controls the slow loop
		//---------------------------------
		case 4:
			medium_loopCounter=0;
			//mediumLoop_event();
			slow_loop();
			break;	
	}
}

void slow_loop()
{
	// This is the slow (3 1/3 Hz) loop pieces
	//----------------------------------------
	switch (slow_loopCounter){
		case 0:
			slow_loopCounter++;
			// nothing here
			break;
			
		case 1:
			slow_loopCounter++;

			// Read 3-position switch on radio
			// -------------------------------
			read_control_switch();

			// Read main battery voltage if hooked up - does not read the 5v from radio
			// ------------------------------------------------------------------------
			#if BATTERY_EVENT == 1
				read_battery();
			#endif
			
			break;
			
		case 2:
			slow_loopCounter = 0;
			// Reserved
			// Probably put uplink here
			break;
	}
}


void update_GPS(void)
{
	#if GPS_PROTOCOL != 3
	decode_gps();
	#endif

	if (GPS_new_data){
		//print_position();
		send_message(MSG_LOCATION);

		if(ground_start_count == 1){
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0){
				Serial.println("!! bad loc");
				ground_start_count = 5;
				
			}else{
				init_home();
				ground_start_count = 0;
			}
			
		}else if(ground_start_count > 1){
			ground_start_count--;
		}
				
		// save performance data to the EEPROM (WHY?? -- this is a waste of eeprom)
                // TODO
		// -------------------------------------
		//set_max_altitude_speed();
	}
}

void update_current_flight_mode(void)
{
	switch(control_mode){
		case AUTO:
			calc_throttle();
			break;
		
		case MANUAL:
			servo_out[CH_STEERING]  = ((radio_in[CH_STEERING]  - radio_trim[CH_STEERING]) * 45  * REVERSE_ROLL) / 500;
			//servo_out[CH_THROTTLE_R] = ((radio_in[CH_THROTTLE_R] - radio_trim[CH_THROTTLE_R]) * 45 * REVERSE_PITCH) / 500;
			break;
	}
}

void update_navigation()
{
	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	// ------------------------------------------------------------------------

	// distance and bearing calcs only
	
	switch(control_mode){
	case AUTO:
			// reset hold course
			hold_course == -1;
			
			// check if we have missed the WP
			loiter_delta = (target_bearing - old_target_bearing)/100;
			
			// reset the old value
			old_target_bearing = target_bearing;
			
			// wrap values
			if (loiter_delta > 170) loiter_delta -= 360;
			if (loiter_delta < -170) loiter_delta += 360;
			loiter_sum += loiter_delta;
	
			if (wp_distance <= 0){
				// do nothing
				
			}else if (wp_distance <= wp_radius){
//				waypoint_event(EVENT_WILL_REACH_WAYPOINT);
				reached_waypoint();
				
			}else{
				update_crosstrack();
				if(loiter_sum > 300){
					Serial.println("MSG missed WP");
					//waypoint_event(EVENT_WILL_REACH_WAYPOINT);
					reached_waypoint();
				}else if (loiter_sum < -300){
					Serial.println("MSG missed WP");
					//waypoint_event(EVENT_WILL_REACH_WAYPOINT);
					reached_waypoint();
				}
			}
			break;
				
	}

	// Wrap the values
	// ---------------
	wrap_bearing();
}
