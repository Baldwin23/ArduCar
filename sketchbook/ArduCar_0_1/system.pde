/*****************************************************************************
The init_ardupilot function processes everything we need for an in-air restart
	We will determine later if we are actually on the ground and process a 
	ground start in that case.
	
Also in this function we will dump the log if applicable based on the slide switch
*****************************************************************************/
void init_ardupilot()
{

	#if GPS_PROTOCOL == -1
		Serial.begin(NO_GPS);
		Serial.println("NO GPS");
	#elif GPS_PROTOCOL == 0
		Serial.begin(NMEA_GPS);
		Serial.println("GPS: NMEA");
	#elif GPS_PROTOCOL == 1
		Serial.begin(EM406_GPS);
		Serial.println("GPS: EM406");
	#elif GPS_PROTOCOL == 3
		Serial.begin(ARDU_IMU);  
		Serial.println("ARDU IMU");
	#elif GPS_PROTOCOL == 4
		Serial.begin(MTK_GPS);  
		Serial.println("MediaTek GPS");
	#endif

	Serial.println();
 	Serial.println("Init ArduCar 0.0.1");
	
 	// ATMEGA ADC
 	// PC0 - ADC0 	- 23 - X sensor
 	// PC1 - ADC1 	- 24 - Y sensor
 	// PC2 - ADC2 	- 25 - Z sensor
 	// PC3 - ADC3 	- 26 - Pressure sensor
 	// PC4 - ADC4 	- 27 - 
 	// PC5 - ADC5 	- 28 - Battery Voltage
 	
 	// ATMEGA
	// PORTD
	// p0				// PD0 - RXD  		- Serial RX 	
	// p1				// PD1 - TXD  		- Serial TX 
	pinMode(2,INPUT);	// PD2 - INT0 		- Rudder in							- INPUT Rudder/Aileron
	pinMode(3,INPUT);	// PD3 - INT1 		- Elevator in 						- INPUT Elevator
	pinMode(4,INPUT);	// PD4 - XCK/T0 	- MUX pin							- Connected to Pin 2 on ATtiny
	pinMode(5,INPUT);	// PD5 - T0			- Mode pin							- Connected to Pin 6 on ATtiny   - Select on MUX
	pinMode(6,OUTPUT);	// PD6 - T1			- Ground start signaling Pin	
	pinMode(7,OUTPUT);	// PD7 - AIN0		- GPS Mux pin 
	// PORTB
	pinMode(8, OUTPUT); // PB0 - AIN1		- Servo throttle					- OUTPUT THROTTLE
	pinMode(9, OUTPUT);	// PB1 - OC1A		- Elevator PWM out					- Elevator PWM out
	pinMode(10,OUTPUT);	// PB2 - OC1B		- Rudder PWM out					- Aileron PWM out
	pinMode(11,INPUT); 	// PB3 - MOSI/OC2	-  

        //
	pinMode(12,OUTPUT);     // FRONT RANGE (replaces GPS lock blue LED)
	pinMode(13,INPUT); 	// PB5 - SCK		- Yellow LED pin   					- INPUT Throttle

	// PORTC - Analog ports
	// PC0 - Thermopile - x
	// PC1 - Thermopile - y 
	// PC2 - Thermopile - z
	// PC3 - Airspeed
	// PC4 - CH4 OUT - Rudder output
	// PC5 - Battery

	// set Analog out 4 to output
	DDRC |= B00010000;
	

	digitalWrite(6,HIGH);
	
	// Enable GPS
	// ----------------
	setGPSMux();

	// setup control switch
	// ----------------
	initControlSwitch();
		
	// load launch settings from EEPROM
	// --------------------------------
	restore_EEPROM();

	// connect to radio 
	// ----------------
	init_radio();

	// setup PWM timers
	// ----------------
	init_PWM();

	// print the radio 
	// ---------------
	print_radio();
				
	// set the correct flight mode
	// ---------------------------
	reset_control_switch();

	Serial.print("freeRAM: ");
	Serial.println(freeRAM(),DEC);
	
	if(startup_check()){
		Serial.println("MSG [initialization'");
		startup_ground();
                load_waypoint();
        }

}

byte startup_check(void){
	if(DEBUG_SUBSYSTEM > 0){
		debug_subsystem();
	}else{
		if ((readSwitch() == 1) && (radio_in[CH_THROTTLE] < 1200)){
			// we are in manual
			return 1;
		}else{
			return 0;
		}
	}
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void startup_ground(void)
{
	// Configure GPS
	// -------------
	init_gps();

	// Output waypoints for confirmation
	// --------------------------------
	print_waypoints();

	//Signal the IMU to perform ground start
	//------------------------
	digitalWrite(6,LOW);
        
	// Makes the servos wiggle
	// step 1 = 1 wiggle
	// -----------------------
	demo_servos();

	// set a default reasonable ir_max val
	// -----------------------------------
	ir_max = 150;
		
	// read the radio to set trims
	// ---------------------------
	trim_radio();
	
	#if SET_RADIO_LIMITS == 1
	read_radio_limits();
	#endif

	// Number of reads before saving Home position
	// -------------------------------------------
	ground_start_count = 6;

	// Save the settings for in-air restart
	// ------------------------------------
	save_EEPROM_groundstart();

	// Lower signal pin in case of IMU power glitch
	// --------------------------------------------
	digitalWrite(6,HIGH);
	
	Serial.println(" ");
	Serial.println("MSG Ready to FLY. ");
}


void set_mode(byte mode)
{
	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}
	#if AUTO_TRIM == 1
		if(control_mode == MANUAL) 
			trim_control_surfaces();
	#endif
	
	control_mode = mode;

	switch(control_mode)
	{
		case MANUAL:
			break;

		case AUTO:
			load_waypoint();
			break;
	}
	
	// output control mode to the ground station
	send_message(MSG_HEARTBEAT);
}

void set_failsafe(boolean mode)
{
	// only act on changes
	// -------------------
	if(failsafe != mode){

		// store the value so we don't trip the gate twice
		// -----------------------------------------------
		failsafe = mode;

		if (failsafe == false){
			// We're back in radio contact
			// ---------------------------

			// re-read the switch so we can return to our preferred mode
			reset_control_switch();
			
			// Reset control integrators
			// ---------------------
			reset_I();

			// Release hardware MUX just in case it's not set
			// ----------------------------------------------
			set_servo_mux(false);
			
		}else{
			// We've lost radio contact
			// ------------------------
			// nothing to do right now
		}
		
            // FAILSAFE WAS CALLED HERE
	}
}



// This hack is to control the V2 shield so we can read the serial from 
// the XBEE radios - which is not implemented yet
void setGPSMux(void)
{
	#if SHIELD_VERSION < 1 || GPS_PROTOCOL == 3 // GPS_PROTOCOL == 3 -> With IMU always go low.
		digitalWrite(7, LOW); //Remove Before Fly Pull Up resistor
    #else
		digitalWrite(7, HIGH); //Remove Before Fly Pull Up resistor
	#endif
}


void setCommandMux(void)
{
	#if SHIELD_VERSION < 1
		digitalWrite(7, HIGH); //Remove Before Fly Pull Up resistor
    #else
		digitalWrite(7, LOW); //Remove Before Fly Pull Up resistor
	#endif
}

void update_GPS_light(void)
{
  // Replace with digital-numerical readout
}

/* This function gets the current value of the heap and stack pointers.
* The stack pointer starts at the top of RAM and grows downwards. The heap pointer
* starts just above the static variables etc. and grows upwards. SP should always
* be larger than HP or you'll be in big trouble! The smaller the gap, the more
* careful you need to be. Julian Gall 6-Feb-2009.
*/
unsigned long freeRAM() {
	uint8_t * heapptr, * stackptr;
	stackptr = (uint8_t *)malloc(4); // use stackptr temporarily
	heapptr = stackptr; // save value of heap pointer
	free(stackptr); // free up the memory again (sets stackptr to 0)
	stackptr = (uint8_t *)(SP); // save value of stack pointer
	return stackptr - heapptr;
}


//***********************************************************************************
//  The following functions are used during startup and are not for telemetry
//***********************************************************************************

void print_radio()
{
	#if GCS_PROTOCOL == 3
	Serial.print("MSG ");
	#endif
	Serial.print("Radio in A: ");
	Serial.print(radio_in[CH_STEERING],DEC);
	Serial.print("\tE: ");
	Serial.print(radio_in[CH_THROTTLE_R],DEC);
	Serial.print("\tT :");
	Serial.print(radio_in[CH_THROTTLE],DEC);
	Serial.print("\tR :");
	Serial.println(radio_in[CH_UNUSED],DEC);
}

void print_waypoints(){
	#if GCS_PROTOCOL == 3
	Serial.print("MSG ");
	#endif

	Serial.print("WP Total: ");
	Serial.println(wp_total, DEC);
	// create a location struct to hold the temp Waypoints for printing
	//Location tmp;
	struct Location tmp = get_wp_with_index(0);
	
	#if GCS_PROTOCOL == 3
	Serial.print("MSG ");
	#endif
	Serial.print("home: \t");
	Serial.print(tmp.lat, DEC);
	Serial.print("\t");
	Serial.print(tmp.lng, DEC);
	Serial.print("\t");
	Serial.println(tmp.alt,DEC);	

	for (int i = 1; i <= wp_total; i++){
		tmp = get_wp_with_index(i);
		#if GCS_PROTOCOL == 3
		Serial.print("MSG ");
		#endif
		Serial.print("wp #");
		Serial.print(i);
		Serial.print("\t");
		Serial.print(tmp.lat, DEC);
		Serial.print("\t");
		Serial.print(tmp.lng, DEC);
		Serial.print("\t");
		Serial.println(tmp.alt,DEC);
	}
}
