void save_EEPROM_groundstart(void)
{	
	// Sensor settings
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t *)	EE_WP_INDEX, 	wp_index);					eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_AP_OFFSET, 	airpressure_offset);		eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_IR_MAX, 		ir_max);					eeprom_busy_wait();


	// Radio settings
	eeprom_write_word((uint16_t *)	EE_CH1_MIN, radio_min[CH_STEERING]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH2_MIN, radio_min[CH_THROTTLE_R]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH4_MIN, radio_min[CH_UNUSED]);			eeprom_busy_wait();

	eeprom_write_word((uint16_t *)	EE_CH1_TRIM, radio_trim[CH_STEERING]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH2_TRIM, radio_trim[CH_THROTTLE_R]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH4_TRIM, radio_trim[CH_UNUSED]);		eeprom_busy_wait();

	eeprom_write_word((uint16_t *)	EE_CH1_MAX, radio_max[CH_STEERING]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH2_MAX, radio_max[CH_THROTTLE_R]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH4_MAX, radio_max[CH_UNUSED]);			eeprom_busy_wait();

	eeprom_write_word((uint16_t *)	EE_ELEVON1_TRIM, 	elevon1_trim);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_ELEVON2_TRIM, 	elevon2_trim);
}

void restore_EEPROM(void)
{
	// if this is a first time use
	// check_eeprom_defaults();
	
	// Read out user options
	// ----------------------
	eeprom_busy_wait();
	wp_total 						= eeprom_read_byte((uint8_t *)	EE_WP_TOTAL);	eeprom_busy_wait();	//includes home
	wp_index 						= eeprom_read_byte((uint8_t *)  EE_WP_INDEX); 	eeprom_busy_wait();	// or return current waypoint

	wp_radius 						= eeprom_read_byte((uint8_t *)	EE_WP_RADIUS);	eeprom_busy_wait();
	airpressure_offset 				= eeprom_read_word((uint16_t *)	EE_AP_OFFSET);	eeprom_busy_wait(); 
	ir_max							= eeprom_read_word((uint16_t *)	EE_IR_MAX);		eeprom_busy_wait(); 

	// lets fix broken values
	// ----------------------
	wp_index 						= constrain(wp_index, 0, wp_total);

	wp_radius 						= constrain(wp_radius, 					10, 	50);
	airpressure_offset 				= constrain(airpressure_offset, 		0, 		512);
	ir_max	 						= constrain(ir_max, 					40, 	512);

	radio_min[CH_STEERING] 				= constrain(radio_min[CH_STEERING], 		800, 	2200);
	radio_min[CH_THROTTLE_R] 			= constrain(radio_min[CH_THROTTLE_R], 		800, 	2200);
	radio_min[CH_UNUSED] 			= constrain(radio_min[CH_THROTTLE_R], 		800, 	2200);

	radio_trim[CH_STEERING] 			= constrain(radio_trim[CH_STEERING], 		800, 	2200);
	radio_trim[CH_THROTTLE_R] 			= constrain(radio_trim[CH_THROTTLE_R], 		800, 	2200);
	radio_trim[CH_UNUSED] 			= constrain(radio_trim[CH_UNUSED], 		800, 	2200);
 
 	radio_max[CH_STEERING] 				= constrain(radio_max[CH_STEERING], 		800, 	2200);
	radio_max[CH_THROTTLE_R] 			= constrain(radio_max[CH_THROTTLE_R],	 	800, 	2200);
	radio_max[CH_UNUSED] 			= constrain(radio_max[CH_THROTTLE_R],	 	800, 	2200);
	
	elevon1_trim 					= constrain(elevon1_trim, 				950, 	2050);
	elevon2_trim 					= constrain(elevon2_trim, 				950, 	2050);
	
	// load home latitude, long, alt
	// -----------------------------
	home = get_wp_with_index(0);
	
	// load next WP
	// ------------
	load_waypoint();
}

void check_eeprom_defaults(void)
{
	int test = eeprom_read_word((uint16_t *) EE_CH1_TRIM);	eeprom_busy_wait();
	
	if (test < 100){
		
		eeprom_busy_wait();
		eeprom_write_byte((uint8_t *)	EE_WP_RADIUS, 15);		eeprom_busy_wait();	// default WP radius
		eeprom_write_word((uint16_t *)	EE_AP_OFFSET,  250);	eeprom_busy_wait();	// airpressure_offset
		eeprom_write_word((uint16_t *)	EE_IR_MAX, 200);		eeprom_busy_wait();	// ir_max 

		eeprom_write_word((uint16_t *)	EE_CH1_TRIM, 1500);		eeprom_busy_wait();	// radio_trim[CH_STEERING]
		eeprom_write_word((uint16_t *)	EE_CH2_TRIM, 1500);		eeprom_busy_wait();	// radio_trim[CH_THROTTLE_R]
		eeprom_write_word((uint16_t *)	EE_CH4_TRIM, 1500);		eeprom_busy_wait(); // radio_trim[CH_UNUSED]

	}
}

