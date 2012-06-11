// GPS baud rates
// --------------
#define NO_GPS		38400
#define EM406_GPS	57600
#define ARDU_IMU	38400

// Radio channels -- from 0!
// Channels:
// 0: Steering -- Left/Right
// 1: Throttle -- Forward
// 2: Throttle -- Reverse
// NOTE:  We've separated throttle into seperate channels for the benefit of the receiver
#define CH_STEERING 0
#define CH_THROTTLE_R 1
#define CH_THROTTLE 2
#define CH_UNUSED 3

#define WP_START_BYTE 0x18		// where in memory home WP is stored + all other WP
#define WP_SIZE 10
#define IR_MAX_FIX .88

// PID enumeration
// ---------------
#define CASE_SERVO_ROLL 0
#define CASE_SERVO_PITCH 1
#define CASE_SERVO_RUDDER 2
#define CASE_NAV_ROLL 3
#define CASE_NAV_PITCH_ASP 4
#define CASE_NAV_PITCH_ALT 5
#define CASE_TE_THROTTLE 6
#define CASE_ALT_THROTTLE 7

// Feedforward cases
// ----------------
#define CASE_PITCH_COMP 0
#define CASE_RUDDER_MIX 1
#define CASE_P_TO_T 2

// Auto Pilot modes
// ----------------
#define MANUAL 0
#define AUTO 10
//#define RTL 11

// Waypoint Modes
// ----------------
#define ABS_WP 0
#define REL_WP 1

// Events
// ------
#define EVENT_WILL_REACH_WAYPOINT 1
#define EVENT_SET_NEW_WAYPOINT_INDEX 2
#define EVENT_LOADED_WAYPOINT 3
#define EVENT_LOOP 4

//GPS_fix
#define VALID_GPS 0x00
#define BAD_GPS 0x01
#define FAILED_GPS 0x03

//#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))/0.000294117
#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))/0.2941176470588

#define AIRSPEED_PIN 3		// Need to correct value
#define BATTERY_PIN 5		// Need to correct value

//#define BLUE_LED_PIN 12			//36 = B,	37 = A,	35 = C

#define HOLD_ALT_ABOVE_HOME 8 // bitmask value


//GPS SIRF configuration strings... 
#define SIRF_BAUD_RATE_4800    "$PSRF100,1,4800,8,1,0*0E\r\n"
#define SIRF_BAUD_RATE_9600    "$PSRF100,1,9600,8,1,0*0D\r\n"
#define SIRF_BAUD_RATE_19200    "$PSRF100,1,19200,8,1,0*38\r\n"
#define SIRF_BAUD_RATE_38400    "$PSRF100,1,38400,8,1,0*3D\r\n"  
#define SIRF_BAUD_RATE_57600    "$PSRF100,1,57600,8,1,0*36\r\n"
#define GSA_ON   "$PSRF103,2,0,1,1*27\r\n"   // enable GSA
#define GSA_OFF  "$PSRF103,2,0,0,1*26\r\n"   // disable GSA
#define GSV_ON   "$PSRF103,3,0,1,1*26\r\n"  // enable GSV
#define GSV_OFF  "$PSRF103,3,0,0,1*27\r\n"  // disable GSV
#define USE_WAAS   1     //1 = Enable, 0 = Disable, good in USA, slower FIX... 
#define WAAS_ON    "$PSRF151,1*3F\r\n"       // enable WAAS
#define WAAS_OFF   "$PSRF151,0*3E\r\n"       // disable WAAS

//GPS Locosys configuration strings...
#define USE_SBAS 0
#define SBAS_ON "$PMTK313,1*2E\r\n"
#define SBAS_OFF "$PMTK313,0*2F\r\n"

#define LOCOSYS_REFRESH_RATE_200 "$PMTK220,200*2C" //200 milliseconds 
#define LOCOSYS_REFRESH_RATE_250 "$PMTK220,250*29\r\n" //250 milliseconds

#define LOCOSYS_BAUD_RATE_4800 "$PMTK251,4800*14\r\n"
#define LOCOSYS_BAUD_RATE_9600 "$PMTK251,9600*17\r\n"
#define LOCOSYS_BAUD_RATE_19200 "$PMTK251,19200*22\r\n"
#define LOCOSYS_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

//  GCS Message ID's
#define MSG_ACKNOWLEDGE 0x00
#define MSG_HEARTBEAT 0x01
#define MSG_ATTITUDE 0x02
#define MSG_LOCATION 0x03
#define MSG_PRESSURE 0x04
#define MSG_STATUS_TEXT 0x05
#define MSG_PERF_REPORT 0x06
#define MSG_COMMAND 0x22
#define MSG_VALUE 0x32
#define MSG_PID 0x42
#define MSG_TRIMS 0x50
#define MSG_MINS 0x51
#define MSG_MAXS 0x52


//EEPRROM 
#define EE_CONFIG 0x00
#define EE_AP_OFFSET 0x01
#define EE_ROLL_TRIM 0x03
#define EE_PITCH_TRIM 0x04
#define EE_MAX_ALT 0x05
#define EE_MAX_SPEED 0x07
#define EE_WP_TOTAL 0x09
#define EE_WP_INDEX 0x0A
#define EE_WP_RADIUS 0x0B
#define EE_HOME_ALT 0x0C
#define EE_HOME_LAT 0x0E
#define EE_HOME_LNG 0x12
#define EE_ALT_HOLD_HOME 0x16

#define EE_CH3_TT 0x384

#define EE_CH1_TRIM 0x386
#define EE_CH2_TRIM 0x388
#define EE_CH3_TRIM 0x388
#define EE_CH4_TRIM 0x38C  // fix

#define EE_CH1_MIN 0x38E
#define EE_CH2_MIN 0x390
#define EE_CH3_MIN 0x392
#define EE_CH4_MIN 0x394  // fix

#define EE_CH1_MAX 0x396
#define EE_CH2_MAX 0x398
#define EE_CH3_MAX 0x39A
#define EE_CH4_MAX 0x39C  // fix
// This needs to be changed
#define EE_ELEVON1_TRIM 0x39E
#define EE_ELEVON2_TRIM 0x3A0
#define EE_IR_MAX 0x3A2
