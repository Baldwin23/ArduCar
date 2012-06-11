void read_battery(void)
{
	filter_batt_voltage = ((float)analogRead(BATTERY_PIN) * .05) + (filter_batt_voltage * .95);
	battery_voltage = BATTERY_VOLTAGE(filter_batt_voltage);
	if(battery_voltage < INPUT_VOLTAGE)
          Serial.println("Low Battery!");
		//low_battery_event();
}

//TODO: HOW LONG DOES THIS TAKE??
long getFrontDistanceCM()
{
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);
  
  return duration / 29 / 2;
}



// returns the sensor values as degrees of roll
//   0 ----- 511  ---- 1023    IR Sensor
// -90°       0         90°	    degree output * 100
// sensors are limited to +- 60° (6000 when you multply by 100)

long getRoll(void)
{
	#if XY_SENSOR_LOCATION ==1
	return constrain((x_axis() + y_axis()) / 2, -6000, 6000);
	#endif
	
	#if XY_SENSOR_LOCATION ==0
	return constrain((-x_axis() - y_axis()) / 2, -6000, 6000);
	#endif

	#if XY_SENSOR_LOCATION ==3
	return constrain((-x_axis() - y_axis()) / 2, -6000, 6000);
	#endif
	
	#if XY_SENSOR_LOCATION ==2
	return constrain((x_axis() + y_axis()) / 2, -6000, 6000);
	#endif
}

long getPitch(void)
{
  #if XY_SENSOR_LOCATION ==1
  return constrain((-x_axis() + y_axis()) / 2, -6000, 6000);
  #endif
  
  #if XY_SENSOR_LOCATION ==0
  return constrain((x_axis() - y_axis()) / 2, -6000, 6000);
  #endif

  #if XY_SENSOR_LOCATION ==3
  return constrain((-x_axis() + y_axis()) / 2, -6000, 6000);
  #endif
  
  #if XY_SENSOR_LOCATION ==2
  return constrain((x_axis() - y_axis()) / 2, -6000, 6000);
  #endif
}

long x_axis(void)// roll
{
	return ((analog1 - 511l) * 9000l) / ir_max;
	//      611 - 511 
	//         100 * 9000 / 100 = 90°  low = underestimate  = 36 looks like 90 = flat plane or bouncy plane
	//         100 * 9000 / 250 = 36°   				    = 36 looks like 36
	//		   100 * 9000 / 500 = 18°  high = over estimate = 36 looks like 18 = crash plane
}

long y_axis(void)// pitch
{
	return ((analog0 - 511l) * 9000l) / ir_max;
}
