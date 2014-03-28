/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void
get_stabilize_roll(int32_t target_angle)
{
    // convert constrained angle error to desired Rate:
    int32_t target_rate = constrain(wrap_180(target_angle - target_ef.x), -4500, 4500) * g.acro_p;

    // set targets for rate controller
    set_roll_rate_target(target_rate, EARTH_FRAME);
}

static void
get_stabilize_pitch(int32_t target_angle)
{
    // convert constrained angle error to desired Rate:
    int32_t target_rate = constrain(wrap_180(target_angle - target_ef.y), -4500, 4500) * g.acro_p;

    // set targets for rate controller
    set_pitch_rate_target(target_rate, EARTH_FRAME);
}

static void
get_stabilize_yaw(int32_t target_angle)
{
    // convert constrained angle error to desired Rate:
    int32_t target_rate = constrain(wrap_180(target_angle - target_ef.z), -4500, 4500) * g.acro_p;

    // set targets for rate controller
    set_yaw_rate_target(target_rate, EARTH_FRAME);
}

static void
get_acro_yaw(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_yaw_rate_target(target_rate, BODY_FRAME);
}


// Yaw with rate input and stabilized in the earth frame
static void
get_yaw_rate_stabilized_ef()
{
	  set_yaw_rate_target(g.rc_4.control_in * g.acro_p, EARTH_FRAME);
}

// use max stick deflection of roll, pitch or yaw to determine level mix
float CalcLevelMix()
{
  float r = ((float)labs(g.rc_1.control_in)) / MAX_INPUT_ROLL_ANGLE;
  float p = ((float)labs(g.rc_2.control_in)) / MAX_INPUT_PITCH_ANGLE;

  p = max(r,p);

  return 1.0 - p;
}

int32_t CalcLimit(int32_t acro_rate, int32_t max_level_rate)
{
  return labs(labs(acro_rate) - max_level_rate);
}

// Roll with rate input and stabilized to body frame
static void
get_roll_rate_stabilized_bf()
{
    // Convert the input to the desired roll rate
    int32_t roll_rate = g.rc_1.control_in * g.acro_p;

    // Calculate rate limiter to avoid discontinuity when crossing 180
    float lf = CalcLevelMix() * (g.acro_balance_roll/100.0);
    int32_t limit = CalcLimit(roll_rate, lf * g.acro_p * 4500);
    
    // Calculate level rate for blending (similar to MultiWiii horizon mode), ignore roll level as we approach pitch +-90 (cos_pitch_x)
    int32_t level_rate = g.acro_p * constrain(-target_ef.x, -4500, 4500) * lf * cos_pitch_x;

    // combine/limit rate
    roll_rate = constrain((roll_rate + level_rate), -limit, limit);
    
    // Set body frame target for rate controller 
    set_roll_rate_target(roll_rate, BODY_FRAME);

}

// Pitch with rate input and stabilized to body frame
static void
get_pitch_rate_stabilized_bf()
{
    // Convert the input to the desired rate
    int32_t pitch_rate = g.rc_2.control_in * g.acro_p;
    int32_t yaw_rate = g.rc_4.control_in * g.acro_p;

    // Calculate pitch rate limiter to avoid discontinuity when crossing 180
    float lf = CalcLevelMix() * (g.acro_balance_pitch/100.0); 
    int32_t limit = CalcLimit(pitch_rate, lf * g.acro_p * 4500);

    // Calculate level rate for blending (similar to MultiWiii horizon mode)
    int32_t level_rate = g.acro_p * constrain(-target_ef.y, -4500, 4500) * lf;

    // when leveling (acro < max_level), maintain heading by splitting between pitch/yaw
    if(labs(pitch_rate) < (g.acro_p * 4500 * lf) ) 
    {
      // combine/limit rate
      pitch_rate = constrain((pitch_rate + level_rate), -limit, limit);

      yaw_rate += pitch_rate * (-sin_roll);
      pitch_rate *= cos_roll_x;
    }
    else
    {
      // combine/limit rate
      pitch_rate = constrain((pitch_rate + level_rate), -limit, limit);
    }

    // Set body frame target for rate controller 
    set_pitch_rate_target(pitch_rate, BODY_FRAME);
    set_yaw_rate_target(yaw_rate, BODY_FRAME);
}

//////////////////////////////////////////////////////////////////////////////



// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        roll_rate_target_bf = desired_rate;
    }else{
        roll_rate_target_ef = desired_rate;
    }
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        pitch_rate_target_bf = desired_rate;
    }else{
        pitch_rate_target_ef = desired_rate;
    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        yaw_rate_target_bf = desired_rate;
    }else{
        yaw_rate_target_ef = desired_rate;
    }
}



void update_axis_movement()
{
  float dt = ins.get_delta_time();

  error_bf = error_bf - ((omega * DEGX100) * dt);

  // calculate target rotation so stabilized modes see where we 'will' be
  Matrix3f temp = ahrs.get_dcm_matrix();
  temp.rotate(error_bf * RADX100);
  temp.to_euler(&target_ef.x, &target_ef.y, &target_ef.z);
  
  target_ef *= DEGX100;
  if(target_ef.z < 0) target_ef.z += 36000;
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
update_rate_contoller_targets()
{
    if( rate_targets_frame == EARTH_FRAME ) {
        // convert earth frame rates to body frame rates
        roll_rate_target_bf 	= roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        pitch_rate_target_bf 	= cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        yaw_rate_target_bf 		= cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }

    // now do body frame trajectory control:

    // limit acceleration to avoid unatainable error accumulation
    prev_roll_rate_bf  = constrain(roll_rate_target_bf,  prev_roll_rate_bf-g.acro_acclim_roll,   prev_roll_rate_bf+g.acro_acclim_roll);
    prev_pitch_rate_bf = constrain(pitch_rate_target_bf, prev_pitch_rate_bf-g.acro_acclim_pitch, prev_pitch_rate_bf+g.acro_acclim_pitch);
    prev_yaw_rate_bf   = constrain(yaw_rate_target_bf,   prev_yaw_rate_bf-g.acro_acclim_yaw,     prev_yaw_rate_bf+g.acro_acclim_yaw);

    // adjust rate target with correction
    roll_rate_target_bf  = prev_roll_rate_bf  + g.pid_stabilize_roll.get_p(error_bf.x)  + g.pid_stabilize_roll.get_i(error_bf.x, G_Dt)  + g.pid_stabilize_roll.get_d(error_bf.x, G_Dt);
    pitch_rate_target_bf = prev_pitch_rate_bf + g.pid_stabilize_pitch.get_p(error_bf.y) + g.pid_stabilize_pitch.get_i(error_bf.y, G_Dt) + g.pid_stabilize_pitch.get_d(error_bf.y, G_Dt);
    yaw_rate_target_bf   = prev_yaw_rate_bf   + g.pid_stabilize_yaw.get_p(error_bf.z)   + g.pid_stabilize_yaw.get_i(error_bf.z, G_Dt)   + g.pid_stabilize_yaw.get_d(error_bf.z, G_Dt);

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe)) {
        error_bf.x = 0;       error_bf.y = 0;       error_bf.z = 0;
        prev_roll_rate_bf = 0;   prev_pitch_rate_bf = 0;   prev_yaw_rate_bf = 0;
        roll_rate_target_bf = 0; pitch_rate_target_bf = 0; yaw_rate_target_bf = 0;
    }

    // Calculate target for next iteration
    error_bf.x = constrain(error_bf.x + (prev_roll_rate_bf * G_Dt),  -MAX_BF_ROLL_OVERSHOOT,  MAX_BF_ROLL_OVERSHOOT);
    error_bf.y = constrain(error_bf.y + (prev_pitch_rate_bf * G_Dt), -MAX_BF_PITCH_OVERSHOOT, MAX_BF_PITCH_OVERSHOOT);
    error_bf.z = constrain(error_bf.z + (prev_yaw_rate_bf * G_Dt),   -MAX_BF_YAW_OVERSHOOT,   MAX_BF_YAW_OVERSHOOT);

}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
run_rate_controllers()
{
#if FRAME_CONFIG == HELI_FRAME          // helicopters only use rate controllers for yaw and only when not using an external gyro
    if(!motors.ext_gyro_enabled) {
		g.rc_1.servo_out = get_heli_rate_roll(roll_rate_target_bf);
		g.rc_2.servo_out = get_heli_rate_pitch(pitch_rate_target_bf);
        g.rc_4.servo_out = get_heli_rate_yaw(yaw_rate_target_bf);
    }
#else
    // call rate controllers
    g.rc_1.servo_out = get_rate_roll(roll_rate_target_bf);
    g.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
    g.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);
#endif

    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
    if( g.throttle_accel_enabled && throttle_accel_controller_active ) {
        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
    }
}

#if FRAME_CONFIG == HELI_FRAME
// init_rate_controllers - set-up filters for rate controller inputs
void init_rate_controllers()
{
   // initalise low pass filters on rate controller inputs
   // 1st parameter is time_step, 2nd parameter is time_constant
   rate_roll_filter.set_cutoff_frequency(0.01, 2.0);
   rate_pitch_filter.set_cutoff_frequency(0.01, 2.0);
   // rate_yaw_filter.set_cutoff_frequency(0.01, 2.0);
   // other option for initialisation is rate_roll_filter.set_cutoff_frequency(<time_step>,<cutoff_freq>);
}

static int16_t
get_heli_rate_roll(int32_t target_rate)
{
    int32_t p,i,d,ff;                // used to capture pid values for logging
	int32_t current_rate;			// this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

	// get current rate
	current_rate    = (omega.x * DEGX100);

    // filter input
    current_rate = rate_roll_filter.apply(current_rate);
	
    // call pid controller
    rate_error  = target_rate - current_rate;
    p   = g.pid_rate_roll.get_p(rate_error);

    if (motors.flybar_mode == 1) {												// Mechanical Flybars get regular integral for rate auto trim
		if (target_rate > -50 && target_rate < 50){								// Frozen at high rates
			i		= g.pid_rate_roll.get_i(rate_error, G_Dt);
		} else {
			i		= g.pid_rate_roll.get_integrator();
		}
	} else {
		i			= g.pid_rate_roll.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);		// Flybarless Helis get huge I-terms. I-term controls much of the rate
	}
	
	d = g.pid_rate_roll.get_d(rate_error, G_Dt);
	
	ff = g.heli_roll_ff * target_rate;

    output = p + i + d + ff;

    // constrain output
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_heli_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d,ff;                                                                   // used to capture pid values for logging
	int32_t current_rate;                                                     			// this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

	// get current rate
	current_rate    = (omega.y * DEGX100);
	
	// filter input
	current_rate = rate_pitch_filter.apply(current_rate);
	
	// call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);						// Helicopters get huge feed-forward
	
	if (motors.flybar_mode == 1) {												// Mechanical Flybars get regular integral for rate auto trim
		if (target_rate > -50 && target_rate < 50){								// Frozen at high rates
			i		= g.pid_rate_pitch.get_i(rate_error, G_Dt);
		} else {
			i		= g.pid_rate_pitch.get_integrator();
		}
	} else {
		i			= g.pid_rate_pitch.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);	// Flybarless Helis get huge I-terms. I-term controls much of the rate
	}
	
	d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
	
	ff = g.heli_pitch_ff*target_rate;
    
    output = p + i + d + ff;

    // constrain output
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        if( pid_log_counter == 0 ) {               // relies on get_heli_rate_roll to update pid_log_counter
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, 0, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_heli_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d,ff;                                                                   // used to capture pid values for logging
	int32_t current_rate;                                                               // this iteration's rate
    int32_t rate_error;
    int32_t output;

	// get current rate
    current_rate    = (omega.z * DEGX100);
	
	// filter input
	// current_rate = rate_yaw_filter.apply(current_rate);
	
    // rate control
    rate_error              = target_rate - current_rate;

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    
    i = g.pid_rate_yaw.get_i(rate_error, G_Dt);

    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);
	
	ff = g.heli_yaw_ff*target_rate;

    output  = p + i + d + ff;
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_YAW_RATE_KP || g.radio_tuning == CH6_YAW_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
	
#endif

	// output control
	return output;
}
#endif // HELI_FRAME

#if FRAME_CONFIG != HELI_FRAME
static int16_t
get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
    current_rate    = (omega.x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = g.pid_rate_roll.get_p(rate_error);

    // freeze I term if we've breached roll-pitch limits
    if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
        i	= g.pid_rate_roll.get_integrator();
    }else{
        //i   = g.pid_rate_roll.get_i(rate_error, G_Dt);
        // testing leaky integrators for better takeoff stability
        i = g.pid_rate_roll.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);
    }

    d = g.pid_rate_roll.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        pid_log_counter++;                          // Note: get_rate_pitch pid logging relies on this function to update pid_log_counter so if you change the line above you must change the equivalent line in get_rate_pitch
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    current_rate    = (omega.y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);
    // freeze I term if we've breached roll-pitch limits
    if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
        i = g.pid_rate_pitch.get_integrator();
    }else{
        //i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
        // testing leaky integrators for better takeoff stability
        i = g.pid_rate_pitch.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);
    }
    d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        if( pid_log_counter == 0 ) {               // relies on get_rate_roll having updated pid_log_counter
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    rate_error              = target_rate - (omega.z * DEGX100);

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    // freeze I term if we've breached yaw limits
    if( motors.reached_limit(AP_MOTOR_YAW_LIMIT) ) {
        i = g.pid_rate_yaw.get_integrator();
    }else{
        // i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
        // testing leaky integrators for better takeoff stability
        i = g.pid_rate_yaw.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);
    }
    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    output  = p+i+d;
    output = constrain(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && g.radio_tuning == CH6_YAW_RATE_KP ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

#if FRAME_CONFIG == TRI_FRAME
    // constrain output
    return output;
#else // !TRI_FRAME
    // output control:
    int16_t yaw_limit = 2200 + abs(g.rc_4.control_in);

    // smoother Yaw control:
    return constrain(output, -yaw_limit, yaw_limit);
#endif // TRI_FRAME
}
#endif // !HELI_FRAME

// calculate modified roll/pitch depending upon optical flow calculated position
static int32_t
get_of_roll(int32_t input_roll)
{
#if OPTFLOW == ENABLED
    static float tot_x_cm = 0;      // total distance from target
    static uint32_t last_of_roll_update = 0;
    int32_t new_roll = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_roll_update) {
        last_of_roll_update = optflow.last_update;

        // add new distance moved
        tot_x_cm += optflow.x_cm;

        // only stop roll if caller isn't modifying roll
        if( input_roll == 0 && current_loc.alt < 1500) {
            p = g.pid_optflow_roll.get_p(-tot_x_cm);
            i = g.pid_optflow_roll.get_i(-tot_x_cm,1.0);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_roll.get_d(-tot_x_cm,1.0);
            new_roll = p+i+d;
        }else{
            g.pid_optflow_roll.reset_I();
            tot_x_cm = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }
        // limit amount of change and maximum angle
        of_roll = constrain(new_roll, (of_roll-20), (of_roll+20));

 #if LOGGING_ENABLED == ENABLED
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            pid_log_counter++;              // Note: get_of_pitch pid logging relies on this function updating pid_log_counter so if you change the line above you must change the equivalent line in get_of_pitch
            if( pid_log_counter >= 5 ) {    // (update rate / desired output rate) = (100hz / 10hz) = 10
                pid_log_counter = 0;
                Log_Write_PID(CH6_OPTFLOW_KP, tot_x_cm, p, i, d, of_roll, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_roll = constrain(of_roll, -1000, 1000);

    return input_roll+of_roll;
#else
    return input_roll;
#endif
}

static int32_t
get_of_pitch(int32_t input_pitch)
{
#if OPTFLOW == ENABLED
    static float tot_y_cm = 0;  // total distance from target
    static uint32_t last_of_pitch_update = 0;
    int32_t new_pitch = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_pitch_update ) {
        last_of_pitch_update = optflow.last_update;

        // add new distance moved
        tot_y_cm += optflow.y_cm;

        // only stop roll if caller isn't modifying pitch
        if( input_pitch == 0 && current_loc.alt < 1500 ) {
            p = g.pid_optflow_pitch.get_p(tot_y_cm);
            i = g.pid_optflow_pitch.get_i(tot_y_cm, 1.0);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_pitch.get_d(tot_y_cm, 1.0);
            new_pitch = p + i + d;
        }else{
            tot_y_cm = 0;
            g.pid_optflow_pitch.reset_I();
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change
        of_pitch = constrain(new_pitch, (of_pitch-20), (of_pitch+20));

 #if LOGGING_ENABLED == ENABLED
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            if( pid_log_counter == 0 ) {        // relies on get_of_roll having updated the pid_log_counter
                Log_Write_PID(CH6_OPTFLOW_KP+100, tot_y_cm, p, i, d, of_pitch, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_pitch = constrain(of_pitch, -1000, 1000);

    return input_pitch+of_pitch;
#else
    return input_pitch;
#endif
}

/*************************************************************
 * yaw controllers
 *************************************************************/

static void get_look_ahead_yaw(int16_t pilot_yaw)
{
    // Commanded Yaw to automatically look ahead.
    if (g_gps->fix && g_gps->ground_course > YAW_LOOK_AHEAD_MIN_SPEED) {
        nav_yaw = get_yaw_slew(nav_yaw, g_gps->ground_course, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(wrap_360(nav_yaw + pilot_yaw));   // Allow pilot to "skid" around corners up to 45 degrees
    }else{
        nav_yaw += pilot_yaw * g.acro_p * G_Dt;
        nav_yaw = wrap_360(nav_yaw);
        get_stabilize_yaw(nav_yaw);
    }
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
static void update_throttle_cruise(int16_t throttle)
{
    // ensure throttle_avg has been initialised
    if( throttle_avg == 0 ) {
        throttle_avg = g.throttle_cruise;
    }
    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * .99 + (float)throttle * .01;
        g.throttle_cruise = throttle_avg;
    }
}

#if FRAME_CONFIG == HELI_FRAME
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
// for traditional helicopters
static int16_t get_angle_boost(int16_t throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
    angle_boost_factor = 1.0 - constrain(angle_boost_factor, .5, 1.0);
    int16_t throttle_above_mid = max(throttle - motors.throttle_mid,0);

    // to allow logging of angle boost
    angle_boost = throttle_above_mid*angle_boost_factor;

    return throttle + angle_boost;
}
#else   // all multicopters
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
static int16_t get_angle_boost(int16_t throttle)
{
    float temp = cos_pitch_x * cos_roll_x;
    int16_t throttle_out;

    temp = constrain(temp, .5, 1.0);

    // reduce throttle if we go inverted
    temp = constrain(9000-max(labs(ahrs.roll_sensor),labs(ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

    // apply scale and constrain throttle
    throttle_out = constrain((float)(throttle-g.throttle_min) * temp + g.throttle_min, g.throttle_min, 1000);

    // to allow logging of angle boost
    angle_boost = throttle_out - throttle;

    return throttle_out;
}
#endif // FRAME_CONFIG == HELI_FRAME

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost )
{
    if( apply_angle_boost ) {
        g.rc_3.servo_out = get_angle_boost(throttle_out);
    }else{
        g.rc_3.servo_out = throttle_out;
        // clear angle_boost for logging purposes
        angle_boost = 0;
    }
}

// set_throttle_accel_target - to be called by upper throttle controllers to set desired vertical acceleration in earth frame
void set_throttle_accel_target( int16_t desired_acceleration )
{
    if( g.throttle_accel_enabled ) {
        throttle_accel_target_ef = desired_acceleration;
        throttle_accel_controller_active = true;
    }else{
        // To-Do log dataflash or tlog error
        cliSerial->print_P(PSTR("Err: target sent to inactive acc thr controller!\n"));
    }
}

// disable_throttle_accel - disables the accel based throttle controller
// it will be re-enasbled on the next set_throttle_accel_target
// required when we wish to set motors to zero when pilot inputs zero throttle
void throttle_accel_deactivate()
{
    throttle_accel_controller_active = false;
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors
static int16_t
get_throttle_accel(int16_t z_target_accel)
{
    static float z_accel_error = 0;     // The acceleration error in cm.
    static uint32_t last_call_ms = 0;   // the last time this controller was called
    int32_t p,i,d;                      // used to capture pid values for logging
    int16_t output;
    float z_accel_meas;
    uint32_t now = millis();

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(ahrs.get_accel_ef().z + gravity) * 100;

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_accel_error = 0;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        z_accel_error = z_accel_error + 0.11164 * (constrain(z_target_accel - z_accel_meas, -32000, 32000) - z_accel_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle_accel.get_p(z_accel_error);
    // freeze I term if we've breached throttle limits
    if( motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT) ) {
        i = g.pid_throttle_accel.get_integrator();
    }else{
        i = g.pid_throttle_accel.get_i(z_accel_error, .01);
    }
    d = g.pid_throttle_accel.get_d(z_accel_error, .01);

    //
    // limit the rate
    output =  constrain(p+i+d+g.throttle_cruise, g.throttle_min, g.throttle_max);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_THR_ACCEL_KP || g.radio_tuning == CH6_THR_ACCEL_KI || g.radio_tuning == CH6_THR_ACCEL_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (50hz / 10hz) = 5hz
            pid_log_counter = 0;
            Log_Write_PID(CH6_THR_ACCEL_KP, z_accel_error, p, i, d, output, tuning_value);
        }
    }
#endif

    return output;
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output [g.throttle_min to g.throttle_max]
static int16_t get_pilot_desired_throttle()   
{
    int16_t throttle_control = g.rc_3.control_in; // 0:DZ,DZ+1:1000 => 0:0,g.throttle_min:g.throttle_max
    int16_t mid_stick = g.rc_3.trim_range;        // control value at mid stick (defined by trim)
    int16_t throttle_out;

    // ensure reasonable throttle values
    throttle_control = constrain(throttle_control,0,1000);
    g.throttle_mid = constrain(g.throttle_mid,300,700);

    // check throttle is above, below trim
    if (throttle_control < mid_stick) // below trim
    {        
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid-g.throttle_min))/((float)(mid_stick-g.throttle_min));
    }
    else // above trim
    {
        throttle_out = g.throttle_mid + ((float)(throttle_control-mid_stick))*((float)(g.throttle_max-g.throttle_mid))/((float)(g.throttle_max-mid_stick));
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define PILOTZ_DZ 50        // the throttle input channel's deadband in PWM
static int16_t get_pilot_desired_climb_rate()
{
    int16_t throttle_control = g.rc_3.control_in; // 0:DZ,DZ+1:1000 => 0:0,g.throttle_min:g.throttle_max
    int16_t dz_top = g.rc_3.trim_range + (PILOTZ_DZ/2); 
    int16_t dz_bot = g.rc_3.trim_range - (PILOTZ_DZ/2); 

    int16_t desired_rate = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < dz_bot) {
        // below the deadband
        desired_rate = ((int32_t)g.pilot_velocity_z_max * (throttle_control-dz_bot)) / (dz_bot-g.throttle_min);
    }else if (throttle_control > dz_top) {
        // above the deadband
        desired_rate = ((int32_t)g.pilot_velocity_z_max * (throttle_control-dz_top)) / (g.throttle_max-dz_top);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

//TODO remove all dependecies on these
#define THROTTLE_IN_MIDDLE 500
#define THROTTLE_IN_DEADBAND 100        // the throttle input channel's deadband in PWM
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+THROTTLE_IN_DEADBAND)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-THROTTLE_IN_DEADBAND)  // bottom of the deadband

// get_pilot_desired_acceleration - transform pilot's throttle input to a desired acceleration
// default upper and lower bounds are 500 cm/s/s (roughly 1/2 a G)
// returns acceleration in cm/s/s
static int16_t get_pilot_desired_acceleration(int16_t throttle_control)
{
    int32_t desired_accel = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_accel = (int32_t)ACCELERATION_MAX_Z * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else if(throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_accel = (int32_t)ACCELERATION_MAX_Z * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else{
        // must be in the deadband
        desired_accel = 0;
    }

    return desired_accel;
}

// get_pilot_desired_direct_alt - transform pilot's throttle input to a desired altitude
// return altitude in cm between 0 to 10m
static int32_t get_pilot_desired_direct_alt(int16_t throttle_control)
{
    int32_t desired_alt = 0;

    // throttle failsafe check
    if( ap.failsafe ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain(throttle_control,0,1000);

    desired_alt = throttle_control;

    return desired_alt;
}

// get_throttle_rate - calculates desired accel required to achieve desired z_target_speed
// sets accel based throttle controller target
static void
get_throttle_rate(int16_t z_target_speed)
{
    static uint32_t last_call_ms = 0;
    static float z_rate_error = 0;   // The velocity error in cm.
    int32_t p,i,d;      // used to capture pid values for logging
    int16_t output;     // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors
    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_rate_error    = 0;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        z_rate_error    = z_rate_error + 0.20085 * ((z_target_speed - climb_rate) - z_rate_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle.get_p(z_rate_error);

    // freeze I term if we've breached throttle limits
    if(motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT)) {
        i = g.pid_throttle.get_integrator();
    }else{
        i = g.pid_throttle.get_i(z_rate_error, .02);
    }
    d = g.pid_throttle.get_d(z_rate_error, .02);

    // consolidate target acceleration
    output =  p+i+d;

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_THROTTLE_KP || g.radio_tuning == CH6_THROTTLE_KI || g.radio_tuning == CH6_THROTTLE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (50hz / 10hz) = 5hz
            pid_log_counter = 0;
            Log_Write_PID(CH6_THROTTLE_KP, z_rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // send output to accelerometer based throttle controller if enabled otherwise send directly to motors
    if( g.throttle_accel_enabled ) {
        // set target for accel based throttle controller
        set_throttle_accel_target(output);
    }else{
        set_throttle_out(g.throttle_cruise+output, true);
    }

    // update throttle cruise
    // TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
    if( z_target_speed == 0 ) {
        update_throttle_cruise(g.rc_3.servo_out);
    }
}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
static void
get_throttle_althold(int32_t target_alt, int16_t target_climb_rate, int16_t min_climb_rate, int16_t max_climb_rate)
{
    int32_t alt_error;
    int16_t desired_rate;
    int32_t linear_distance;      // the distace we swap between linear and sqrt.

    // calculate altitude error
    alt_error    = target_alt - current_loc.alt;

    // check kP to avoid division by zero
    if( g.pi_alt_hold.kP() != 0 ) {
        linear_distance = 250/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if( alt_error > 2*linear_distance ) {
            desired_rate = safe_sqrt(2*250*(alt_error-linear_distance));
        }else if( alt_error < -2*linear_distance ) {
            desired_rate = -safe_sqrt(2*250*(-alt_error-linear_distance));
        }else{
            desired_rate = g.pi_alt_hold.get_p(alt_error);
        }
    }else{
        desired_rate = 0;
    }
    desired_rate = desired_rate + target_climb_rate; // correction + continuing rate
    desired_rate = constrain(desired_rate, min_climb_rate, max_climb_rate);

    // call rate based throttle controller which will update accel based throttle controller targets
    get_throttle_rate(desired_rate);

    // update altitude error reported to GCS
    altitude_error = alt_error;

    // TO-DO: enabled PID logging for this controller
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
static void
get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    // throttle controllers running at 50Hz: dt = 0.02
    int32_t delta = constrain(target_alt-controller_desired_alt, min_climb_rate*0.02, max_climb_rate*0.02);
    // limit target altitude change
    controller_desired_alt += delta;

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);
    
    // throttle controllers running at 50Hz
    get_throttle_althold(controller_desired_alt, delta*50, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_rate_stabilized - rate controller with additional 'stabilizer'
// 'stabilizer' ensure desired rate is being met
// calls normal throttle rate controller which updates accel based throttle controller targets
static void
get_throttle_rate_stabilized(int16_t target_rate)
{
    controller_desired_alt += target_rate * 0.02; // throttle controllers running at 50Hz: dt = 0.02

    int16_t delta_lim = 750;
    if(target_rate != 0) // only limit when outside of the deadzone
    {
      delta_lim = constrain((abs(target_rate) >> 3), 20, 750); // 125ms 'lag' when pilot adjusting  (essentially velocity control...)
    }
    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain(controller_desired_alt,current_loc.alt-delta_lim,current_loc.alt+delta_lim);

    set_new_altitude(controller_desired_alt);

    get_throttle_althold(controller_desired_alt, target_rate, -g.pilot_velocity_z_max-250, g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_land - high level landing logic
// sends the desired acceleration in the accel based throttle controller
// called at 50hz
static void
get_throttle_land()
{
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (current_loc.alt >= LAND_START_ALT && !(g.sonar_enabled && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        get_throttle_althold_with_slew(LAND_START_ALT, g.auto_velocity_z_min, -abs(g.land_speed));
    }else{
        get_throttle_rate_stabilized(-abs(g.land_speed));

        // detect whether we have landed by watching for minimum throttle and now movement
        if (abs(climb_rate) < 20 && (g.rc_3.servo_out <= get_angle_boost(g.throttle_min) || g.pid_throttle_accel.get_integrator() <= -150)) {
            if( land_detector < LAND_DETECTOR_TRIGGER ) {
                land_detector++;
            }else{
                set_land_complete(true);
                if( g.rc_3.control_in == 0 || ap.failsafe ) {
                    init_disarm_motors();
                }
            }
        }else{
            // we've sensed movement up or down so decrease land_detector
            if (land_detector > 0 ) {
                land_detector--;
            }
        }
    }
}

// get_throttle_surface_tracking - hold copter at the desired distance above the ground
// updates accel based throttle controller targets
static void
get_throttle_surface_tracking(int16_t target_rate)
{
    static float target_sonar_alt = 0;   // The desired altitude in cm above the ground
    static uint32_t last_call_ms = 0;
    float distance_error;
    float sonar_induced_slew_rate;

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 200 ) {
        target_sonar_alt = sonar_alt + controller_desired_alt - current_loc.alt;
    }
    last_call_ms = now;

    target_sonar_alt += target_rate * 0.02;

    distance_error = (target_sonar_alt-sonar_alt);
    sonar_induced_slew_rate = constrain(fabs(THR_SURFACE_TRACKING_P * distance_error),0,THR_SURFACE_TRACKING_VELZ_MAX);

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain(target_sonar_alt,sonar_alt-750,sonar_alt+750);
    controller_desired_alt = current_loc.alt+(target_sonar_alt-sonar_alt);
    set_new_altitude(controller_desired_alt);

    get_throttle_althold_with_slew(controller_desired_alt, target_rate-sonar_induced_slew_rate, target_rate+sonar_induced_slew_rate);   // VELZ_MAX limits how quickly we react
}

/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    reset_rate_I();
    reset_stability_I();
    reset_wind_I();
    reset_throttle_I();
    reset_optflow_I();

    // This is the only place we reset Yaw
    g.pid_stabilize_yaw.reset_I();
}

static void reset_rate_I()
{
    g.pid_rate_roll.reset_I();
    g.pid_rate_pitch.reset_I();
    g.pid_rate_yaw.reset_I();
}

static void reset_optflow_I(void)
{
    g.pid_optflow_roll.reset_I();
    g.pid_optflow_pitch.reset_I();
    of_roll = 0;
    of_pitch = 0;
}

static void reset_wind_I(void)
{
    // Wind Compensation
    // this i is not currently being used, but we reset it anyway
    // because someone may modify it and not realize it, causing a bug
    g.pi_loiter_lat.reset_I();
    g.pi_loiter_lon.reset_I();

    g.pid_loiter_rate_lat.reset_I();
    g.pid_loiter_rate_lon.reset_I();

    g.pid_nav_lat.reset_I();
    g.pid_nav_lon.reset_I();
}

static void reset_throttle_I(void)
{
    // For Altitude Hold
    g.pi_alt_hold.reset_I();
    g.pid_throttle.reset_I();
    g.pid_throttle_accel.reset_I();
}

static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_throttle_accel.set_integrator(pilot_throttle-g.throttle_cruise);
}

static void reset_stability_I(void)
{
    // Used to balance a quad
    // This only needs to be reset during Auto-leveling in flight
    g.pid_stabilize_roll.reset_I();
    g.pid_stabilize_pitch.reset_I();
}
