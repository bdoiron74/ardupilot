/*
 *       AP_Motors.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_Motors.h"

// parameters for the motor class
const AP_Param::GroupInfo AP_Motors::var_info[] PROGMEM = {
    // @Param: TB_RATIO
    // @DisplayName: Top Bottom Ratio
    // @Description: Not Used.  Will control the speed of the top motors vs bottom motors on frames such as the Octo-Quad and Y6
    AP_GROUPINFO("TB_RATIO", 0, AP_Motors,  top_bottom_ratio, AP_MOTORS_TOP_BOTTOM_RATIO),      // not used

    // @Param: TCRV_ENABLE
    // @DisplayName: Thrust Curve Enable
    // @Description: Controls whether a curve is used to linearize the thrust produced by the motors
    // @Values: 0:Disabled,1:Enable
    AP_GROUPINFO("TCRV_ENABLE", 1, AP_Motors, _throttle_curve_enabled, THROTTLE_CURVE_ENABLED),

    // @Param: TCRV_MIDPCT
    // @DisplayName: Thrust Curve mid-point percentage
    // @Description: Set the pwm position that produces half the maximum thrust of the motors
    // @Range: 20 80
    AP_GROUPINFO("TCRV_MIDPCT", 2, AP_Motors, _throttle_curve_mid, THROTTLE_CURVE_MID_THRUST),

    // @Param: TCRV_MAXPCT
    // @DisplayName: Thrust Curve max thrust percentage
    // @Description: Set to the lowest pwm position that produces the maximum thrust of the motors.  Most motors produce maximum thrust below the maximum pwm value that they accept.
    // @Range: 20 80
    AP_GROUPINFO("TCRV_MAXPCT", 3, AP_Motors, _throttle_curve_max, THROTTLE_CURVE_MAX_THRUST),

    // @Param: TBST_KA
    // @DisplayName: 
    // @Description: 
    // @Range: 0 - 1.0
    AP_GROUPINFO("TBST_KA", 4, AP_Motors, _throttle_boost_ka, THROTTLE_BOOST_KA),

    // @Param: TBST_KB
    // @DisplayName: 
    // @Description: 
    // @Range: 0 - 1.0
    AP_GROUPINFO("TBST_KB", 5, AP_Motors, _throttle_boost_kb, THROTTLE_BOOST_KB),

    // @Param: TBST_KB
    // @DisplayName: 
    // @Description: 
    // @Range: 0 - 1000
    AP_GROUPINFO("TBST_LIM", 6, AP_Motors, _throttle_boost_limit, THROTTLE_BOOST_LIM),

    // @Param: VTARGET
    // @DisplayName: 
    // @Description: Target battery voltage in V
    // @Range: 
    AP_GROUPINFO("VTARGET", 7, AP_Motors, _voltage_target, THROTTLE_VTARGET),

    // @Param: VTARGTC
    // @DisplayName: 
    // @Description: fact = (1.0-VTARGTC)*old + VTARGTC*new
    // @Range: 
    AP_GROUPINFO("VTARGTC", 8, AP_Motors, _voltage_tc, 0.01),

    AP_GROUPEND
};

// Constructor
AP_Motors::AP_Motors( uint8_t APM_version, APM_RC_Class* rc_out, RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz ) :
    _rc(rc_out),
    _rc_roll(rc_roll),
    _rc_pitch(rc_pitch),
    _rc_throttle(rc_throttle),
    _rc_yaw(rc_yaw),
    _speed_hz(speed_hz),
    _armed(false),
    _auto_armed(false),
    _frame_orientation(0),
    _min_throttle(AP_MOTORS_DEFAULT_MIN_THROTTLE),
    _max_throttle(AP_MOTORS_DEFAULT_MAX_THROTTLE)
{
    uint8_t i;

    top_bottom_ratio = AP_MOTORS_TOP_BOTTOM_RATIO;

    // initialise motor map
    if( APM_version == AP_MOTORS_APM1 ) {
        set_motor_to_channel_map(APM1_MOTOR_TO_CHANNEL_MAP);
    } else {
        set_motor_to_channel_map(APM2_MOTOR_TO_CHANNEL_MAP);
    }

    // clear output arrays
    for(i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        motor_out[i] = 0;
    }

    _mstate = S_STOPPED;
    _mstate_counter = 0;
    _mssb = 1;
};

// init
void AP_Motors::Init()
{
    // set-up throttle curve - motors classes will decide whether to use it based on _throttle_curve_enabled parameter
    setup_throttle_curve();
};

// throttle_pass_through - passes throttle through to motors - dangerous but used for initialising ESCs
void AP_Motors::throttle_pass_through()
{
    if( armed() ) {
        for( int16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++ ) 
        {
//          if(ap.failsafe)
//          {
//#if ESC3D == ENABLED
//            _rc->OutputCh(_motor_to_channel_map[i], _rc_throttle->radio_trim);
//#else
//            _rc->OutputCh(_motor_to_channel_map[i], _rc_throttle->radio_min);
//#endif
//          }
//          else
          {
            _rc->OutputCh(_motor_to_channel_map[i], _rc_throttle->radio_in);
          }
        }
    }
}

// setup_throttle_curve - used to linearlise thrust output by motors
// returns true if set up successfully
bool AP_Motors::setup_throttle_curve()
{
  //  int16_t min_pwm = _rc_throttle->radio_min;
  //  int16_t max_pwm = _rc_throttle->radio_max;
	//int16_t mid_throttle_pwm = (max_pwm + min_pwm) / 2;
  //  int16_t mid_thrust_pwm = min_pwm + (float)(max_pwm - min_pwm) * ((float)_throttle_curve_mid/100.0);
  //  int16_t max_thrust_pwm = min_pwm + (float)(max_pwm - min_pwm) * ((float)_throttle_curve_max/100.0);
    bool retval = true;

    // some basic checks that the curve is valid
//    if( mid_thrust_pwm >= (min_pwm+_min_throttle) && mid_thrust_pwm <= max_pwm && max_thrust_pwm >= mid_thrust_pwm && max_thrust_pwm <= max_pwm ) 
    if(1)
    {
        // clear curve
        _throttle_curve.clear();

        // curve initialisation
#if 0
        retval &= _throttle_curve.add_point(min_pwm, min_pwm);
        retval &= _throttle_curve.add_point(min_pwm+_min_throttle, min_pwm+_min_throttle);
        retval &= _throttle_curve.add_point(mid_throttle_pwm, mid_thrust_pwm);
        retval &= _throttle_curve.add_point(max_pwm, max_thrust_pwm);
#else // HACK to linearize thrust
#warning "Make parameters for some number of segments. This one just happens to be linear."
        // currently mapping _min_throttle [as %*10] to _throttle_curve_mid [as min_pwm]
#if ESC3D == ENABLED
        retval &= _throttle_curve.add_point(0,                  0);
        retval &= _throttle_curve.add_point(_min_throttle,      ((int16_t)_throttle_curve_mid)*10);  // SimonK has 1/16 dz at the bottom, but we might want to leave room
        retval &= _throttle_curve.add_point(1000,               969); // SimonK has 1/32 dz at the top
#else
                                            //  mot,  PWM
        retval &= _throttle_curve.add_point(_rc_throttle->radio_min,               _rc_throttle->radio_min);
        retval &= _throttle_curve.add_point(_rc_throttle->radio_min+_min_throttle, _rc_throttle->radio_min+((int16_t)_throttle_curve_mid)*10); 
        retval &= _throttle_curve.add_point(_rc_throttle->radio_max,               _rc_throttle->radio_max);     
#endif
#endif

        // return success
        return retval;
    }else{
        retval = false;
    }

    // disable throttle curve if not set-up corrrectly
    if( !retval ) {
        _throttle_curve_enabled = false;
        Serial.printf_P(PSTR("AP_Motors: failed to create throttle curve"));
    }

    return retval;
}