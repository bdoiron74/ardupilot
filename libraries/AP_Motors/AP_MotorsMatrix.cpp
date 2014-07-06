/*
 *       AP_MotorsMatrix.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsMatrix.h"

// Init
void AP_MotorsMatrix::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate( uint16_t speed_hz )
{
    uint32_t fast_channel_mask = 0;
    int8_t i;

    // record requested speed
    _speed_hz = speed_hz;

    // check each enabled motor
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            // set-up fast channel mask
            fast_channel_mask |= _BV(_motor_to_channel_map[i]);                 // add to fast channel map
        }
    }

    // enable fast channels
    _rc->SetFastOutputChannels(fast_channel_mask, _speed_hz);
}

// set frame orientation (normally + or X)
void AP_MotorsMatrix::set_frame_orientation( uint8_t new_orientation )
{
    // return if nothing has changed
    if( new_orientation == _frame_orientation ) {
        return;
    }

    // call parent
    AP_Motors::set_frame_orientation( new_orientation );

    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsMatrix::enable()
{
    int8_t i;

    // enable output channels
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            _rc->enable_out(_motor_to_channel_map[i]);
        }
    }
}

// output_min - sends minimum values out to the motors
void AP_MotorsMatrix::output_min()
{
    int8_t i;

    // fill the motor_out[] array for HIL use and send minimum value to each motor
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
#if ESC3D == ENABLED
            motor_out[i] = _rc_throttle->radio_trim;
            _rc->OutputCh(_motor_to_channel_map[i], motor_out[i]);
            _motor_v_estimate[i] = 0;
#else
            motor_out[i] = _rc_throttle->radio_min;
            _rc->OutputCh(_motor_to_channel_map[i], motor_out[i]);
            _motor_v_estimate[i] = _rc_throttle->radio_min;
#endif
        }
    }
}

extern FastSerial *cliSerial;

// output_armed - sends commands to the motors
void AP_MotorsMatrix::output_armed()
{
    int8_t i;
#if ESC3D == ENABLED
    int16_t out_min = _min_throttle;
    int16_t out_max = 1000; // rescale to radio_min:radio_trim:radio_max later
#else
    int16_t out_min = _rc_throttle->radio_min + _min_throttle;
    int16_t out_max = _rc_throttle->radio_max;
#endif
    int16_t rc_yaw_constrained_pwm;
    int16_t rc_yaw_excess;
    int16_t upper_margin, lower_margin;
    int16_t motor_adjustment = 0;
    int16_t yaw_to_execute = 0;
    int16_t servo_out = _rc_throttle->servo_out;

#if ESC3D == ENABLED
    int8_t r;
#else
    const int8_t r = 1;
    const int8_t sb = 0;
#endif

    // initialize reached_limit flag
    _reached_limit = AP_MOTOR_NO_LIMITS_REACHED;
#if ESC3D == ENABLED
    // Throttle is -1000 to 1000
    r = dir(servo_out); // -1, 0, +1
    _mssb = false;
#define BRAKE_COUNT 10

    switch(_mstate)
    {
      case S_STOPPED:
        servo_out = 0;
        _mssb = true;
        if(r == 1) _mstate = S_FWD_START;
        else if(r == -1) _mstate = S_REV_START;
        _mstate_counter = 0;
        break;

      case S_FWD_START:
        servo_out = _min_throttle;// + (abs(servo_out) >> (BRAKE_COUNT-_mstate_counter));
        _mssb = true;
        if(r == 1) { if (++_mstate_counter >= BRAKE_COUNT) { _mstate = S_FWD; } }
        else { _mstate = S_BRK; }
        break;

      case S_REV_START:
        servo_out = _min_throttle;// + (abs(servo_out) >> (BRAKE_COUNT-_mstate_counter));
        _mssb = true;
        if(r == -1) { if(++_mstate_counter >= BRAKE_COUNT) { _mstate = S_REV; } }
        else { _mstate = S_BRK; }
        break;

      case S_FWD:
        servo_out = constrain(abs(servo_out), _min_throttle, _max_throttle);
        if(r != 1) { _mstate = S_BRK; }
        break;

      case S_REV:
        servo_out = constrain(abs(servo_out), _min_throttle, _max_throttle);
        if(r != -1) { _mstate = S_BRK; }
        break;

      case S_BRK:
        servo_out = 0;
        _mssb = true;
        if(--_mstate_counter <= 0) _mstate = S_STOPPED;
        break;

      default:
        servo_out = 0;
        _mstate = S_BRK;
        _mstate_counter = BRAKE_COUNT;
        break;
    }
#else
    // Throttle is 0 to 1000 only
    servo_out = constrain(servo_out, 0, _max_throttle);
todo: _mssb
#endif

    // capture desired roll, pitch, yaw and throttle from receiver

    // these three are constrained to servo_out=[-5000:5000], which turns into pwm_out = -500:500
    _rc_roll->calc_pwm();
    _rc_pitch->calc_pwm();
    _rc_yaw->calc_pwm();

    if(_mssb) // don't add r/p/y when starting or braking
    {
      // if we have any roll, pitch or yaw input then it's breaching the limit
      if( _rc_roll->pwm_out != 0 || _rc_pitch->pwm_out != 0 ) {
          _reached_limit |= AP_MOTOR_ROLLPITCH_LIMIT;
      }
      if( _rc_yaw->pwm_out != 0 ) {
          _reached_limit |= AP_MOTOR_YAW_LIMIT;
      }
      _rc_roll->pwm_out = 0;
      _rc_pitch->pwm_out = 0;
      _rc_yaw->pwm_out = 0;
    }
#if ESC3D == ENABLED
#else
    _rc_throttle->calc_pwm(); // fills in radio_out
#endif


    // if we are not sending a throttle output, we cut the motors
    if(servo_out == 0) {
        for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
            if( motor_enabled[i] ) {
#if ESC3D == ENABLED
                motor_out[i]    = _rc_throttle->radio_trim;
                _motor_v_estimate[i] = 0;
#else
                motor_out[i]    = _rc_throttle->radio_min;
#endif
            }
        }
    } else {    // non-zero throttle

        // initialise rc_yaw_contrained_pwm that we will certainly output and rc_yaw_excess that we will do on best-efforts basis.
        // Note: these calculations and many others below depend upon _yaw_factors always being 0, -1 or 1.
        if(_frame_orientation != AP_MOTORS_V_FRAME)
        {
          if( _rc_yaw->pwm_out < -AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM ) {
              rc_yaw_constrained_pwm = -AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
              rc_yaw_excess = _rc_yaw->pwm_out+AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
          }else if( _rc_yaw->pwm_out > AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM ) {
              rc_yaw_constrained_pwm = AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
              rc_yaw_excess = _rc_yaw->pwm_out-AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
          }else
          {
              rc_yaw_constrained_pwm = _rc_yaw->pwm_out;
              rc_yaw_excess = 0;
          }
        }
        else // disabled limiting for VTail hack
        {
          rc_yaw_constrained_pwm = _rc_yaw->pwm_out;
          rc_yaw_excess = 0;
        }
        // initialise upper and lower margins
        upper_margin = lower_margin = out_max - out_min;

        // add roll, pitch, throttle and constrained yaw for each motor
        for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
            if( motor_enabled[i] ) {
#if ESC3D == ENABLED
                motor_out[i] = (servo_out * _throttle_factor[i]) +
                               (r * (_rc_roll->pwm_out * _roll_factor[i] +
                                     _rc_pitch->pwm_out * _pitch_factor[i] +
                                     rc_yaw_constrained_pwm * _yaw_factor[i]));
#else
              x
#error "Throttle Factor won't work like this... radio_out = 1000:2000"
                motor_out[i] = _rc_throttle->radio_out * _throttle_factor[i] +
                               _rc_roll->pwm_out * _roll_factor[i] +
                               _rc_pitch->pwm_out * _pitch_factor[i] +
                               rc_yaw_constrained_pwm * _yaw_factor[i];
#endif

                // calculate remaining room between fastest running motor and top of pwm range
                if( out_max - motor_out[i] < upper_margin) {
                    upper_margin = out_max - motor_out[i];
                }
                // calculate remaining room between slowest running motor and bottom of pwm range
                if( motor_out[i] - out_min < lower_margin ) {
                    lower_margin = motor_out[i] - out_min;
                }
            }
        }

        // if motors are running too fast and we have enough room below, lower overall throttle
        if( upper_margin < 0 || lower_margin < 0 ) {

            // calculate throttle adjustment that equalizes upper and lower margins.  We will never push the throttle beyond this point
            motor_adjustment = (upper_margin - lower_margin) / 2;      // i.e. if overflowed by 20 on top, 30 on bottom, upper_margin = -20, lower_margin = -30.  will adjust motors -5.

            // if we have overflowed on the top, reduce but no more than to the mid point
            if( upper_margin < 0 ) {
                motor_adjustment = max(upper_margin, motor_adjustment);
            }

            // if we have underflowed on the bottom, increase throttle but no more than to the mid point
            if( lower_margin < 0 ) {
                motor_adjustment = min(-lower_margin, motor_adjustment);
            }
        }

        // move throttle up or down to to pull within tolerance
        if( motor_adjustment != 0 ) {
            for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
                if( motor_enabled[i] ) {
                    motor_out[i] += motor_adjustment;
                }
            }

            // we haven't even been able to apply roll, pitch and minimal yaw without adjusting throttle so mark all limits as breached
            _reached_limit |= AP_MOTOR_ROLLPITCH_LIMIT | AP_MOTOR_YAW_LIMIT | AP_MOTOR_THROTTLE_LIMIT;
        }

        // if we didn't give all the yaw requested, calculate how much additional yaw we can add
        if( rc_yaw_excess != 0 ) {

            // try for everything
            yaw_to_execute = rc_yaw_excess;

            // loop through motors and reduce as necessary
            for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
                if( motor_enabled[i] && (_yaw_factor[i]*r) != 0 ) {

                    // calculate upper and lower margins for this motor
                    upper_margin = max(0,out_max - motor_out[i]);
                    lower_margin = max(0,motor_out[i] - out_min);

                    // motor is increasing, check upper limit
                    if( rc_yaw_excess > 0 && (_yaw_factor[i]*r) > 0 ) {
                        yaw_to_execute = min(yaw_to_execute, upper_margin);
                    }

                    // motor is decreasing, check lower limit
                    if( rc_yaw_excess > 0 && (_yaw_factor[i]*r) < 0 ) {
                        yaw_to_execute = min(yaw_to_execute, lower_margin);
                    }

                    // motor is decreasing, check lower limit
                    if( rc_yaw_excess < 0 && (_yaw_factor[i]*r) > 0 ) {
                        yaw_to_execute = max(yaw_to_execute, -lower_margin);
                    }

                    // motor is increasing, check upper limit
                    if( rc_yaw_excess < 0 && (_yaw_factor[i]*r) < 0 ) {
                        yaw_to_execute = max(yaw_to_execute, -upper_margin);
                    }
                }
            }
            // check yaw_to_execute is reasonable
            if( yaw_to_execute != 0 && ((yaw_to_execute>0 && rc_yaw_excess>0) || (yaw_to_execute<0 && rc_yaw_excess<0)) ) {
                // add the additional yaw
                for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
                    if( motor_enabled[i] ) {
                        motor_out[i] += (_yaw_factor[i]*r) * yaw_to_execute;
                    }
                }
            }
            // mark yaw limit reached if we didn't get everything we asked for
            if( yaw_to_execute != rc_yaw_excess ) {
                _reached_limit |= AP_MOTOR_YAW_LIMIT;
            }
        }

        // voltage scaling. Since the limiting has already been performed, this should not boost any signals.
        // it ~can~ if the target voltage is higher than the battery voltage, which could lead to instability, so don't do that. 
        if((battery_voltage > 5.0) && (_voltage_target > 5.0))
        {
          static float factor = 0.79;
          
//#warning "this probably isn't doing the right thing at all since motor_out is offset 1000:2000"
//          factor = (factor * (1.0-_voltage_tc)) + (sqrt(_voltage_target / battery_voltage)*_voltage_tc);

          factor = (factor * (1.0-_voltage_tc)) + (_voltage_target / battery_voltage)*_voltage_tc;
          factor = constrain(factor, 0.79, 1.15);

          for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) 
          {
            if( motor_enabled[i] ) 
            {
              motor_out[i] = motor_out[i] * factor;
            }
          }
        }

        // throttle booster, experimental. Try to get motors to the desired rpm faster by using the full battery voltage
        if(_throttle_boost_kb != 0)
        {
          for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
              if( motor_enabled[i] ) {
                  motor_out[i] = motor_out[i] + constrain( ((motor_out[i] - _motor_v_estimate[i])*_throttle_boost_kb), -_throttle_boost_limit, _throttle_boost_limit);
                  _motor_v_estimate[i] = _motor_v_estimate[i] + (_throttle_boost_ka * (motor_out[i] - _motor_v_estimate[i]));
              }
          }
        }

        // adjust for throttle curve
        if( _throttle_curve_enabled ) {
            for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
                if( motor_enabled[i] ) {
                    motor_out[i] = _throttle_curve.get_y(motor_out[i]);
                }
            }
        }

        // clip motor output if required (shouldn't be)
        for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
            if( motor_enabled[i] ) {
#if ESC3D == ENABLED
                motor_out[i] = _rc_throttle->range_to_radio_out(motor_out[i]*r); // back to correct sign, then to radio offset/scale
#else
                motor_out[i] = constrain(motor_out[i], out_min, out_max);
#endif
            }
        }
    }

    // send output to each motor
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            _rc->OutputCh(_motor_to_channel_map[i], motor_out[i]);
        }
    }
}

// output_disarmed - sends commands to the motors
void AP_MotorsMatrix::output_disarmed()
{
    if(_rc_throttle->control_in != 0) {
        // we have pushed up the throttle
        // remove safety for auto pilot
        _auto_armed = true;
    }
    // Send minimum values to all motors
    output_min();
}

// output_disarmed - sends commands to the motors
void AP_MotorsMatrix::output_test()
{
    int8_t min_order, max_order;
    int8_t i,j;

    // find min and max orders
    min_order = test_order[0];
    max_order = test_order[0];
    for(i=1; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( test_order[i] < min_order )
            min_order = test_order[i];
        if( test_order[i] > max_order )
            max_order = test_order[i];
    }

    // shut down all motors
    output_min();

    // first delay is longer
    delay(4000);
    // loop through all the possible orders spinning any motors that match that description
    for( i=min_order; i<=max_order; i++ ) {
        for( j=0; j<AP_MOTORS_MAX_NUM_MOTORS; j++ ) {
            if( motor_enabled[j] && test_order[j] == i ) {
                // turn on this motor and wait 1/3rd of a second
#if ESC3D == ENABLED
                _rc->OutputCh(_motor_to_channel_map[j], _rc_throttle->radio_trim + 50);
                delay(300);
                _rc->OutputCh(_motor_to_channel_map[j], _rc_throttle->radio_trim);
                delay(2000);
#else
                _rc->OutputCh(_motor_to_channel_map[j], _rc_throttle->radio_min + 100);
                delay(300);
                _rc->OutputCh(_motor_to_channel_map[j], _rc_throttle->radio_min);
                delay(2000);
#endif
            }
        }
    }

    // shut down all motors
    output_min();
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float throt_fac, float roll_fac, float pitch_fac, float yaw_fac, int8_t testing_order)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // increment number of motors if this motor is being newly motor_enabled
        if( !motor_enabled[motor_num] ) {
            motor_enabled[motor_num] = true;
            _num_motors++;
        }

        // set roll, pitch, thottle factors and opposite motor (for stability patch)
        _throttle_factor[motor_num] = throt_fac;
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;

        // set order that motor appears in test
        if( testing_order == AP_MOTORS_MATRIX_ORDER_UNDEFINED ) {
            test_order[motor_num] = motor_num;
        }else{
            test_order[motor_num] = testing_order;
        }
    }
}

// add_motor using just position and prop direction
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, int8_t direction, int8_t testing_order)
{
    // call raw motor set-up method
    add_motor_raw(
        motor_num,
        1.0F,
        cos(radians(angle_degrees + 90)),               // roll factor
        cos(radians(angle_degrees)),                    // pitch factor
        (float)direction,                                               // yaw factor
        testing_order);

}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // if the motor was enabled decrement the number of motors
        if( motor_enabled[motor_num] )
            _num_motors--;

        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _throttle_factor[motor_num] = 0;
        _roll_factor[motor_num] = 0;
        _pitch_factor[motor_num] = 0;
        _yaw_factor[motor_num] = 0;
    }
}

// remove_all_motors - removes all motor definitions
void AP_MotorsMatrix::remove_all_motors()
{
    for( int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        remove_motor(i);
    }
    _num_motors = 0;
}