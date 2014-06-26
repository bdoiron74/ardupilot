/*
 *       AP_MotorsQuad.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsQuad.h"

// setup_motors - configures the motors for a quad
void AP_MotorsQuad::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for supported frames
    if( _frame_orientation == AP_MOTORS_PLUS_FRAME ) {
        // plus frame set-up
        add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_MOTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_MOTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_MOTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_MOTOR_CW,  3);
    }else if( _frame_orientation == AP_MOTORS_V_FRAME ) {
        // V frame set-up        
        //add_motor(AP_MOTORS_MOT_1,   45,  0.8877,  1);
        //add_motor(AP_MOTORS_MOT_2, -135,  1.1123,  3);
        //add_motor(AP_MOTORS_MOT_3,  -45, -0.8877,  4);
        //add_motor(AP_MOTORS_MOT_4,  135, -1.1123,  2);

        // hack - using V_FRAME as V_TAIL (1=NE, 2=SE, 3=SW, 4=NW)
                                    // T,     R,    P,    Y    Test order
#if 0 // inverted rear motors
        add_motor_raw(AP_MOTORS_MOT_1, 1.0, -0.7,  0.7, -0.23,  1);
        add_motor_raw(AP_MOTORS_MOT_2, 1.0,   0.0, -0.7,  0.7,  2);
        add_motor_raw(AP_MOTORS_MOT_3, 1.0,   0.0, -0.7, -0.7,  3);
        add_motor_raw(AP_MOTORS_MOT_4, 1.0,  0.7,  0.7,  0.23,  4);
#else
        add_motor_raw(AP_MOTORS_MOT_1, 1.00, -0.7,  0.70,  0.35,  1);
        add_motor_raw(AP_MOTORS_MOT_2, 1.00,  0.0, -0.70, -0.70,  2);
        add_motor_raw(AP_MOTORS_MOT_3, 1.00,  0.0, -0.70,  0.70,  3);
        add_motor_raw(AP_MOTORS_MOT_4, 1.00,  0.7,  0.70, -0.35,  4);
#endif

    }else{
        // X frame set-up
#if 0
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_MOTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_MOTOR_CW,  2);
        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_MOTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_MOTOR_CW,  4);
#else
        add_motor_raw(AP_MOTORS_MOT_1, 1.00, -0.7,  0.70,  0.70,  1);
        add_motor_raw(AP_MOTORS_MOT_2, 1.00, -0.7, -0.70, -0.70,  2);
        add_motor_raw(AP_MOTORS_MOT_3, 1.00,  0.7, -0.70,  0.70,  3);
        add_motor_raw(AP_MOTORS_MOT_4, 1.00,  0.7,  0.70, -0.70,  4);
#endif

    }
}
