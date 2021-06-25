/**
 * S T E W A R T    P L A T F O R M    O N    E S P 3 2
 *
 * Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

 #ifndef __CONFIG_H__
 #define __CONFIG_H__

 #include <Arduino.h>

#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

//#pragma once

/*
 * ======== SERVO SETTINGS ==========
 */

#define NB_SERVOS 6

/*
 * Servo calibration values.
 * These values should not be used to restrict servo movements.
 * Use MIN/MAX COORDINATES below for that.
 */
const double SERVO_FULL_ANGULAR_RANGE = radians(180);
const double SERVO_HALF_ANGULAR_RANGE = SERVO_FULL_ANGULAR_RANGE / 2;
const int SERVO_MIN_PWM = 500;
const int SERVO_MAX_PWM = 2500;

/*
 * Offset values in µs to compensate for arm angle errors.
 */
const int PW_OFFSET[] = {
    0,
    0,
    0,
    0,
    0,
    0};

/*
 * The gain in µs/rad
 * Typical gain =~ 318 µs/rad =~ 5.6 µs/°
 */
const double gain = (SERVO_MAX_PWM - SERVO_MIN_PWM) /
                    (SERVO_FULL_ANGULAR_RANGE);

/*
 * calibration_t
 */
typedef struct
{
    double gain;
    int offset;
} calibration_t;


typedef struct
{
    double hx_x; // Surge, translation along X axis (mm)
    double hx_y; // Sway, translation along Y axis (mm)
    double hx_z; // Heave, translation along Z axis (mm)
    double hx_a; // Roll, rotation around X axis (rad)
    double hx_b; // Pitch, rotation around Y axis (rad)
    double hx_c; // Yaw, rotation around Z axis (rad)
} platform_t;

/*
 * Calibration factors. These values take into account the fact
 * that the odd and even arms are a reflection of each other.
 * The calibration is linear:
 * pulse width (µs) = gain (µs/rad) + offset (µs)
 */
const calibration_t SERVO_CALIBRATION[NB_SERVOS] = {
    {-gain, SERVO_MAX_PWM + PW_OFFSET[0]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[1]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[2]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[3]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[4]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[5]}};

/*
 * Pin numbers for each servo signal.
 */
const int SERVO_PINS[] = {
    4,
    27,
    26,
    25,
    33,
    32};

/*
 * ======== GEOMETRY SETTINGS ==========
 */

/*
 * Orientations of the servos arms relative to the X axis.
 */
const double THETA_S[NB_SERVOS] = {
    radians(-60),
    radians(120),
    radians(180),
    radians(0),
    radians(60),
    radians(-120)};

/*
 * MIN/MAX COORDINATES
 * NOTE: The actual min and max for each DOF are interdependent. eg:
 * If the platform is pitched by some amount, the roll min/max will be physically
 * different than what's defined here. These are just the absolute maximums under
 * ideal conditions (eg: max for roll when pitch is zero).
 */
const double HX_X_MIN = -50;
const double HX_X_MAX = 50;
const double HX_X_MID = (HX_X_MAX + HX_X_MIN) / 2;
const double HX_X_BAND = HX_X_MAX - HX_X_MIN;

const double HX_Y_MIN = -50;
const double HX_Y_MAX = 50;
const double HX_Y_MID = (HX_Y_MAX + HX_Y_MIN) / 2;
const double HX_Y_BAND = HX_Y_MAX - HX_Y_MIN;

const double HX_Z_MIN = -10.0;
const double HX_Z_MAX = 18.0;
const double HX_Z_MID = (HX_Z_MAX + HX_Z_MIN) / 2;
const double HX_Z_BAND = HX_Z_MAX - HX_Z_MIN;

const double HX_A_MIN = radians(-15);
const double HX_A_MAX = radians(20);
const double HX_A_MID = (HX_A_MAX + HX_A_MIN) / 2;
const double HX_A_BAND = HX_A_MAX - HX_A_MIN;

const double HX_B_MIN = radians(-20);
const double HX_B_MAX = radians(15);
const double HX_B_MID = (HX_B_MAX + HX_B_MIN) / 2;
const double HX_B_BAND = HX_B_MAX - HX_B_MIN;

const double HX_C_MIN = radians(-55);
const double HX_C_MAX = radians(55);
const double HX_C_MID = (HX_C_MAX + HX_C_MIN) / 2;
const double HX_C_BAND = HX_C_MAX - HX_C_MIN;

const double THETA_P = radians(41.85); // Platform joint angle (radians) offset from AXIS[1|2|3]. A value of zero puts these joints directly on the axes.
const double THETA_B = radians(16.71); // Base Servo pinion angle (radians) offset from AXIS[1|2|3]. A value of zero puts the servo pinion directly on the axes.
const double P_RAD = 86.185;
//const double P_RAD = 84.69;          // Platform radius (mm). The distance from the center of the platform to the center of one platform / pushrod "joint". This should be the same for all six pushrods.
//const double B_RAD = 94.757; old, before 14mm standoffs added
const double B_RAD = 111.28;// Base radius (mm). Distance from the center of the base plate to the center of one servo pinion gear. Again, this should be the same for all six servos.
const double ARM_LENGTH = 25.0;          // Servo arm length (mm). Distance from the center of the servo pivot to the center of the pushrod pivot on the servo arm.
const double ROD_LENGTH = 180.0;         // Push rod length (mm). Distance between pushrod ball joints (servo to platform).
const double Z_HOME = -170; // Default Z height of the platform (above the base), with servo arms horizontal. Formally, the distance from the plane described by the collection of servo pinion gear centers, to the plane described by the collection of platform / pushrod joints. Must Be fine tuned manualy or computed with a numerical solver.
//const double Z_HOME = -187; // Default Z height of the platform (above the base), with servo arms horizontal. Formally, the distance from the plane described by the collection of servo pinion gear centers, to the plane described by the collection of platform / pushrod joints. Must Be fine tuned manualy or computed with a numerical solver.




// Kalman filter parameters
typedef struct {
    int n;
    double A;
} KF_Config;


/*
Kalman filter matrix coeficient definitions.
*/

// X axis filter parameters
const double AxxVal[4] = {1, 0.017,
                          0, 1};

//const double BxxVal[4] = {0.07883, 1.051,
//                          0, 0};

const double BxxVal[4] = {0, 1,
                          0, 0};

const double HxxVal[4] = {1.0, 0,
                          0, 1.0};

/*
Decent enough values. Revert to these if you fuck it up.
const double QxxVal[4] = {0.05, 0.01,
                        	0.01, 0.05};

const double RxxVal[4] = {0.08, 0.001,
                        	 0.001, 0.3};

*/

const double QxxVal[4] = {0.05, 0,
                        	0, 0.05};

const double RxxVal[4] = {0.01, 0,
                        	 0, 0.15};

const double PxxVal[4] = {1, 0,
		 							        0, 1,};

//const double KxxVal[4] = {0.6166, 0.2197, -0.8355, 0.0009};
//const double KxxVal[4] = {0.0639, 0.2016, -.2, 0.001};
//const double KxxVal[2] = {0.05, 0.04};

//const double KxxVal[2] = {0.07, 0.028}; // good gains for the smaller platform
//const double KxxVal[2] = {0.0012, 0.00038};
const double KxxVal[2] = {0.0012, 0.0005};

// Y axis filter parameters

const double AyyVal[4] = {1, 0.017,
                          0, 1};

const double ByyVal[4] = {0, 0,
                         0.07883, 1.051};

const double HyyVal[4] = {1.0, 0,
                          0, 1.0};

const double QyyVal[4] = {0.05, 0,
                        	 0, 0.05};

const double RyyVal[4] = {0.01, 0,
                        	 0, 0.15};

const double PyyVal[4] = {1, 0,
		 							        0, 1,};

const double XVal[2] = {0, 0};

//const double KyyVal[4] = {0.2531, 0.1696, 0.0412, -0.0225};
//const double KyyVal[2] = {0.04, 0.04}; // decent gains for integral control


//const double KyyVal[2] = {0.07, 0.028}; // good gains for the smaller platform
//const double KyyVal[2] = {0.0012, 0.0006};
const double KyyVal[2] = {0.0012, 0.0005};




// Platform X axis filter parameters
const double APxxVal[4] = {0.3131, 0.3232,
                          -0.3223, 0.3131};

const double BPxxVal[4] = {0.6869, -0.2723,
                          0.3223, 0.6869};

const double HPxxVal[4] = {1.0, 0,
                          0, 1.0};

const double QPxxVal[4] = {0.05, 0.0,
                        	0.0, 0.05};

const double RPxxVal[4] = {0.08, 0.0,
                        	 0.0, 0.3};

const double PPxxVal[4] = {1, 0,
		 							        0, 1,};


#endif //__CONFIG_H__
