#pragma once

#include <Arduino.h>
// #include <math.h>
// #include <memory>

// #include "SSC32.h"
// #include "RobotController.h"
// #include "TouchScreen.h"
// #include "Controller.h"
// #include "PID.h"
// #include "Hexapod_Kinematics.h"
// #include "Hexapod_Config_1.h"
// #include "SensorPack.h"
// #include "kalmanfilter.h"
// #include "Logger.h"
/// Defines

/// pin definitions for arduino library
constexpr auto servo_0_pin = A0;
constexpr auto servo_1_pin = A1;
constexpr auto servo_2_pin = A2;
constexpr auto servo_3_pin = A3;
constexpr auto servo_4_pin = A6;
constexpr auto servo_5_pin = A7;

/// Baud definitions
constexpr int BPS = 115200;

/// SSC 32 servo configurations
///ssc[0].config(500, 2500, 0, 180, -60, true); 
///void config(int min, int max, float deg_min, float deg_max, int offset, bool cfg_CCW);
/// universal servo configuration values
constexpr int servo_min_duty = 500; ///< duty cycle corrisponding to minimum degree position
constexpr int servo_max_duty = 2500; ///< duty cycle corrisponding to maximum degree position 
constexpr int servo_min_degrees = 0; ///< minimum allowable degree command
constexpr int servo_max_degrees = 180; ///< maximum allowable degree command
constexpr bool servo_CCW_positive = true; ///< defines servo with CCW direction as positive
constexpr bool servo_CW_positive = false; ///< defines servo with CW direction as positive

/// unique servo configuration values
constexpr int servo_0_offset = -60;
constexpr int servo_1_offset = 70;
constexpr int servo_2_offset = -20;
constexpr int servo_3_offset = 70;
constexpr int servo_4_offset = -70;
constexpr int servo_5_offset = 100;
/// Defines

// main.h