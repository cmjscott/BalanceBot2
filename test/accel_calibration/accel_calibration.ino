#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
   */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
//    Displays some basic information on this sensor from the unified
//    sensor API sensor_t type (see Adafruit_Sensor for more information)
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
   // Display some basic info about the sensor status
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
 //   Display sensor calibration status
/**************************************************************************/
bool displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);

    if((system == 3) && (gyro == 3) && (accel == 3) && (mag == 3))
    {
      return true;
    } else {
      return false;
    }
}

/**************************************************************************/
   // Display the raw calibration offset and radius data
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}


void setup(void)
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("x, y, z");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }


    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    calibrationData.accel_offset_x = 1;
    calibrationData.accel_offset_y = -18;
    calibrationData.accel_offset_z = -11;

    calibrationData.gyro_offset_x = 0;
    calibrationData.gyro_offset_y = 0;
    calibrationData.gyro_offset_z = -1;

    calibrationData.mag_offset_x = -141;
    calibrationData.mag_offset_y = -42;
    calibrationData.mag_offset_z = -279;

    calibrationData.accel_radius = 1000;
    calibrationData.mag_radius = 729;

    //Accelerometer: 1 -18 -11 
    //Gyro: 0 0 -1 
    //Mag: -141 -42 -279 
    //Accel Radius: 1000
    //Mag Radius: 729
    
    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    

    delay(1000);

    /* Display some basic information on this sensor */
    //displaySensorDetails();

    /* Optional: Display current status */
    //displaySensorStatus();

   /* Crystal must be configured AFTER loading calibration data into BNO055. */
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    bno.setSensorOffsets(calibrationData);

}

void loop() {
    /* Get a new sensor event */
    sensors_event_t orientation, angVelocity, linAccel, accel, grav;

    //bno.getEvent(&orientation);
    bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocity, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&grav, Adafruit_BNO055::VECTOR_GRAVITY);

    /* Display the floating point data */
    // Serial.print("X: ");
    // Serial.print(event.orientation.y, 4);
    // Serial.print("\tY: ");
    // Serial.print(event.orientation.z, 4);
    // Serial.print("\tZ: ");
    // Serial.print(event.orientation.x, 4);

    //Serial.print(orientation.orientation.x);
    //Serial.print(", ");

    // Serial.print(accel.acceleration.x);
    // Serial.print(", ");
    // Serial.print(accel.acceleration.y);
    // Serial.print(", ");
    // Serial.print(accel.acceleration.z);
    // Serial.println("");

    // Serial.print(grav.acceleration.x);
    // Serial.print(", ");
    // Serial.print(grav.acceleration.y);
    // Serial.print(", ");
    // Serial.print(grav.acceleration.z);
    // Serial.println("");
    
    // Serial.print(accel.acceleration.x - grav.acceleration.x);
    // Serial.print(", ");
    // Serial.print(accel.acceleration.y - grav.acceleration.y);
    // Serial.print(", ");
    // Serial.print(accel.acceleration.z - grav.acceleration.z);
    // Serial.println("");

//    Serial.print(linAccel.acceleration.x);
//    Serial.print(", ");
//    Serial.print(linAccel.acceleration.y);
//    Serial.print(", ");
//    Serial.print(linAccel.acceleration.z);
//    Serial.println("");

    /* Optional: Display calibration status
    if(displayCalStatus())
    {
      adafruit_bno055_offsets_t newCalib;
      bno.getSensorOffsets(newCalib);
      displaySensorOffsets(newCalib);
    }

    /* Optional: Display sensor status (debug only) */
    displaySensorStatus();

    /* New line for the next sample */
    Serial.println("");

    /* Wait the specified delay before requesting new data */
    //delay(BNO055_SAMPLERATE_DELAY_MS);
    delay(5);
}
