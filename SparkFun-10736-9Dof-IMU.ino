
#include "config.h"
#include <Wire.h>


// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false


/*******************************************************************
** Globals *********************************************************
********************************************************************/

/* Serial communication globals */
//static bool g_BaudLock = false; /* Used to set baud rate */
static bool g_BaudLock = true; /* Used to set baud rate */

/* LED state globals */
static bool g_LedState = false; /* Used to set LED state */
uint32_t    g_LastBlinkTime = 0;   /* Used to set LED state */

/* DCM variables */
float MAG_Heading;
float Accel_Vector[3]        = {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]         = {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]        = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]             = {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]             = {0, 0, 0}; // Omega Integrator
float Omega[3]               = {0, 0, 0};
float errorRollPitch[3]      = {0, 0, 0};
float errorYaw[3]            = {0, 0, 0};
float DCM_Matrix[3][3]       = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3]    = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

float yaw   = -1.0;
float pitch =  2.5;
float roll  = -3.25;
float mag[]   = {0,0,0};
float accel[] = {0,0,0};
float gyro[]  = {0,0,0};

/* DCM timing in the main loop */
unsigned long timestamp     = 0;
unsigned long timestamp_old = 0;
float G_Dt = 0; // Integration time for DCM algorithm
float mydt = 0;
int count  = 0;




void setup()
{
  f_InitHardware();
  if ( !initIMU() ) 
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1) ; // Loop forever if we fail to connect
  }
  LOG_PORT.println("> IMU Initialized");
  delay(20);
  
  read_sensors();
  Reset_Sensor_Fusion();
}

// Main loop
void loop()
{
  while( (millis() - timestamp) < 20 ) {}
  
  // Update sensor readings
  read_sensors();
  f_UpdateTime();
  Matrix_Update();
  Normalize();
  Drift_Correction();
  Euler_Angles();

  /* Blink LED 
  ** TO DO: It would be nice to have a blink code
  **        to communicate during operation */
  f_BlinkLED();
}






