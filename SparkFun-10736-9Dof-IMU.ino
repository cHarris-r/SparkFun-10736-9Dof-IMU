
#include "config.h"
#include <Wire.h>


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




/*******************************************************************
** START ***********************************************************
********************************************************************/

/*************************************************
** Setup Function 
** This function contains the setup functions 
** including the initialization of the hardware
** and the initialization of the serial ports
*/
void setup()
{
	LOG_PORT.begin(9600);
	
  Init_Hardware();
  if ( !Init_IMU() ) 
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1) ; // Loop forever if we fail to connect
  }
  LOG_PORT.println("> IMU Initialized");
  delay(20);
}

// Main loop
void loop()
{ 
  /* Update sensor readings */
  Read_Sensors();
	
	/* Apply the DCM Filter */
	DCM_Filter();

  /* Blink LED 
  ** TO DO: It would be nice to have a blink code
  **        to communicate during operation */
  Blink_LED();
}






