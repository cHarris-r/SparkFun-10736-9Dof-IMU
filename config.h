/* Serial Port Config */
#define DEBUG 1
//#define LOG_PORT if(DEBUG)Serial
//#define LOG_PORT if(DEBUG)SERIAL_PORT_USBVIRTUAL
#define LOG_PORT Serial
#define COMM_PORT Serial

/* Define the output */
#define OUTPUT__BAUD_RATE 115200
#define UART_BLINK_RATE 1000

/* Define the hardware */
#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)


/* DCM parameters
******************************************************************/
// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM gain
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define HW_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi



/* Sensor I2C 
******************************************************************/

/* Accelerometer addresses */
#define ACCEL_ADDRESS ((int16_t) 0x53) // 0x53 = 0xA6 / 2
#define ACCEL_POWER    0x2D
#define ACCEL_MEASURE  0x08
#define ACCEL_FORMAT   0x31
#define ACCEL_RESOLT   0x08
#define ACCEL_RATE     0x2C
#define ACCEL_READ     0x32

/* Magnometer addresses */
#define MAGN_ADDRESS  ((int16_t) 0x1E) // 0x1E = 0x3C / 2
#define MAGN_READ      0x03

/* Gyroscope addresses */
#define GYRO_ADDRESS  ((int16_t) 0x68) // 0x68 = 0xD0 / 2

/* I2C Macros */
#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 



/* SENSOR CALIBRATION
*****************************************************************
** How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
** Put MIN/MAX and OFFSET readings for your board here!
** Accelerometer
** "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX" 
*/
#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)
// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_GAIN (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_GAIN (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_GAIN (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))
#define ACCEL_X_SCALED(x) (x * ACCEL_X_GAIN) 
#define ACCEL_Y_SCALED(x) (x * ACCEL_Y_GAIN) 
#define ACCEL_Z_SCALED(x) (x * ACCEL_Z_GAIN) 

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)
// Sensor calibration scale and offset values
#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

/*******************************************************************
** Tyedefs *********************************************************
********************************************************************/
/* The RESPONSE_TYPE is used 
** to store temporary resonse data
** for responding to request from master */
typedef struct{
  uint16_t Packet_nBytes;  /* Length of entire packet, minus this variable, in bytes */
  uint16_t PacketType;     /* Type code of packet */
  uint16_t Buffer_nBytes;  /* Length of data buffer in bytes (0-50) */
  unsigned char  Buffer[50];     /* Data buffer */
  unsigned char  CheckSum;       /* CheckSum of data buffer only */
} RESPONSE_TYPE;








