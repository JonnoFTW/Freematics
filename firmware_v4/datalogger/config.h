#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/

// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1

// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 1

#define ENABLE_DATA_CACHE 0
#define MAX_CACHE_SIZE 256  /* bytes */

#define USE_FRIENDLY_PID_NAME 0

// followings define the format of data streaming, enable one of them only
// FORMAT_BIN is required by Freematics OBD iOS App
//#define STREAM_FORMAT FORMAT_BIN
// FORMAT_TEXT is for readable text output
#define STREAM_FORMAT FORMAT_TEXT

// serial baudrate for data out streaming
#define STREAM_BAUDRATE 115200

// maximum size per file, a new file will be created on reaching this size
#define MAX_LOG_FILE_SIZE 1024 /* KB */

// outputs more debug information
#define VERBOSE 0

/**************************************
* Hardware setup
**************************************/

// max allowed time for connecting OBD-II (0 for forever)
#define OBD_ATTEMPT_TIME 0 /* seconds */

#define OBD_PROTOCOL PROTO_AUTO

// should be one of:
/*  PROTO_AUTO = 0,
    PROTO_ISO_9141_2 = 3,
    PROTO_KWP2000_5KBPS = 4,
    PROTO_KWP2000_FAST = 5,
    PROTO_CAN_11B_500K = 6,
    PROTO_CAN_29B_500K = 7,
    PROTO_CAN_29B_250K = 8,
    PROTO_CAN_11B_250K = 9,
 */

// OBD-II UART baudrate
#define OBD_UART_BAUDRATE 115200L

// SD pin
#define SD_CS_PIN 10

// enable(1)/disable(0) MEMS sensor
#define USE_MPU6050 0
#define ACC_DATA_RATIO 160
#define GYRO_DATA_RATIO 256
#define COMPASS_DATA_RATIO 8

// enable(1)/disable(0) GPS module
#define USE_GPS 1
#define LOG_GPS_NMEA_DATA 0
#define LOG_GPS_PARSED_DATA 1

// GPS parameters
#define GPS_SERIAL_BAUDRATE 115200L

#endif // CONFIG_H_INCLUDED
