/*************************************************************************
  OBD-II/MEMS/GPS Data Logging Sketch for Freematics ONE
  Distributed under BSD license
  Visit http://freematics.com/products/freematics-one for more information
  Developed by Stanley Huang <stanleyhuangyc@gmail.com>

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FreematicsONE.h>
#include "config.h"
#if ENABLE_DATA_LOG
#include <SD.h>
#endif
#include "datalogger.h"

// states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_FILE_READY 0x20
#define STATE_SLEEPING 0x40

#if VERBOSE && !ENABLE_DATA_OUT
#define SerialInfo Serial
#else
#define SerialInfo SerialRF
#endif

static uint8_t lastFileSize = 0;
static uint16_t fileIndex = 0;
static unsigned long GPS_LAST_TIME = 0;
static int VIN_LOGGED = 0;

uint16_t MMDD = 0;
uint32_t UTC = 0;

#if USE_MPU6050
MEMS_DATA mems;
#endif

class ONE : public COBDSPI, public CDataLogger
{
  public:
    ONE(): state(0) {}
    void setup()
    {
      state = 0;

      begin();
      SerialRF.print("Firmware Version ");
      SerialRF.println(version);

#if ENABLE_DATA_LOG
      SerialRF.print(millis());
      SerialRF.print(",SD,");
      uint16_t volsize = initSD();
      if (volsize) {
        SerialRF.print(volsize);
        SerialRF.println("MB!");
      } else {
        SerialRF.println("NO!");
      }
#endif

#if USE_MPU6050
      SerialRF.print(millis());
      SerialRF.print(",MEMS,");
      Wire.begin();
      if (memsInit()) {
        state |= STATE_MEMS_READY;
        SerialRF.println("OK!");
      } else {
        SerialRF.println("NO!");
      }
#endif

      connectOBD();


#if USE_GPS
      connectGPS();
#endif

      delay(1000);
    }
    bool connectOBD() {
      SerialRF.print(millis());
      SerialRF.print(",OBD,");
      if (init(OBD_PROTOCOL)) {
        state |= STATE_OBD_READY;
        SerialRF.println("OK!");
        return 1;
      } else {
        SerialRF.println("NO!");
        return 0;
      }
    }
    void connectGPS() {
      if (!GPS_LAST_TIME || millis() > GPS_LAST_TIME + 10000) {
        delay(100);
        GPS_LAST_TIME = millis();
        SerialRF.print(GPS_LAST_TIME);
        SerialRF.print(",GPS,");
        if (initGPS(GPS_SERIAL_BAUDRATE)) {
          state |= STATE_GPS_FOUND;
          SerialRF.println("OK!");
        } else {
          SerialRF.println("NO!");
        }
      }
    }
#if USE_GPS
    void logGPSData()
    {
#if LOG_GPS_NMEA_DATA
      // issue the command to get NMEA data (one line per request)
      char buf[255];
      byte n = getGPSRawData(buf, sizeof(buf));
      if (n) {
        dataTime = millis();
        // strip heading $GPS and ending \r\n
        logData(buf + 4, n - 6);
      }
#endif
#if LOG_GPS_PARSED_DATA
      // issue the command to get parsed GPS data
      GPS_DATA gd = {0};
      if (getGPSData(&gd)) {
        dataTime = millis();
        if (gd.time && gd.time != UTC) {
          byte day = gd.date / 10000;
          if (MMDD % 100 != day) {
            logData(PID_GPS_DATE, gd.date);
          }
          logData(PID_GPS_TIME, gd.time);
          logData(PID_GPS_LATITUDE, gd.lat);
          logData(PID_GPS_LONGITUDE, gd.lng);
          logData(PID_GPS_ALTITUDE, gd.alt);
          logData(PID_GPS_SPEED, gd.speed);
          logData(PID_GPS_SAT_COUNT, gd.sat);
          // save current date in MMDD format
          unsigned int DDMM = gd.date / 100;
          UTC = gd.time;
          MMDD = (DDMM % 100) * 100 + (DDMM / 100);
          // set GPS ready flag
          state |= STATE_GPS_READY;
        }
      }
#endif
    }
#endif
#if USE_MPU6050
    void logMEMSData()
    {
      // log the loaded MEMS data
      logData(PID_ACC, mems.value.x_accel / ACC_DATA_RATIO, mems.value.y_accel / ACC_DATA_RATIO, mems.value.z_accel / ACC_DATA_RATIO);
    }
#endif
#if ENABLE_DATA_LOG
    uint16_t initSD()
    {
      state &= ~STATE_SD_READY;
      pinMode(SS, OUTPUT);
      Sd2Card card;
      uint32_t volumesize = 0;
      if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
        SdVolume volume;
        if (volume.init(card)) {
          volumesize = volume.blocksPerCluster();
          volumesize >>= 1; // 512 bytes per block
          volumesize *= volume.clusterCount();
          volumesize /= 1000;
        }
      }
      if (SD.begin(SD_CS_PIN)) {
        state |= STATE_SD_READY;
        return volumesize;
      } else {
        return 0;
      }
    }
    void flushData()
    {
      // flush SD data every 1KB
      byte dataSizeKB = dataSize >> 10;
      if (dataSizeKB != lastFileSize) {
#if VERBOSE
        // display logged data size
        SerialInfo.print(dataSize);
        SerialInfo.println(" bytes");
#endif
        flushFile();
        lastFileSize = dataSizeKB;
#if MAX_LOG_FILE_SIZE
        if (dataSize >= 1024L * MAX_LOG_FILE_SIZE) {
          closeFile();
          state &= ~STATE_FILE_READY;
        }
#endif
      }
    }
#endif
    void reconnect()
    {
      SerialRF.println("Reconnecting");
      if (init()) {
        // reconnected
        return;
      }
      SerialRF.println("Sleeping");
#if ENABLE_DATA_LOG
      closeFile();
#endif
      state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_GPS_FOUND);
      // cut off GPS power
      initGPS(0);
      // check if OBD is accessible again
      for (;;) {
        int value;
        if (read(PID_RPM, value))
          break;
        sleep(2);
      }
      // reset device
      void(* resetFunc) (void) = 0; //declare reset function at address 0
      resetFunc();
    }
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
#if USE_MPU6050
      if (state & STATE_MEMS_READY) {
        memsRead(&mems);
      }
#endif
    }
    byte state;
};

static ONE one;

void setup()
{
#if VERBOSE
  SerialInfo.begin(STREAM_BAUDRATE);
#endif
  one.initSender();
  one.setup();

}
void loop()
{
#if ENABLE_DATA_LOG
  if (!(one.state & STATE_FILE_READY) && (one.state & STATE_SD_READY)) {
    if (one.state & STATE_GPS_FOUND) {
      // GPS connected
      if (one.state & STATE_GPS_READY) {
        uint32_t dateTime = (uint32_t)MMDD * 10000 + UTC / 10000;
        if (one.openFile(dateTime) != 0) {
          SerialRF.print("FILE,");
          SerialRF.println(dateTime);
          MMDD = 0;
          one.state |= STATE_FILE_READY;
        } else {
          SerialRF.println("File error");
        }
      }
    } else {
      // no GPS connected
      int index = one.openFile(0);
      if (one.openFile(0) != 0) {
        one.state |= STATE_FILE_READY;
        SerialRF.print("0,FILE,");
        SerialRF.print(index);
        SerialRF.println("!");
      } else {
        SerialRF.println("File error!");
      }
    }
  }
#endif
  if (one.state & STATE_OBD_READY) {
    if(VIN_LOGGED < 5) {
        int vin_bytes = 96;
        char vin[vin_bytes];
        if(one.getVIN(vin, sizeof(vin))) {
          char buff [vin_bytes+4];
          int bytes = sprintf(buff, "VIN,%s", vin);
          one.logData(buff, bytes);
          VIN_LOGGED = 10;
        } else {
          VIN_LOGGED++;
        }
        
    }
    static byte index2 = 0;
    // from OBD.h
    const byte pids[8] = {PID_ENGINE_LOAD, PID_THROTTLE, PID_INTAKE_TEMP, PID_ENGINE_FUEL_RATE,
                            PID_INTAKE_MAP,  PID_RPM,      PID_SPEED,       PID_MAF_FLOW };
    const int errCode = -255;
//    static int pidsIndex = 0;
    int values[sizeof pids];
    for(int i = 0; i < sizeof(values) / sizeof values[0]; i++) {
      values[i] = errCode;
    }
    // read multiple OBD-II PIDs
    one.read(pids, sizeof pids, values);
    one.dataTime = millis();
    for (byte n = 0; n < sizeof(pids); n++) {
//      if(values[n] != errCode)
        one.logData(pids[n] | 0x100U, values[n]);
    }
    SerialRF.flush();
    delay(30);
    
    
    
    //        static byte lastSec = 0;
    //        const byte pids2[] = {PID_COOLANT_TEMP, PID_ENGINE_FUEL_RATE, PID_DISTANCE};
    //        byte sec = (uint8_t)(millis() >> 10);
    //        if (sec != lastSec) {
    //          // goes in every other second
    //          int value;
    //          byte pid = pids2[index2 = (index2 + 1) % (sizeof(pids2))];
    //          // read single OBD-II PID
    //          if (one.isValidPID(pid) && one.read(pid, value)) {
    //            one.dataTime = millis();
    //            one.logData((uint16_t)pid | 0x100, value);
    //            lastSec = sec;
    //          }
    //        }
    if (one.errors >= 10) {
      one.reconnect();  
    }
  } else {
    if (!OBD_ATTEMPT_TIME || millis() < OBD_ATTEMPT_TIME * 1000) {
      if (!one.connectOBD()) {
           // one.state |= STATE_OBD_READY;
            one.dataIdleLoop();
        }
      }/* else {
        
      }*/
    
  }
  
#if USE_MPU6050
  if (one.state & STATE_MEMS_READY) {
    one.logMEMSData();
  }
#endif
#if USE_GPS
  if (one.state & STATE_GPS_FOUND) {
    one.logGPSData();
  } else {
    one.connectGPS();
  }
#endif
#if ENABLE_DATA_LOG
  one.flushData();
#endif
}
