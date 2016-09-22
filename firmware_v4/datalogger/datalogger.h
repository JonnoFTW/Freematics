/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Written by Stanley Huang <stanleyhuangyc@gmail.com>
* Visit http://freematics.com for more information
*************************************************************************/

#define PID_GPS_LATITUDE 0xA
#define PID_GPS_LONGITUDE 0xB
#define PID_GPS_ALTITUDE 0xC
#define PID_GPS_SPEED 0xD
#define PID_GPS_HEADING 0xE
#define PID_GPS_SAT_COUNT 0xF
#define PID_GPS_TIME 0x10
#define PID_GPS_DATE 0x11
#define PID_GPS_DATETIME 0x12

#define PID_ACC 0x20
#define PID_GYRO 0x21
#define PID_COMPASS 0x22
#define PID_MEMS_TEMP 0x23
#define PID_BATTERY_VOLTAGE 0x24

#define PID_DATA_SIZE 0x80

#if ENABLE_DATA_OUT

#if defined(RF_SERIAL)
#define SerialRF RF_SERIAL
#else
#define SerialRF Serial
#endif

#endif

#if ENABLE_DATA_LOG
static File sdfile;
static SdFat SD;
#endif

class CDataLogger {
public:
    CDataLogger():m_lastDataTime(0),dataTime(0),dataSize(0)
    {
#if ENABLE_DATA_CACHE
        cacheBytes = 0;
#endif
    }
    void initSender()
    {
#if ENABLE_DATA_OUT
        SerialRF.begin(STREAM_BAUDRATE);
#endif
    }
    byte genTimestamp(char* buf, bool absolute)
    {
      byte n;
      if (absolute || dataTime >= m_lastDataTime + 60000) {
        // absolute timestamp
        n = sprintf(buf, "#%lu", dataTime);
      } else {
        // relative timestamp
        n += sprintf(buf, "%u", (unsigned int)(dataTime - m_lastDataTime));
      }
      buf[n++] = ',';      
      return n;
    }
    void record(const char* buf, byte len)
    {
#if ENABLE_DATA_LOG
        char tmp[36];
        byte n = genTimestamp(tmp, dataSize == 0);
        dataSize += sdfile.write(tmp, n);
        dataSize += sdfile.write(buf, len);
        dataSize += sdfile.println();
#endif
        m_lastDataTime = dataTime;
#if ENABLE_DATA_OUT
        SerialRF.write(tmp, n);
        SerialRF.write(buf, len);
        SerialRF.println("!");
//        SerialRF.println(dataSize);
#endif
    }
    void dispatch(const char* buf, byte len)
    {
      return;
      /*
#if ENABLE_DATA_CACHE
        if (cacheBytes + len < MAX_CACHE_SIZE - 10) {
          cacheBytes += genTimestamp(cache + cacheBytes, cacheBytes == 0);
          memcpy(cache + cacheBytes, buf, len);
          cacheBytes += len;
          cache[cacheBytes++] = ' ';
          cache[cacheBytes] = 0;
        }
#else
        char tmp[12];
        byte n = genTimestamp(tmp, dataTime >= m_lastDataTime + 20);
        SerialRF.write(tmp, n);
#endif
#if ENABLE_DATA_OUT
        SerialRF.write(buf, len);
        SerialRF.println();
#endif*/
    }
    void logDateTime(uint32_t date, uint32_t timev) {
        char time_str[8+6];
        sprintf(time_str, "%06lu%08lu", date, timev); // DDMMYYHHMMSSmm
        String s(time_str);
        // put them into a whole string
        char datetime_str [30];
        byte n = sprintf(datetime_str, "12,%s/%s/20%s %s:%s:%s.%s", s.substring(0,2).c_str(),
                                                 s.substring(2,4).c_str(), 
                                                 s.substring(4,6).c_str(), 
                                                 s.substring(6,8).c_str(),
                                                 s.substring(8,10).c_str(),
                                                 s.substring(10,12).c_str(),
                                                 s.substring(12,14).c_str());
        record(datetime_str, n);
        // should show: 09/09/2016 07:09:58.40
      
    }
    void logData(const char* buf, byte len)
    {
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid)
    {
        char buf[8];
        byte len = translatePIDName(pid, buf);
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int value)
    {
        char buf[16];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%d", value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%ld", value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, uint32_t value)
    {
        char buf[20];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%lu", value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%d,%d,%d", value1, value2, value3) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[24];
        byte len = translatePIDName(pid, buf);
        len += sprintf(buf + len, "%d.%06lu", (int)(value / 1000000), abs(value) % 1000000);
        dispatch(buf, len);
        record(buf, len);
    }
//    void showMem() {
//        SerialRF.print("freeMemory=");
//        SerialRF.print(freeMemory());
//        SerialRF.println("b");
//    }
#if ENABLE_DATA_LOG
    uint16_t openFile(uint32_t dateTime = 0)
 {
        uint16_t fileIndex = 1;
        char path[20] = "/DATA";
        dataSize = 0;
        bool exists_ = SD.exists("/DATA");
        if (exists_) {
            // use index number as file name
            for (; fileIndex; fileIndex++) {
                sprintf(path + 5, "/DAT%05u.CSV", fileIndex);
                if (!SD.exists(path)) {
                    break;
                }
            }           
        } else {
            bool check = SD.mkdir(path);
            if(SD.exists(path)) {
              fileIndex = 1;
              sprintf(path + 5, "/DAT%05u.CSV", 1);
            } else {
              return 0;
            }
        }
        sdfile = SD.open(path, FILE_WRITE);
        if (!sdfile) {
            SerialRF.println("could not open file");
            return 0;
        }
        return fileIndex;
 }
    void closeFile()
    {
        SerialRF.println(F("Closing file!"));
        sdfile.close();
        dataSize = 0;
    }
    void flushFile()
    {
        sdfile.flush();
        SerialRF.flush();
    }
#endif
    uint32_t dataTime;
    uint32_t dataSize;
#if ENABLE_DATA_CACHE
    char cache[MAX_CACHE_SIZE];
    int cacheBytes;
#endif
private:
    byte translatePIDName(uint16_t pid, char* text)
    {
#if USE_FRIENDLY_PID_NAME
        for (uint16_t n = 0; n < sizeof(pidNames) / sizeof(pidNames[0]); n++) {
            uint16_t id = pgm_read_byte(&pidNames[n].pid);
            if (pid == id) {
                memcpy_P(text, pidNames[n].name, 3);
                text[3] = ',';
                return 4;
            }
        }
#endif
        return sprintf(text, "%X,", pid);
    }
    uint32_t m_lastDataTime;
};
