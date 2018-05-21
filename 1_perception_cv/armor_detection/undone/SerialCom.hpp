#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <ctime>
#include <time.h>
#include <sys/time.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <linux/types.h>
#include <linux/tty_flags.h>
#include <cstdint>

//FrameHeader(5-Byte) CmdID(2-Byte) Data(n-Byte) FrameTail(2-Byte, CRC16)

class DataSegment
{
protected:
  DataSegment(const uint16_t CmdID);
  static const uint16_t CmdID;
};

class GimbalCtlData
{
public:
  GimbalCtlData(float dx, float dy);
  static const uint16_t CmdID = 0x1;
  struct
  {
    float dx;
    float dy;
  } data;
};

class SerialCom
{
public:
  SerialCom(){};
  bool initialize(const char *portName, const speed_t baudRate);

private:
  const char *portName = NULL;
  speed_t baudRate;
  termios tty;
};