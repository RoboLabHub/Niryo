#include <DynamixelSDK.h>
#include <stdarg.h>

// Protocol version
#define PROTOCOL_VERSION2               2.0

// Default setting
#define DEVICENAME                      "1"                 // Default to external (OpenCM485 expansion) on OpenCM9.04

// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define CMD_SERIAL                      Serial              // USB Serial

typedef union
{
  uint8_t  u8Data[4];
  uint16_t u16Data[2];
  uint32_t u32Data;

  int8_t   s8Data[4];
  int16_t  s16Data[2];
  int32_t  s32Data;
} dxl_ret_t;

char *dev_name = (char*)DEVICENAME;

// Initialize Packethandler2 instance
dynamixel::PacketHandler *packetHandler2;
dynamixel::PortHandler   *portHandler;

int m_id   = -1;
int m_baud = -1;

void write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  //flushCmd();

  if (length == 1)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (uint8_t)value, &dxl_error);
  else if (length == 2)
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (uint16_t)value, &dxl_error);
  else if (length == 4)
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, (uint32_t)value, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS) 
  {
    if (dxl_error != 0) 
    {
      CMD_SERIAL.printf("Fail to write addr %d [err1]: ", addr);
      CMD_SERIAL.println(packetHandler->getRxPacketError(dxl_error));
    }
  }
  else
  {
    CMD_SERIAL.printf("Fail to write addr %d [err2]: ", addr);
    CMD_SERIAL.println(packetHandler->getTxRxResult(dxl_error));
  }

  delay(100);
}

dxl_ret_t read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int     dxl_comm_result = COMM_TX_FAIL;
  dxl_ret_t ret;

  int8_t  value8  = 0;
  int16_t value16 = 0;
  int32_t value32 = 0;

  if (length == 1)
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&value8, &dxl_error);
  else if (length == 2)
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&value16, &dxl_error);
  else if (length == 4)
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&value32, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) CMD_SERIAL.println(packetHandler->getRxPacketError(dxl_error));

    if (length == 1)
      ret.u32Data = value8;
    else if (length == 2)
      ret.u32Data = value16;
    else if (length == 4)
      ret.u32Data = value32;
  }
  else
  {
    CMD_SERIAL.println(packetHandler->getTxRxResult(dxl_error));
    CMD_SERIAL.printf("Fail to read addr %d!\n", addr);
  }

  return ret;
}

uint8_t GetChar()
{
    CMD_SERIAL.println(">>");
    
    while (true)
    {
        if (CMD_SERIAL.available())
          return CMD_SERIAL.read();
    }

    return 0;
}

void flushCmd(void)
{
  while (CMD_SERIAL.available())
  {
    CMD_SERIAL.read();
  }
}

void drawTitle(void)
{
  flushCmd();
  
  CMD_SERIAL.println(" ");
  CMD_SERIAL.println(" ");
  CMD_SERIAL.println("1. setup 1st motor");
  CMD_SERIAL.println("2. setup 2nd motor");
  CMD_SERIAL.println("3. setup 3rd motor");
  CMD_SERIAL.println("4. setup 4th motor");
  CMD_SERIAL.println("5. test  1st motor");
  CMD_SERIAL.println("6. test  2nd motor");
  CMD_SERIAL.println("7. test  3rd motor");
  CMD_SERIAL.println("8. test  4th motor");
}

void setup()
{
  CMD_SERIAL.begin(57600);
  while (!CMD_SERIAL);

  packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
  portHandler    = dynamixel::PortHandler::getPortHandler(dev_name);

  // Open port
  if (portHandler->openPort())
  {
    CMD_SERIAL.println("Succeeded to open the port!");
    CMD_SERIAL.printf(" - Device Name : %s\r\n", dev_name);
    CMD_SERIAL.printf(" - Baudrate    : %d\r\n", portHandler->getBaudRate());
    m_baud = portHandler->getBaudRate();
  }
  else
  {
    CMD_SERIAL.printf("Failed to open the port! [%s]\n", dev_name);
    CMD_SERIAL.printf("Press any key to terminate...\n");
    while (1);
  }

  CMD_SERIAL.println("\r\nStart setup motor");
  drawTitle();
}

bool requestConfirm(void)
{
  CMD_SERIAL.print("Do you really want to setup ? y/n : ");

  while (1)
  {
    if (CMD_SERIAL.available())
    {
      uint8_t ch = CMD_SERIAL.read();

      if (ch == 'y' || ch == 'Y')
      {
        CMD_SERIAL.println("yes");
        flushCmd();
        return true;
      }

      break;
    }
  }

  CMD_SERIAL.println("no");
  return false;
}

bool findServo(int id)
{
  uint32_t baud_tbl[2] = { 57600, 1000000 };
#define COUNT_BAUD (sizeof(baud_tbl)/sizeof(baud_tbl[0]))
  uint32_t index;
  uint32_t baud_pre;

  std::vector<unsigned char> vec;

  baud_pre = portHandler->getBaudRate();

  CMD_SERIAL.printf("Find Motor %d ... ", id);

  m_id = -1;
  
  // First try to find the specific servo ID wanted
  for (index = 0; index < COUNT_BAUD; index++)
  {
    portHandler->setBaudRate(baud_tbl[index]);
    uint16_t model_number;
    int dxl_comm_result = packetHandler2->ping(portHandler, id, &model_number);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      if (m_id == -1)
      {
        m_id = id;
        m_baud =  baud_tbl[index];
      }
      else
      {
        CMD_SERIAL.printf("Warning Servo %d found at two baud rates %d and %d using %d\n",
                          id, m_baud, baud_tbl[index], baud_tbl[index]);
        m_baud = baud_tbl[index];
      }
    }
  }

  if (m_id != -1)
  {
    CMD_SERIAL.println("    ... SUCCESS");
    CMD_SERIAL.printf("    [ID: %d found at baud: %d]\n", m_id, m_baud);
    portHandler->setBaudRate(m_baud);
    return true;
  }

  // Did not find the actual ID we were looking for so see if we find any servos?
  for (index = 0; index < COUNT_BAUD; index++)
  {
    CMD_SERIAL.printf("    setbaud : %d\r\n", baud_tbl[index]);

    portHandler->setBaudRate(baud_tbl[index]);
    m_baud = baud_tbl[index];

    // Lets see if we find the actual one we want to update?
    int dxl_comm_result = packetHandler2->broadcastPing(portHandler, vec);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      CMD_SERIAL.println(packetHandler2->getTxRxResult(dxl_comm_result));
      continue;
    }

    for (unsigned int i = 0; i < vec.size(); i++)
    {
      CMD_SERIAL.println("    ... SUCCESS");
      CMD_SERIAL.println("    [ID:" + String(vec.at(i)) + "]");
      m_id = vec.at(i);
    }

    if (vec.size() > 0)
    {
      if (vec.size() > 1)
      {
        flushCmd();
        CMD_SERIAL.printf("    WARNING: multiple servos found, choose one of found id >> \n");
        m_id = GetChar() - '0';
      }
      break;
    }
    else
      CMD_SERIAL.println("not found");
  }

  if (m_id < 0)
  {
    portHandler->setBaudRate(baud_pre);
    return false;
  }

  CMD_SERIAL.printf("Using found id: %d\n", m_id);

  return true;
}

void setupXL430(int id)
{
  CMD_SERIAL.printf("Setup XL430 with id: %d ...\n", id);

  if (!findServo(id))
    return;

  write(portHandler, packetHandler2, m_id, 64, 1, 0);   // Torque off
  write(portHandler, packetHandler2, m_id, 7, 1, id);   // Set ID
  write(portHandler, packetHandler2, id, 12, 1, 255); // Deactivate shadow ID

  write(portHandler, packetHandler2, id, 8, 1, 3);    // Baud rate 1M
  portHandler->setBaudRate(1000000);
  write(portHandler, packetHandler2, id, 10, 1, 0);   // Drive mode  0
  write(portHandler, packetHandler2, id, 11, 1, 3);   // Operating mode (position)
  CMD_SERIAL.println("OK");
}

void setupXL320(int id)
{ 
  CMD_SERIAL.printf("Setup XL320 with id: %d ...\n", id);

  if (!findServo(id))
    return;

  write(portHandler, packetHandler2, m_id, 24, 1, 0);   // Torque off
  write(portHandler, packetHandler2, m_id, 3, 1, id);   // Set ID

  write(portHandler, packetHandler2, id, 4, 1, 3);    // Baud rate 1M
  portHandler->setBaudRate(1000000);
  write(portHandler, packetHandler2, id, 11, 1, 2);     // Operating mode (joint)
  write(portHandler, packetHandler2, id, 15, 2, 1023);  // Reset max torque
  write(portHandler, packetHandler2, id, 5, 1, 250);    // Reset Delay timeout
  write(portHandler, packetHandler2, id, 17, 1, 2);     // Reset Return Level
  write(portHandler, packetHandler2, id, 18, 1, 3);     // Set default shutdown protection

  dxl_ret_t ret = read(portHandler, packetHandler2, id, 50, 1);
  CMD_SERIAL.printf("HW err=%ld\n", ret.u32Data);

  ret = read(portHandler, packetHandler2, id, 18, 1);
  CMD_SERIAL.printf("Alarm err=%ld\n", ret.u32Data);
  
  CMD_SERIAL.println("OK");
}

void testXL430(int id)
{
  CMD_SERIAL.printf("Test XL430 %d... ", id);
  if (!findServo(id))
    return;

  uint16_t model_number;
  int dxl_comm_result = packetHandler2->ping(portHandler, id, &model_number);
  if (dxl_comm_result != COMM_SUCCESS)
  {
     CMD_SERIAL.printf("not found\n", id);
     return;
  }
  
  CMD_SERIAL.printf(" found type: %d\n", model_number);
  write(portHandler, packetHandler2, id, 11, 1, 3);   // Mode pos
  
  write(portHandler, packetHandler2, id, 64, 1, 1);   // Torque on

  write(portHandler, packetHandler2, id, 116, 4, 1024); // Move to 1024
  delay(1000);
  write(portHandler, packetHandler2, id, 116, 4, 0);
  delay(1000);
  
  write(portHandler, packetHandler2, id, 64, 1, 0);   // Torque off
}

void testXL320(int id)
{
  CMD_SERIAL.printf("Test XL320 %d... ", id);
  if (!findServo(id))
    return;

  uint16_t model_number;
  int dxl_comm_result = packetHandler2->ping(portHandler, id, &model_number);
  if (dxl_comm_result != COMM_SUCCESS)
  {
     CMD_SERIAL.printf("not found\n", id);
     return;
  }
  
  CMD_SERIAL.printf(" found type: %d\n", model_number);
  write(portHandler, packetHandler2, id, 11, 1, 2);   // Mode joint

  write(portHandler, packetHandler2, id, 6, 2, 0);
  write(portHandler, packetHandler2, id, 8, 2, 1023);
  write(portHandler, packetHandler2, id, 15, 2, 1023);

  //write(portHandler, packetHandler2, id, 12, 1, 65);

  //write(portHandler, packetHandler2, id, 13, 1, 60);
  //write(portHandler, packetHandler2, id, 14, 1, 90);

  write(portHandler, packetHandler2, id, 35, 2, 1023);   // Torque limit (set to max)
  write(portHandler, packetHandler2, id, 51, 2, 1023);   // Min motor current

  //write(portHandler, packetHandler2, id, 32, 2, 256);   // Moving speed

  write(portHandler, packetHandler2, id, 24, 1, 1);   // Torque on

  write(portHandler, packetHandler2, id, 30, 2, 128); //1023);  // Goal pos 1023
  delay(1000);
  
  write(portHandler, packetHandler2, id, 30, 2, 0);
  delay(1000);
  
  write(portHandler, packetHandler2, id, 24, 1, 0);   // Torque off

  dxl_ret_t ret = read(portHandler, packetHandler2, id, 18, 1);
  CMD_SERIAL.printf("Alarm err=%ld\n", ret.u32Data);  
}

void loop()
{
  uint8_t ch = GetChar();
  flushCmd();

  if (ch == '1') setupXL430(1);
  if (ch == '2') setupXL430(2);
  if (ch == '3') setupXL320(3);
  if (ch == '4') setupXL320(4);
  
  if (ch == '5') testXL430(1);
  if (ch == '6') testXL430(2);
  if (ch == '7') testXL320(3);
  if (ch == '8') testXL320(4);

  drawTitle();
}

