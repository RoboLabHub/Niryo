#include <variant.h>
#include <DynamixelSDK.h>

const uint8_t kMode_Velocity          = 1;
const uint8_t kMode_Joint             = 2;  // XL320
const uint8_t kMode_Position          = 3;
const uint8_t kMode_ExtendedPosition  = 4;
const uint8_t kMode_PWM               = 16;

class DMXDriver
{
public:
    virtual ~DMXDriver() 
    {
        close();
    }

    void LogError(const char* descr, const char* err)
    {
        LogInfo(descr);
        LogInfo(err);
        LogInfo("\n");
    }
  
    bool init()
    {
        m_portHandler   = dynamixel::PortHandler::getPortHandler("1");      // Device name
        m_packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);  // Protocol version

        // Open port
        if (m_portHandler->openPort() == false)
        {
            LogInfo1("Failed to open port(Motor Driver)\n");
            return false;
        }

        // Set port baudrate
        if (m_portHandler->setBaudRate(1000000) == false)
        {
            LogInfo1("Failed to set baud rate(Motor Driver)\n");
            return false;
        }

        LogInfo1("Motors initialized.\n");
        return true;      
    }
  
    void close(void)
    {
        // Close port
        m_portHandler->closePort();
    }

    bool write_byte(uint8_t id, uint16_t addr, uint8_t value)
    {
        uint8_t dxl_error = 0;
        int8_t dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, id, addr, value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            LogError("write1ByteTxRx(1): ", m_packetHandler->getTxRxResult(dxl_error));
            return false;
        }
        else if(dxl_error != 0)
        {
            LogError("write1ByteTxRx(2): ", m_packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        return true;
    }
    
    bool write_word(uint8_t id, uint16_t addr, uint16_t value)
    {
        uint8_t dxl_error = 0;
        int8_t dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, addr, value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            LogError("write2ByteTxRx(1): ", m_packetHandler->getTxRxResult(dxl_error));
            return false;
        }
        else if(dxl_error != 0)
        {
            LogError("write2ByteTxRx(2): ", m_packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        return true;        
    }
    
    bool write_dword(uint8_t id, uint16_t addr, uint32_t value)
    {
        uint8_t dxl_error = 0;
        int8_t dxl_comm_result = m_packetHandler->write4ByteTxRx(m_portHandler, id, addr, value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            LogError("write4ByteTxRx(1): ", m_packetHandler->getTxRxResult(dxl_error));
            return false;
        }
        else if(dxl_error != 0)
        {
            LogError("write4ByteTxRx(2): ", m_packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        return true;
    }

    bool read_byte(uint8_t id, uint16_t addr, uint8_t& value)
    {
        uint8_t dxl_error = 0;
        int8_t dxl_comm_result = m_packetHandler->read1ByteTxRx(m_portHandler, id, addr, &value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            LogError("read1ByteTxRx(1): ", m_packetHandler->getTxRxResult(dxl_error));
            return false;
        }
        else if(dxl_error != 0)
        {
            LogError("read1ByteTxRx(2): ", m_packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        return true;
    }

    bool read_word(uint8_t id, uint16_t addr, uint16_t& value)
    {
        uint8_t dxl_error = 0;
        int8_t dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, addr, &value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            LogError("read2ByteTxRx(1): ", m_packetHandler->getTxRxResult(dxl_error));
            return false;
        }
        else if(dxl_error != 0)
        {
            LogError("read2ByteTxRx(2): ", m_packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        return true;
    }

    bool read_dword(uint8_t id, uint16_t addr, uint32_t& value)
    {
        uint8_t dxl_error = 0;
        int8_t dxl_comm_result = m_packetHandler->read4ByteTxRx(m_portHandler, id, addr, &value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            LogError("read4ByteTxRx(1): ", m_packetHandler->getTxRxResult(dxl_error));
            return false;
        }
        else if(dxl_error != 0)
        {
            LogError("read4ByteTxRx(2): ", m_packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        return true;
    }

    bool setTorque(uint8_t id, bool torque, bool xl430 = true) 
    { 
        return write_byte(id, xl430 ? 64 : 24, torque ? 1 : 0); 
    }

    bool setMaxTorqueXL320(uint8_t id, bool maxTorque) 
    { 
        return write_dword(id, 35, maxTorque); 
    }

    bool setDriveMode(uint8_t id, uint8_t mode)
    {
        return write_byte(id, 10, mode);
    }
    
    bool setOperatingMode(uint8_t id, uint8_t mode)
    {
        return write_byte(id, 11, mode);
    }

    bool getEncoder(uint8_t id, int32_t& value)
    {
        return read_dword(id, 132, (uint32_t&)value);
    }

    bool getPositionXL320(uint8_t id, int32_t& value)
    {
        uint16_t val = 0;
        bool ret = read_word(id, 37, val);
        value = val;
        return ret;
    }
    
    bool setVelocity(uint8_t id, int32_t value)
    {
        return write_dword(id, 104, value);
    }
    
    bool setPosition(uint8_t id, int32_t value, bool xl430 = true)
    {
        if (xl430)
            return write_dword(id, 116, value);
        
        return write_word(id, 30, value);
    }
    
    bool setPWM(uint8_t id, int32_t  value)
    {
        return write_dword(id, 100, value);
    }

    bool getHWError(uint8_t id, uint8_t& value, bool xl430 = true)
    {
        return read_byte(id, xl430 ? 63 : 50, value);
    }

private:
    dynamixel::PortHandler*   m_portHandler;
    dynamixel::PacketHandler* m_packetHandler;
};

