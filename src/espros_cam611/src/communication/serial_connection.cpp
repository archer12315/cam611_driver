#include <ros/ros.h>
#include "serial_connection.h"
#include "communication_constants.h"
#include "crc_calc.h"
#include "util.h"

using namespace std;

namespace com_lib
{

SerialConnection::SerialConnection()
{  
  expectedSize = 0;
  fileDescription = 0;
  //connect(this, &QSerialPort::readyRead, this, &SerialConnection::readRxData); //TODO...
}

SerialConnection::~SerialConnection()
{
  deviceListString.clear();  
  rxArray.clear();
}

/**
 * @brief List the available devices
 *
 * This function is used to list the available devices. The index of a device in the list can later be
 * used to select the port to open. The strings can for example be directly put into a comboBox in the GUI and
 * the index is directly given by the comboBox.
 *
 * @return List of strings containing the names of the available devices
 */
vector<string> SerialConnection::availableDevices()
{
  deviceListString.clear();  

  //const auto infos = QSerialPortInfo::availablePorts(); //TODO...
  //for (const QSerialPortInfo &info : infos) //TODO...
  //{
      string str = "COM1";
      //str.append("/dev/ttyUSB0");
      //str.append("COM1");
      deviceListString.push_back(str);
      //deviceListString.append(info.portName());
  //}

  return deviceListString;
}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool SerialConnection::openPort(std::string &portName, const unsigned int id, const unsigned int baudrate)
{
  if(fileDescription > 0) closePort();

  //fileDescription = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
  fileDescription = open(portName.data(), O_RDWR | O_NOCTTY | O_SYNC);

  if(fileDescription <=0)
  {
    ROS_INFO ("error %d opening %s: %s", errno, "/dev/ttyUSB0", strerror (errno));
    return false;
  }

  set_interface_attribs (B921600, 0);  // set speed to 921600 bps, 8n1 (no parity)
  set_blocking (1);                    // set no blocking

  rxArray.clear();

  return true;
}

/**
 * @brief Close the port
 *
 */
void SerialConnection::closePort()
{
  fileDescription = close(fileDescription);  
}

/**
 * @brief Check, if the checksum is correct
 *
 * This function checks the CRC. It extracts the CRC from the received data and calculates the CRC of the received
 * data and then compares them.
 *
 * @param array Pointer to the received data
 * @param expectedSize Expected size of the received data
 * @return Checksum correct or not
 */
bool SerialConnection::checksumIsCorrect(const vector<uint8_t> &array, const unsigned int expectedSize)
{
  //The received CRC is the one in the data
  uint32_t receivedCrc = Util::getUint32LittleEndian(array, (CommunicationConstants::Data::SIZE_HEADER + expectedSize));

  //The wanted CRC is the one calculated out of the payload
  uint32_t wantedCrc = CrcCalc::calcCrc32((uint8_t *)array.data(), (CommunicationConstants::Data::SIZE_HEADER + expectedSize));

  if (receivedCrc == wantedCrc)  
    return true;

  return false;
}

/**
 * @brief Send a command
 *
 * This function sends a command. It adds the marking and the checksum and sends it
 * to the device.
 *
 * Important: The size of the buffer must be big enough to add the markings an the checksum.
 *
 * @param data Pointer to the data to send
 */
ssize_t SerialConnection::sendData(uint8_t *data)
{
    if(fileDescription <=0 ){
      ROS_INFO("SerialConnection::sendData fileDescription =0 \n");
      return 0;
    }

    //Add the start buffer at the beginning
    data[0] = CommunicationConstants::Command::START_MARK;

    //Calculate the CRC
    uint32_t crc = CrcCalc::calcCrc32(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)));

    //Add it to the buffer123
    Util::setUint32LittleEndian(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)), crc);

    //This is just to print out the data
    ROS_DEBUG("SEND DATA: ");
    for(int i=0; i<14; i++)
        ROS_DEBUG("%x ", data[i]);

    return write(fileDescription, (uint8_t *)(data), CommunicationConstants::Command::SIZE_TOTAL);
}

/**
 * @brief Extract the expected size from the received data
 *
 * @param array Pointer to the received data
 * @return Expected size
 */
int SerialConnection::getExpextedSize(const std::vector<uint8_t> &array)
{
  int expectedSize = Util::getUint16LittleEndian(array, CommunicationConstants::Data::INDEX_LENGTH);
  return expectedSize;
}

/**
 * @brief Extract the type from the received data
 *
 * @param array Pointer to the received data
 * @return Received type
 */
uint8_t SerialConnection::getType(const vector<uint8_t> &array)
{
  return array.at(CommunicationConstants::Data::INDEX_TYPE);
}

/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
int SerialConnection::readRxData(int size)
{
  //Append the new data to the rx buffer 
  uint8_t buf[2048];
  int n = read(fileDescription, buf, size);

  if(n== -1){
    ROS_ERROR("Error on  SerialConnection::readRxData: %d ", n);
    return -1;
  }else {
    ROS_DEBUG("SerialConnection::readRxData: %d ", n);
  }

  rxArray.insert(std::end(rxArray), buf, buf + n); //Append the new data to the rxArray buffer

  //for(int i=0; i<n; i++)
  //  rxArray.push_back(buf[i]);

  processData(rxArray);

  return n;
}

/**
 * @brief Process the received data
 *
 * @param array Pointer to the received byte array
 */
bool SerialConnection::processData(vector<uint8_t> &array)
{
  if (array.size() == 0)
    return false;

  //Check for the marking byte
  if(array.at(0) != CommunicationConstants::Data::START_MARK)
    return true; //--------------------------------->

  //Cancel here if no marking bytes

  //Get the expected size. Cancel here if not enough bytes received
  if (array.size() < (static_cast<int>(CommunicationConstants::Data::SIZE_HEADER)))
    return true; //--------------------------------->

  //Get the expexted size
  expectedSize = getExpextedSize(array);

  //Cancel here if not enough bytes received
  if (array.size() < (static_cast<int>(expectedSize + CommunicationConstants::Data::SIZE_OVERHEAD)))
    return true; //--------------------------------->


  //Check if the end marking is present. Only use the data if this is the case.
  if (checksumIsCorrect(array, expectedSize))
  {
    uint8_t type = getType(array);
    vector<uint8_t> dataArray = array;
    dataArray.erase(dataArray.begin(), dataArray.begin() + CommunicationConstants::Data::SIZE_HEADER);  //remove(0, CommunicationConstants::Data::SIZE_HEADER); //TODO...

    //Remove checksum at the end    
    dataArray.erase(dataArray.begin() + expectedSize,  dataArray.begin() + expectedSize + sizeof(uint32_t));  //dataArray.remove(expectedSize, sizeof(uint32_t)); //TODO...
    dataArray.erase(dataArray.begin() + expectedSize, dataArray.end()); //dataArray.truncate(expectedSize);
    sigReceivedData(dataArray, type);

  }else{
    ROS_INFO("corrupted data %s", array.data());
  }

  array.clear();

  return false;
}

int SerialConnection::set_interface_attribs (int speed, int parity)
{  
  struct termios tty;
  memset (&tty, 0, sizeof tty);

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fileDescription, TCSANOW, &tty) != 0)
  {
      ROS_INFO("error %d from tcsetattr", errno);
      return -1;
  }
  return 0;
}


void SerialConnection::set_blocking (int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fileDescription, &tty) != 0)
  {
      ROS_INFO("error %d from tggetattr", errno);
      return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  if (tcsetattr (fileDescription, TCSANOW, &tty) != 0)
      ROS_INFO ("error %d setting term attributes", errno);
}


} //end namespace com_lib
