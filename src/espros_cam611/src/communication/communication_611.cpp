#include <iostream>
#include "communication_611.h"
#include "communication_constants.h"
#include "util.h"

using namespace std;

namespace com_lib
{

///Fixed baud rate
const unsigned int BAUDRATE = 921600;

Communication611::Communication611()
{
  actualMode = Mode_e::MODE_ULN;
  //initSerialPort();
}

/**
 * @brief Process distance amplitude
 *
 * This function processes distance amplitude specific for TofRange/Frame 611
 *
 * @param array Array with received data
 */
void Communication611::processDistanceAmplitude(const vector<uint8_t> &array)
{
   //The number of pixels can be calculated from the length, because it is clear, that there are the same number of distance and amplitude values of 32bit
  unsigned int numBytesPerPixel = sizeof(uint32_t) + sizeof(uint32_t);

  unsigned int numPixel = array.size() / numBytesPerPixel;

  //First distance, then amplitude
  unsigned int distanceIndex = 0;
  unsigned int amplitudeIndex = numPixel * sizeof(uint32_t);

  //Set distance pointer
  uint32_t *distance =  (uint32_t *)(&array.data()[distanceIndex]);

  //Set amplitude pointer
  uint32_t *amplitude = (uint32_t *)(&array.data()[amplitudeIndex]);

  //Forward the data
  sigReceivedDistanceAmplitude(distance, amplitude, numPixel);
}

/**
 * @brief Process distance data
 *
 * This function processes distance specific for TofRange/Frame 611
 *
 * @param array Array with received data
 */
void Communication611::processDistance(const vector<uint8_t> &array)
{
  //The number of pixels can be calculated from the length, because it is clear, that there are 32Bits per pixel
  unsigned int numBytesPerPixel = sizeof(uint32_t);

  unsigned int numPixel = array.size() / numBytesPerPixel;
  unsigned int distanceIndex = 0;

  //Make a pointer to the distance
  uint32_t *distance =  (uint32_t *)(&array.data()[distanceIndex]);

  //Forward the data
  sigReceivedDistance(distance, numPixel);
}

/**
 * @brief Process dcs distance amplitude data
 *
 * This function processes DCS distance and amplitude specific for TofRange/Frame 611
 *
 * @param array Array with received data
 */
void Communication611::processDcsDistanceAmplitude(const vector<uint8_t> &array)
{
  //The DCS format depends on the mode. Only the ULN mode needs 32Bits per pixel
  if (actualMode == Mode_e::MODE_ULN)
    processDcsDistanceAmplitudeInternal<uint32_t>(array);
  else
    processDcsDistanceAmplitudeInternal<uint16_t>(array);
}

/**
 * @brief Process dcs distance amplitude data with different types
 *
 * This function processes DCS distance and amplitude specific for TofRange/Frame 611
 *
 * @param array Array with received data
 */
template<typename T> void Communication611::processDcsDistanceAmplitudeInternal(const vector<uint8_t> &array)
{
  //The number of pixels can be calculated from the length, because it is clear, which amount of bytes is expected per pixel
  unsigned int numBytesPerPixel = 4 * sizeof(T) + sizeof(uint32_t) + sizeof(uint32_t);  //4DCs with sizeof(T) bytes, 4bytes for distance and amplitude

  //Calculate helper variables
  unsigned int numPixel = array.size() / numBytesPerPixel;
  unsigned int dcsIndex = 0;
  unsigned int distanceIndex = numPixel * 4 * sizeof(T); //4DCS
  unsigned int amplitudeIndex = distanceIndex + numPixel * sizeof(uint32_t);

  //Set DCS pointer
  T *dcs = (T *)(&array.data()[dcsIndex]);

  //Set distance pointer
  uint32_t *distance =  (uint32_t *)(&array.data()[distanceIndex]);

  //Set amplitude pointer
  uint32_t *amplitude = (uint32_t *)(&array.data()[amplitudeIndex]);

  //Forward the data
  //sigReceivedDcsDistanceAmplitude(dcs, distance, amplitude, numPixel);  
}

/**
 * @brief Process dcs  data
 *
 * This function processes DCS specific for TofRange/Frame 611
 *
 * @param array Array with received data
 */
void Communication611::processDcs(const vector<uint8_t> &array)
{
  //The DCS format depends on the mode. Only the ULN mode needs 32Bits per pixel
  if (actualMode == Mode_e::MODE_ULN)  
    processDcsInternal<uint32_t>(array);
  else
    processDcsInternal<uint16_t>(array);
}

/**
 * @brief Process dcs data with different types
 *
 * This template function processes DCS specific for TofRange/Frame 611 and for different types
 *
 * @param array Array with received data
 */
template<typename T> void Communication611::processDcsInternal(const vector<uint8_t> &array)
{
  //The number of pixels can be calculated from the length, because it is clear, which amount of bytes is expected per pixel
  unsigned int numBytesPerPixel = 4 * sizeof(T);  //4DCs

  //Calculate helper variables
  unsigned int numPixel = array.size() / numBytesPerPixel;

  //Set DCS pointer
  T *dcs = (T *)(&array.data()[0]);

  //Forward the data
  //sigReceivedDcs(dcs, numPixel); //TODO...  
}

/**
 * @brief Process grayscale  data
 *
 * This function processes grayscale data specific for TofRange/Frame 611
 *
 * @param array Array with received data
 */
void Communication611::processGrayscale(const vector<uint8_t> &array)
{
  //The number of pixels can be calculated from the length, because it is clear, that there are 32Bits per pixel
  unsigned int numBytesPerPixel = sizeof(uint16_t);

  unsigned int numPixel = array.size() / numBytesPerPixel;
  unsigned int grayscaleIndex = 0;

  //Make a pointer to the distance
  uint16_t *grayscale =  (uint16_t *)(&array.data()[grayscaleIndex]);

}

/**
 * @brief Setup device
 *
 * Tasks:
 * - Check, if it is a device we are looking for
 * - Setup default values per device. This is so far:
 *    - The operation mode
 *
 * @retval true It is a correct device: TofRange611 or TofFrame611
 * @retval false No correct/wanted device
 * @return Correct device true/false
 */
bool Communication611::setupDevice(const Device_e device)
{
  bool deviceIsOk = false;

  switch(device)
  {
    case Device_e::DEVICE_TOFFRAME611:
      actualMode = Mode_e::MODE_TIM;
      deviceIsOk = true;
      break;
    case Device_e::DEVICE_TOFRANGE611:
      actualMode = Mode_e::MODE_ULN;
      deviceIsOk = true;
      break;
    case Device_e::DEVICE_UNKNOWN:      
      //Just a default mode. It will be set correctly, when firmware is loaded to the device
      actualMode = Mode_e::MODE_ULN;

      //Device unknown is also ok! This is for a fabric new device and the user must be able to connect to it
      deviceIsOk = true;
      break;
    default:
      actualMode = Mode_e::MODE_ULN;
      break;
  }

  return deviceIsOk;
}

/**
 * @brief Set mode
 *
 * Call this function to set the operation mode
 *
 * @param mode Mod eto set
 */
ErrorNumber_e Communication611::setMode(const Mode_e mode)
{
  ErrorNumber_e status = ErrorNumber_e::ERROR_NUMBER_INVALID_PARAMETER;

  switch(mode)
  {
    case Mode_e::MODE_TIM:
      status = sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeEpc611::MODE_TIM, true);
      break;
    case Mode_e::MODE_UFS:
      status = sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeEpc611::MODE_UFS, true);
      break;
    case Mode_e::MODE_ULN:
      status = sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeEpc611::MODE_ULN, true);
      break;
    default:
      sigError(status);
      break;
  }  

  //Store the mode to be used for calculations
  actualMode = mode;

  return status;
}

/**
 * @brief Get the baud rate
 *
 * This function returns the baud rate for this device.
 *
 * @return baud rate
 */
unsigned int Communication611::getBaudRate()
{
  return BAUDRATE;
}


void Communication611::initSerialPort(std::string portName_)
{
  setPortName(portName_);
  availableDevices(); //opend and close
  open(0);
  setPower(true);

  unsigned int minor, major;
  getFirmwareRelease(major, minor);  

  uint16_t chipRevision, waferID;
  getChipInformation(chipRevision, waferID);  
}


}  //end namespace com_lib

