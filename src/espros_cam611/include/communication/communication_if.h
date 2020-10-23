/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication Communication
 * @brief Communication to the camera
 * @defgroup communication_if Interface
 * @brief Communication Interface
 * @ingroup communication
 *
 * @{
 */
#ifndef COMMUNICATION_IF_H
#define COMMUNICATION_IF_H

#include <list>
#include <string>
#include <vector>

#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace com_lib
{

///Enum to define the modulation frequencies
enum ModulationFrequency_e
{
  MODULATION_FREQUENCY_10MHZ = 0,
  MODULATION_FREQUENCY_20MHZ = 1
};

///Enum to define the modes
enum Mode_e
{
  MODE_TIM = 0,
  MODE_ULN = 1,
  MODE_UFS = 2
};

//These error numbers are given in the signal "error"
enum ErrorNumber_e
{
  ERROR_NUMMBER_NO_ERROR = 0,
  ERROR_NUMBER_TIMEOUT = 32768,
  ERROR_NUMBER_NOT_ACKNOWLEDGE = 32769,
  ERROR_NUMBER_INVALID_PARAMETER = 32770,
  ERROR_NUMBER_SERIAL_PORT_ERROR = 32771
};

//Device list. The device is read with the command "getIdentification"
enum Device_e
{
  DEVICE_UNKNOWN = 0,
  DEVICE_TOFRANGE611 = 1,
  DEVICE_TOFFRAME611 = 2,
  DEVICE_TOFCAM635 = 3
};

//! Communication Interface
/*!
 * This abstract class is the interface for the communication. All access must be done using this
 * interface.
 */
/* --Remarks--
 *
 * Bit width
 * Where the number of bits is important, the stdint types are used. If the number of bits is not important, the
 * standard types are used, so they could be 32Bit or 64Bit without affect. In any case they are as big as in the
 * sensor module or bigger. Like this the compiler on the PC does not need to do unneeded masking (for example
 * for a uint32_t on a 64Bit machine).
 *
 * Blocking/NonBlocking
 * Settings commands are done blocking. This makes it easier to send a list of commands.
 * Acquisition commands are done non blocking
 */
class Communication_IF
{

  public:                                                                                                           //Command type
    Communication_IF(){};    
    virtual ~Communication_IF(){};

    virtual std::list<std::string> availableDevices() = 0;
    virtual bool open(const unsigned int id) = 0;
    virtual void close() = 0;
	
	//General commands
    virtual ErrorNumber_e setPower(const bool enabled) = 0;                                                         //blocking command
	
	//Information commands
    virtual ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader) = 0;                              //blocking command
    virtual ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader, unsigned int &version) = 0;       //blocking command
    virtual ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId) = 0;                              //blocking command
    virtual ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor) = 0;                         //blocking command
    virtual std::string getDeviceName() = 0;                                                                        //blocking command
    virtual Device_e    getCurrentDevice() = 0;
    virtual ErrorNumber_e getProductionInfo(unsigned int &year, unsigned int &week) = 0;                            //blocking command
    	
	//Setup commands
    virtual ErrorNumber_e setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime) = 0;   //blocking command
    virtual ErrorNumber_e setIntegrationTimeGrayscale(const unsigned int integrationTime) = 0;                      //blocking command
    virtual ErrorNumber_e setModulationFrequency(const ModulationFrequency_e modulationFrequency) = 0;              //blocking command
    virtual ErrorNumber_e setMode(const Mode_e mode) = 0;                                                           //blocking command
    virtual ErrorNumber_e setFilter(const unsigned int threshold, const unsigned int factor) = 0;                   //blocking command
    virtual ErrorNumber_e setCalibrationMode(const bool enabled) = 0;                                               //blocking command
	
  //Update commands
    virtual void updateFirmware(const std::vector<uint8_t> &updateFile) = 0;                                                  // --> firmwareUpdateProgress

  //public slots
    //Information commands
    virtual void getTemperature() = 0;                                                                              // --> receivedTemperature

    //Acquisition commands
    virtual void getDistanceAmplitude(int size) = 0;                                                                        // --> receivedDistanceAmplitude
    virtual void getDistance(int size) = 0;                                                                                 // --> receivedDistance
    virtual void getDcsDistanceAmplitude() = 0;                                                                     // --> receivedDcsDistanceAmplitude
    virtual void getDcs() = 0;                                                                                      // --> receivedDcs
    virtual void getGrayscale() = 0;                                                                                // --> receivedGrayscale
    virtual ErrorNumber_e getIntegrationTime3d(unsigned int &integrationTime) = 0;                                  //blocking command

  //signals
    boost::signals2::signal<void (const uint32_t *distance, const unsigned int numPixel)> sigReceivedDistance;
    boost::signals2::signal<void (const uint32_t *distance, const uint32_t *amplitude, const unsigned int numPixel)> sigReceivedDistanceAmplitude;
    boost::signals2::signal<void (const uint16_t *dcs, const uint32_t *distance, const uint32_t *amplitude, const unsigned int numPixel)> sigReceivedDcsDistanceAmplitude16; //DCS with 16Bits
    boost::signals2::signal<void (const uint32_t *dcs, const uint32_t *distance, const uint32_t *amplitude, const unsigned int numPixel)> sigReceivedDcsDistanceAmplitude32; //DCS with 32Bits
    boost::signals2::signal<void (const uint16_t *dcs, const unsigned int numPixel)> sigReceivedDcs16;   //DCS with 16Bits
    boost::signals2::signal<void (const uint32_t *dcs, const unsigned int numPixel)> sigReceivedDcs32;  //DCS with 32Bits
    boost::signals2::signal<void (const uint16_t *grayscale, const unsigned int numPixel)> sigReceivedGrayscale;
    boost::signals2::signal<void((const int16_t temperature))> sigReceivedTemperature;
    boost::signals2::signal<void (const unsigned int)> sigFirmwareUpdateProgress;    
    boost::signals2::signal<void (const ErrorNumber_e errorNumber)> sigError;
};
} //end namespace com_lib

#endif // COMMUNICATION_IF_H

/** @} */
