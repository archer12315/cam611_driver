/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication_implementation Base Implementation
 * @brief Abstract base implementation
 * @ingroup communication
 *
 * @{
 */
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "communication_if.h"
#include "serial_connection.h"
#include "update_controller.h"
#include "epc_timer.h"
#include <exception>
#include <list>
#include <string>

#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>


#include <iostream>

namespace com_lib
{

//! Communication Base Implementation
/*!
 * This class implements the communication common for different kinds of sensor/camera systems.
 * For concrete devices, an own class must be implemented with some functions containing device
 * specific tasks.
 */
class Communication: public Communication_IF
{

  public:
    Communication();
    virtual ~Communication();
    std::list<std::string> availableDevices();
    bool open(const unsigned int id);
    void close();

    void setPortName(std::string portName_);

    //General commands
    ErrorNumber_e setPower(const bool enabled);

    //Information commands
    ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader);
    ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader, unsigned int &version);
    ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId);
    ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor);
    std::string getDeviceName();
    Device_e    getCurrentDevice();
    ErrorNumber_e getProductionInfo(unsigned int &year, unsigned int &week);

    //Setup commands
    ErrorNumber_e setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime);
    ErrorNumber_e setIntegrationTimeGrayscale(const unsigned int integrationTime);
    ErrorNumber_e setModulationFrequency(const ModulationFrequency_e modulationFrequency);
    virtual ErrorNumber_e setMode(const Mode_e mode) = 0;
    ErrorNumber_e setFilter(const unsigned int threshold, const unsigned int factor);
    ErrorNumber_e setCalibrationMode(const bool enabled);

    //Update commands 
    void updateFirmware(const std::vector<uint8_t> &updateFile);
    
    //Internal update commands--> these commands are not in the interface, because they are used only internally
    void sendCommandJumpToBootloader();
    void sendCommandFirmwareUpdateStart(const unsigned int fileSize);
    void sendCommandFirmwareUpdateWriteData(const uint8_t *dataToWrite, const uint32_t index, const unsigned int bytesToWrite);
    void sendCommandFirmwareUpdateFinished();


    //public slots:
    //Information commands
    void getTemperature();

    //Acquisition commands
    void getDistanceAmplitude(int size);
    void getDistance(int size);
    void getDcsDistanceAmplitude();
    void getDcs();
    void getGrayscale();
    ErrorNumber_e getIntegrationTime3d(unsigned int &integrationTime);


    //private signals:
    boost::signals2::signal<void (const ErrorNumber_e &errorNumber)> sigErrorInternal;
    boost::signals2::signal<void (const uint32_t identification)> sigReceivedIdentification;
    boost::signals2::signal<void (const uint16_t integrationTime)> sigReceivedIntegrationTime;
    boost::signals2::signal<void (const uint16_t waferId, const uint16_t chipId)> sigReceivedChipInformation;    
    boost::signals2::signal<void (const uint32_t firmwareRelease)> sigReceivedFirmwareRelease;    
    boost::signals2::signal<void (const uint8_t year, const uint8_t week)> sigReceivedProductionInfo;
    boost::signals2::signal<void ()> sigReceivedAck;    
    boost::signals2::signal<void ()> sigReceivedAnswer;


    //private slots:
    void onReceivedData(const std::vector<uint8_t> &array, const uint8_t type);
    void onFirmwareUpdateProgress(const unsigned int progress);
    void onFirmwareUpdateFinished();
    void onTimeout();
    //void onError(QSerialPort::SerialPortError errorMessage); //TODO...

  protected:
    ErrorNumber_e sendCommand(uint8_t *data, int size);
    ErrorNumber_e sendCommandWithoutData(const uint8_t command, int size);
    ErrorNumber_e sendCommandSingleByte(const uint8_t command, const uint8_t payload, const bool blocking);
    ErrorNumber_e sendCommandUint16(const uint8_t command, const uint16_t payload, const bool blocking);
    ErrorNumber_e sendCommand2xUint16(const uint8_t command, const uint16_t payload0, const uint16_t payload1, const bool blocking);

    std::list<std::string> deviceNameList;
    std::vector<int>deviceIdList;
    Device_e connectedDevice;            ///<Stores the device type that is connected

  private:
    void sendErrorSignal(const ErrorNumber_e errorNumber);
    bool openInternal(const unsigned int id, Device_e &device, bool &isBootloader);
    void processIdentification(const std::vector<uint8_t> &array);
    void processChipInformation(const std::vector<uint8_t> &array);
    void processTemperature(const std::vector<uint8_t> &array);
    void processFirmwareRelease(const std::vector<uint8_t> &array);
    void processIntegrationTime(const std::vector<uint8_t> &array);
    void processProductionInfo(const std::vector<uint8_t>  &array);

    std::string createDeviceString(const Device_e device);

    //These functions must be implemented by device specific classes
    virtual bool setupDevice(const Device_e device) = 0;
    virtual void processDistanceAmplitude(const std::vector<uint8_t> &array) = 0;
    virtual void processDistance(const std::vector<uint8_t> &array) = 0;
    virtual void processDcsDistanceAmplitude(const std::vector<uint8_t> &array) = 0;
    virtual void processDcs(const std::vector<uint8_t> &array) = 0;
    virtual void processGrayscale(const std::vector<uint8_t> &array) = 0;
    virtual unsigned int getBaudRate() = 0;

    enum CommunicationState_e
    {
      COMMUNICATION_STATE_UNCONNECTED,
      COMMUNICATION_STATE_NORMAL,
      COMMUNICATION_STATE_UPDATE
    };

    std::string portName;
    SerialConnection *serialConnection;     ///<SerialConnection instance   
    EpcTimer *timeoutTimer;
    CommunicationState_e state;             ///<Defines the state of the communication
    UpdateController updateController;      ///<Update controller instance
    unsigned int timeout;                   ///<Stores the timeout time to use


};
}

#endif // COMMUNICATION_H

/** @} */
