/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication_epc611 EPC611
 * @brief Implementation for TofFrame611 and TofRange611
 * @ingroup communication
 *
 * @{
 */
#ifndef COMMUNICATION_611_H
#define COMMUNICATION_611_H

#include "communication.h"

namespace com_lib
{
//! Communication Implementation for TofRange611 and TofFrame611
/*!
 * This class implements the specific functionality for TofRange611 and TofFrame611 devices.
 */
class Communication611: public Communication
{
public:
    Communication611();
    ErrorNumber_e setMode(const Mode_e mode);
    void initSerialPort(std::string portName_);

private:    
    unsigned int getBaudRate();
    bool setupDevice(const Device_e device);
    void processDistanceAmplitude(const std::vector<uint8_t> &array);
    void processDistance(const std::vector<uint8_t> &array);
    void processDcsDistanceAmplitude(const std::vector<uint8_t> &array);
    void processDcs(const std::vector<uint8_t> &array);
    void processGrayscale(const std::vector<uint8_t> &array);
    template<typename T> void processDcsInternal(const std::vector<uint8_t> &array);
    template<typename T> void processDcsDistanceAmplitudeInternal(const std::vector<uint8_t> &array);

    Mode_e actualMode;   ///<Stores the actual selected mode
};
}

#endif // COMMUNICATION_611_H

/** @} */
