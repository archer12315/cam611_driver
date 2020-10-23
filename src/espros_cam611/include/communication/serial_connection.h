/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup serial_connection Serial Connection
 * @brief Specialized serial port
 * @ingroup communication
 *
 * @{
 */
#ifndef SERIAL_CONNECTION_H
#define SERIAL_CONNECTION_H


#include <vector>
#include <string>
#include <list>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>


namespace com_lib
{

//! Specialized implementation of serial port
/*!
 * This class implements some specific functionality for the communication into the
 * base of serial port
 */
class SerialConnection
{    

public:
    SerialConnection();
    ~SerialConnection();

    bool openPort(std::string &portName, const unsigned int id, const unsigned int baudrate);
    void closePort();

    std::vector<std::string> availableDevices();
    ssize_t sendData(uint8_t *data);

    //signals:
    //receivedData(const std::vector<uint8_t> &array, const uint8_t type); //TODO...
    boost::signals2::signal< void (const std::vector<uint8_t>&, const uint8_t)>  sigReceivedData;

    int readRxData(int size); //slot
    std::vector<uint8_t> rxArray;

  private:
    int getExpextedSize(const std::vector<uint8_t> &array);
    bool checksumIsCorrect(const std::vector<uint8_t> &array, const unsigned int expectedSize);
    bool processData(std::vector<uint8_t> &array);
    uint8_t getType(const std::vector<uint8_t> &array);
    void set_blocking (int should_block);
    int set_interface_attribs (int speed, int parity);


    std::vector<std::string> deviceListString;

    int expectedSize;    
    int fileDescription;
};
}

#endif // SERIAL_CONNECTION_H

/** @} */
