/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication_constants Communication Constants
 * @ingroup communication
 *
 * @brief Constants needed for communication on the firmware and on the PC.
 *
 * @{
 */
#ifndef COMMUNICATION_COMMUNICATIONCONSTANTS_H_
#define COMMUNICATION_COMMUNICATIONCONSTANTS_H_

#include <stdint.h>

//! Communication constants
/*!
 * Constants needed for communication on the firmware and on the PC.
 */

namespace CommunicationConstants
{
  namespace Data
	{
    const uint8_t START_MARK = 0xFA;                                           ///<Start marker for data
    const uint32_t INDEX_TYPE = 1;                                             ///<Index where the type is found in the buffer
    const uint32_t INDEX_LENGTH = 2;                                           ///<Index where the length is found in the buffer
    const uint32_t INDEX_DATA = 4;                                             ///<Index where the data is found in the buffer
    const uint32_t SIZE_HEADER = 4;                                            ///<Number of bytes for the data header
    const uint32_t SIZE_OVERHEAD = SIZE_HEADER + sizeof(uint32_t);             ///<Nuber of overhead bytes = additional bytes to payload    
	}
  namespace Command
	{
    const uint8_t START_MARK = 0xF5;                                           ///<Start marker for commands
    const uint32_t INDEX_COMMAND = 1;                                          ///<Index where the command is found in the buffer
    const uint32_t INDEX_DATA = 2;                                             ///<Index where the data if ound in a command
    const uint32_t INDEX_CHECKSUM = 10;                                        ///<Index where the checksum is found in a command
    const uint32_t SIZE_TOTAL = 14;                                            ///<Number of bytes for one command
    const uint32_t SIZE_PAYLOAD = 8;                                           ///<Number of bytes of the payload of a command
	}
  namespace Type
	{
    const uint8_t DATA_ACK = 0x00;                                             ///<Acknowledge from sensor to host
    const uint8_t DATA_NACK = 0x01;                                            ///<Not acknowledge from sensor to host
    const uint8_t DATA_IDENTIFICATION = 0x02;                                  ///<Identification to identify the device
    const uint8_t DATA_DISTANCE = 0x03;                                        ///<Distance information
    const uint8_t DATA_AMPLITUDE = 0x04;                                       ///<Amplitude information
    const uint8_t DATA_DISTANCE_AMPLITUDE = 0x05;                              ///<Distance and amplitude information
    const uint8_t DATA_GRAYSCALE = 0x06;                                       ///<Grayscale information
    const uint8_t DATA_DCS = 0x07;                                             ///<DCS data
    const uint8_t DATA_DCS_DISTANCE_AMPLITUDE = 0x08;                          ///<DCS, distance and amplitude all together
    const uint8_t DATA_INTEGRATION_TIME = 0x09;                                ///<Integration time, answer to COMMAND_GET_INTEGRATION_TIME_3D
    const uint8_t DATA_PRODUCTION_INFO = 0xF9;                                 ///<Production info
    const uint8_t DATA_CALIBRATION_DATA = 0xFA;                                ///<Calibration data
    const uint8_t DATA_REGISTER = 0xFB;                                        ///<Register data
    const uint8_t DATA_TEMPERATURE = 0xFC;                                     ///<Temperature data
    const uint8_t DATA_CHIP_INFORMATION = 0xFD;                                ///<Chip information data
    const uint8_t DATA_FIRMWARE_RELEASE = 0xFE;                                ///<Firmware release
    const uint8_t DATA_ERROR = 0xFF;                                           ///<Error number
	}
  namespace CommandList
	{
    //setup commands
    const uint8_t COMMAND_SET_INTEGRATION_TIME_3D = 0x00;                      ///<Command to set the integration time for 3D operation
    const uint8_t COMMAND_SET_INTEGRATION_TIME_GRAYSCALE = 0x01;               ///<Command to set the integration time for grayscale
    const uint8_t COMMAND_SET_ROI = 0x02;                                      ///<Command to set the region of interest
    const uint8_t COMMAND_SET_BINNING = 0x03;                                  ///<Command to set the binning
    const uint8_t COMMAND_SET_MODE = 0x04;                                     ///<Command to set the mode
    const uint8_t COMMAND_SET_MODULATION_FREQUENCY = 0x05;                     ///<Command to set the modulation frequency
    const uint8_t COMMAND_SET_DLL_STEP = 0x06;                                 ///<Command to set the DLL step
    const uint8_t COMMAND_SET_FILTER = 0x07;                                   ///<Command to set the filter parameters

    //acquisition commands
    const uint8_t COMMAND_GET_DISTANCE = 0x20;                                 ///<Command to request distance data
    const uint8_t COMMAND_GET_AMPLITUDE = 0x21;                                ///<Command to request amplitude data
    const uint8_t COMMAND_GET_DISTANCE_AMPLITUDE = 0x22;                       ///<Command to request distance and amplitude data
    const uint8_t COMMAND_GET_DCS_DISTANCE_AMPLITUDE = 0x23;                   ///<Command to request distance, amplitude and DCS data at once
    const uint8_t COMMAND_GET_GRAYSCALE = 0x24;                                ///<Command to request grayscale data
    const uint8_t COMMAND_GET_DCS = 0x25;                                      ///<Command to request DCS data
    const uint8_t COMMAND_SET_AUTO_ACQUISITION = 0x26;                         ///<Command to enable/disable the auto acquisition
	const uint8_t COMMAND_GET_INTEGRATION_TIME_3D = 0x27;                      ///<Command to read the integration time. Important when using automatic mode

    //general commands
    const uint8_t COMMAND_SET_POWER = 0x40;                                    ///<Command to enable/disable the power
    const uint8_t COMMAND_CALIBRATE_DRNU = 0x41;                               ///<Command to start DRNU calibration
    const uint8_t COMMAND_CALILBRATE_OFFSET = 0x42;                            ///<Command to calibrate the system offset
    const uint8_t COMMAND_GET_CALIBRATION = 0x43;                              ///<Command to read back the calibration for backup/restore
    const uint8_t COMMAND_JUMP_TO_BOOTLOADER = 0x44;                           ///<Command for application to jump to bootloader
    const uint8_t COMMAND_UPDATE_FIRMWARE = 0x45;                              ///<Command to update the firmware
    const uint8_t COMMAND_IDENTIFY = 0x47;                                     ///<Command to identify
    const uint8_t COMMAND_GET_CHIP_INFORMATION = 0x48;                         ///<Command to read the chip information
    const uint8_t COMMAND_GET_FIRMWARE_RELEASE = 0x49;                         ///<Command to read the firmware release
    const uint8_t COMMAND_GET_TEMPERATURE = 0x4A;                              ///<Command to read the temperature
    const uint8_t COMMAND_SET_PRODUCTION_INFO = 0x4F;                          ///<Command to set the production info
    const uint8_t COMMAND_GET_PRODUCTION_INFO = 0x50;                          ///<Command to get the production info
	}
  namespace Update
	{
    const uint32_t INDEX_CONTROL = 2;                                          ///<Index of the control byte
    const uint32_t INDEX_INDEX = 3;                                            ///<Index of the index
    const uint32_t INDEX_DATA = 6;                                             ///<Index of the update data
    const uint8_t CONTROL_START = 0;                                           ///<Control byte start update
    const uint8_t CONTROL_WRITE_DATA = 1;                                      ///<Control byte write data
    const uint8_t CONTROL_COMPLETE = 2;                                        ///<Control byte update complete
    static const uint32_t PASSWORD_DELETE = 0x654321;                          ///<Password needed to prevent an accidently delete
	}
  namespace IntegrationTime
	{
  	const uint32_t INDEX_INDEX_3D = 2;                                         ///<Index of the integration time 3d index
  	const uint32_t INDEX_INTEGRATION_TIME_3D = 3;                              ///<Index of the integration time 3d
  	const uint32_t INDEX_INTEGRATION_TIME_GRAYSCALE = 2;                       ///<Index of the integration time grayscale
	}
  namespace Identification
	{
    const uint8_t CHIP_TYPE_EPC611 = 0x06;                                         ///<Chip type for EPC611
    const uint8_t CHIP_TYPE_EPC635 = 0x04;                                         ///<Chip type for EPC635
    const uint32_t MASK_CHIP_TYPE_DEVICE = 0x00FFFF00;                             ///<Mask out chip type and device
    const uint32_t SHIFT_CHIP_TYPE_DEVICE = 8;                                     ///<Shift for chip type and device
    const uint8_t DEVICE_TOFRANGE = 0x00;                                          ///<Device value for a TofRange
    const uint8_t DEVICE_TOFFRAME = 0x01;                                          ///<Device value for a TofFrame
    const uint8_t DEVICE_TOFCAM = 0x00;                                            ///<Device value for a TofCam

    const uint32_t DEVICE_TOFRANGE611 = (CHIP_TYPE_EPC611 << 8) | DEVICE_TOFRANGE; ///<Combination of chip type and device to get TofRange611
    const uint32_t DEVICE_TOFFRAME611 = (CHIP_TYPE_EPC611 << 8) | DEVICE_TOFFRAME; ///<Combination of chip type and device to get TofFrame611
    const uint32_t DEVICE_TOFCAM635 = (CHIP_TYPE_EPC635 << 8) | DEVICE_TOFCAM;     ///<Combination of chip type and device to get TofCam635

    const uint32_t MASK_VERSION = 0x000000FF;                                      ///<Mask to get the version
    const uint32_t SHIFT_VERSION = 0;                                              ///<Shift for the version

    const uint32_t VALUE_BOOTLOADER = 0x80000000;                              ///<Or this value with the identification in the bootloader
    const uint32_t VALUE_APPLICATION = 0x00000000;                             ///<Or this value with the identification in the application
	}
  namespace ModulationFrequency
	{
    const uint8_t VALUE_10MHZ = 0;                                             ///<Value for 10MHz for command "COMMAND_SET_MODULATION_FREQUENCY"
    const uint8_t VALUE_20MHZ = 1;                                             ///<Value for 20MHz for command "COMMAND_SET_MODULATION_FREQUENCY"
	}
  namespace ModeEpc611
	{
    const uint8_t MODE_TIM = 0;                                                ///<Value for mode TIM for command "COMMAND_SET_MODE"
    const uint8_t MODE_ULN = 1;                                                ///<Value for mode ULN for command "COMMAND_SET_MODE"
    const uint8_t MODE_UFS = 2;                                                ///<Value for mode UFS for command "COMMAND_SET_MODE"
    const uint8_t MODE_UHD = 3;                                                ///<Value for mode UHD for command "COMMAND_SET_MODE"
    const uint8_t MODE_LNH = 4;                                                ///<Value for mode LNH for command "COMMAND_SET_MODE"
    const uint8_t MODE_RBH = 5;                                                ///<Value for mode RBH for command "COMMAND_SET_MODE"
    const uint8_t MODE_NBF = 6;                                                ///<Value for mode NBF for command "COMMAND_SET_MODE"
    const uint8_t MODE_GBI8 = 7;                                               ///<Value for mode GBI8 for command "COMMAND_SET_MODE"
    const uint8_t MODE_GBI4 = 8;                                               ///<Value for mode GBI4 for command "COMMAND_SET_MODE"
	}
  namespace ChipInformation
	{
    const uint32_t INDEX_WAFER_ID = 6;                                         ///<Index of wafer id in data type "DATA_CHIP_INFORMATION"
    const uint32_t INDEX_CHIP_ID = 4;                                          ///<Index of chip id in data type "DATA_CHIP_INFORMATION"
	}

  namespace DataSize{
    const int FRAME_DISTANCE_SIZE = 264;           //64*4   + 8
    const int FRAME_DISTANCE_AMPLITUDE_SIZE = 520; //64*4*2 + 8
    const int RANGE_DISTANCE_SIZE = 12;            //1*4 + 8
    const int RANGE_DISTANCE_AMPLITUDE_SIZE = 16;  //2*4 + 8
  }



};

#endif /* COMMUNICATION_COMMUNICATIONCONSTANTS_H_ */
