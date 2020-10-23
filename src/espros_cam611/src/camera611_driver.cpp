
#include "camera611_driver.h"
#include "communication_constants.h"
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <fstream>
#include <sstream>

#define DISTANCE 0
#define DISTANCE_AMPLITUDE 1

#define NUM_PIX 64
#define IMAGE_WIDTH  8
#define IMAGE_HEIGHT 8

#define MAX_DIST_10MHZ 15000  //mm
#define MAX_DIST_20MHZ  7500  //mm
#define MAX_RANGE_AMPL 150000  //LSB

using namespace com_lib;
using namespace std;

Settings *Camera611Driver::gSettings;

Camera611Driver::Camera611Driver(const ros::Publisher &imagePublisher1, const ros::Publisher &imagePublisher2, Settings &set):
  distancePublisher(imagePublisher1),
  amplitudePublisher(imagePublisher2)
{
    img16_1.data.resize(IMAGE_WIDTH * IMAGE_HEIGHT * 2);
    img16_2.data.resize(IMAGE_WIDTH * IMAGE_HEIGHT * 2);

    gSettings = &set;
    gSettings->frameRate = 1;
    lastSingleShot = false;
    gSettings->runVideo = false;
    gSettings->updateParam = false;
    gSettings->auto_integration_time = false;
    gSettings->modFrequency = ModulationFrequency_e::MODULATION_FREQUENCY_20MHZ;

    lastSingleShot = false;
    lastStreaming = false;
    frameSeq = 0;
    range_field_of_view = rangeDist.field_of_view = 0.18 * 3.14159265 / 180.0; //grad->rad

    communication.initSerialPort(gSettings->port_name);

    unsigned int minor, major;
    communication.getFirmwareRelease(major, minor);
    ROS_INFO("Firmware release:  major= %d  minor= %d ", major, minor);

    uint16_t chipID, waferID;
    communication.getChipInformation(chipID, waferID);
    ROS_INFO("Chip ID= %d   Wafer ID= %d", chipID, waferID);

    gSettings->device = communication.getCurrentDevice();

    communication.sigReceivedDistance.connect(boost::bind(&Camera611Driver::updateDistanceFrame, this, _1, _2));
    communication.sigReceivedDistanceAmplitude.connect(boost::bind(&Camera611Driver::updateDistanceAmplitudeFrame, this, _1, _2, _3));
    gSettings->updateParam = true;

    setParameters();

    timePublish = ros::Time::now();

    if(gSettings->device == Device_e::DEVICE_TOFRANGE611){
        strDistanceFrameID = "distance_range";
        strAmplitudeFrameID = "amplitude_range";
    }else{
        strDistanceFrameID = "distance_frame";
        strAmplitudeFrameID = "amplitude_frame";
    }

    if(gSettings->startStream)
        gSettings->runVideo = true;
}


Camera611Driver::~Camera611Driver()
{
    communication.close();
}


void Camera611Driver::update()
{
    if(gSettings->runVideo && !gSettings->updateParam){

        updateData(); //streaming

    }else if(gSettings->updateParam){
        setParameters(); //update parameters

        if(gSettings->triggerSingleShot && gSettings->triggerSingleShot != lastSingleShot)
            updateData(); //trigger single shot

        lastSingleShot = gSettings->triggerSingleShot;
    }

}


void Camera611Driver::setParameters()
{
    if(gSettings->updateParam){
        gSettings->updateParam = false;
        ROS_INFO("update parameters");

        framePeriod = 1.0 / gSettings->frameRate;

        if(gSettings->device == Device_e::DEVICE_TOFRANGE611){       
            communication.setModulationFrequency(gSettings->modFrequency);

            if(gSettings->auto_integration_time)
                 communication.setIntegrationTime3d(0, 0); //automatic mode
            else communication.setIntegrationTime3d(0, gSettings->integrationTimeTOF); //manual mode

        }else{
            communication.setIntegrationTime3d(0, gSettings->integrationTimeTOF); //only manual mode
        }

        communication.setFilter(gSettings->kalmanThreshold, (uint)(gSettings->kalmanFactor*1000.0));
    }

}


void Camera611Driver::updateData()
{
    double elapsed_time = ros::Time::now().toSec() - timePublish.toSec();

    if(elapsed_time >= framePeriod){

        if(gSettings->iType == DISTANCE){

            if(gSettings->device == Device_e::DEVICE_TOFRANGE611){
                communication.getDistance(CommunicationConstants::DataSize::RANGE_DISTANCE_SIZE);
            }else{
                communication.getDistance(CommunicationConstants::DataSize::FRAME_DISTANCE_SIZE);
            }

            ROS_DEBUG("UPDATE DISTANCE"); //TODO remove

        } else {

            if(gSettings->device == Device_e::DEVICE_TOFRANGE611){
                communication.getDistanceAmplitude(CommunicationConstants::DataSize::RANGE_DISTANCE_AMPLITUDE_SIZE);
            }else{
                communication.getDistanceAmplitude(CommunicationConstants::DataSize::FRAME_DISTANCE_AMPLITUDE_SIZE);
            }

            ROS_DEBUG("UPDATE DISTANCE AND AMPLITUDE"); //TODO remove
        }

    } // emd if elapsed_time

}


void Camera611Driver::updateDistanceFrame(const uint32_t *distance, const unsigned int numPixel){

    timePublish = ros::Time::now();

    if(gSettings->device == Device_e::DEVICE_TOFRANGE611){

        rangeDist.header.seq = frameSeq++;
        rangeDist.header.stamp = timePublish;
        rangeDist.header.frame_id = strDistanceFrameID;
        rangeDist.radiation_type = sensor_msgs::Range::INFRARED;
        rangeDist.field_of_view = range_field_of_view; //radian
        rangeDist.range = round(distance[0] / 10.0); //to->mm
        rangeDist.min_range = 0;
        if(gSettings->modFrequency == ModulationFrequency_e::MODULATION_FREQUENCY_10MHZ)
            rangeDist.max_range = MAX_DIST_10MHZ; //mm
        else rangeDist.max_range = MAX_DIST_20MHZ; //mm

        distancePublisher.publish(rangeDist);

    }else{

        img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = timePublish;
        img16_1.header.frame_id = strDistanceFrameID;
        img16_1.height = IMAGE_HEIGHT;
        img16_1.width  = IMAGE_WIDTH;
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;

        int i, l;
        uint16_t val = 0;

        for(i=0, l=0; l< NUM_PIX; l++, i+=2){
            val = round(distance[l]/10.0); //to -> mm      
            img16_1.data[i] =  val & 0xff;
            img16_1.data[i+1] = (val>>8) & 0xff;
        }

        distancePublisher.publish(img16_1);
    }    
}

void Camera611Driver::updateDistanceAmplitudeFrame(const uint32_t *distance, const uint32_t *amplitude, const unsigned int numPixel)
{    
    timePublish = ros::Time::now();

    if(gSettings->device == Device_e::DEVICE_TOFRANGE611){

        rangeDist.header.seq = frameSeq++;
        rangeDist.header.stamp = timePublish;
        rangeDist.header.frame_id = strDistanceFrameID;
        rangeDist.radiation_type = sensor_msgs::Range::INFRARED;
        rangeDist.field_of_view = range_field_of_view; //radian
        rangeDist.range = round(distance[0] / 10.0);
        rangeDist.min_range = 0;
        if(gSettings->modFrequency == ModulationFrequency_e::MODULATION_FREQUENCY_10MHZ)
            rangeDist.max_range = MAX_DIST_10MHZ;
        else rangeDist.max_range = MAX_DIST_20MHZ;
        distancePublisher.publish(rangeDist);


        rangeAmpl.header.seq = frameSeq;
        rangeAmpl.header.stamp = timePublish;
        rangeAmpl.header.frame_id = strAmplitudeFrameID;
        rangeAmpl.radiation_type = sensor_msgs::Range::INFRARED;
        rangeAmpl.field_of_view = range_field_of_view; //radian
        rangeAmpl.min_range = 0;
        rangeAmpl.max_range = MAX_RANGE_AMPL;
        rangeAmpl.range = amplitude[0];
        amplitudePublisher.publish(rangeAmpl);

    }else{
        int i,l;
        uint16_t val;
        img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = timePublish;
        img16_1.header.frame_id = strDistanceFrameID;
        img16_1.height = IMAGE_HEIGHT;
        img16_1.width  = IMAGE_WIDTH;
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;        

        for(i=0, l=0; l< NUM_PIX; l++, i+=2 ){
            val = round(distance[l] / 10.0); //to-> mm
            img16_1.data[i] =  val & 0xff;
            img16_1.data[i+1] = (val>>8) & 0xff;
        }

        distancePublisher.publish(img16_1);


        img16_2.header.seq = frameSeq;
        img16_2.header.stamp = timePublish;
        img16_2.header.frame_id = strAmplitudeFrameID;
        img16_2.height = IMAGE_HEIGHT;
        img16_2.width  = IMAGE_WIDTH;
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2;
        img16_2.is_bigendian = 0;


        for(i=0, l=0; l< NUM_PIX; l++, i+=2 ){
            val = amplitude[l];
            img16_2.data[i] =  val & 0xff;
            img16_2.data[i+1] = (val>>8) & 0xff;
        }

        amplitudePublisher.publish(img16_2);

    }

}


