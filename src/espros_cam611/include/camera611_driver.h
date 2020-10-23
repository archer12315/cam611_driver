#ifndef CAMERA611_DRIVER_H
#define CAMERA611_DRIVER_H

#include "communication_611.h"
#include <dynamic_reconfigure/server.h>
#include <espros_cam611/cam611_frameConfig.h>
#include <espros_cam611/cam611_rangeConfig.h>

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>


struct Settings{

  int iType;
  double frameRate;
  bool startStream;
  bool triggerSingleShot;
  bool runVideo;
  bool updateParam;
  bool auto_integration_time;
  int integrationTimeTOF;
  double kalmanFactor;
  int kalmanThreshold;    
  std::string port_name;
  com_lib::Device_e device;
  com_lib::ModulationFrequency_e modFrequency;

};



class Camera611Driver
{

public:
    Camera611Driver(const ros::Publisher &imagePublisher1, const ros::Publisher &imagePublisher2, Settings &set);
    ~Camera611Driver();
    void update();       

private:

    static Settings *gSettings;
    const ros::Publisher &distancePublisher;
    const ros::Publisher &amplitudePublisher;

    bool lastSingleShot;
    bool lastStreaming;
    unsigned int frameSeq;
    double range_field_of_view;
    double framePeriod;
    std::string strDistanceFrameID;
    std::string strAmplitudeFrameID;

    sensor_msgs::Image img16_1;
    sensor_msgs::Image img16_2;
    sensor_msgs::Range rangeDist;
    sensor_msgs::Range rangeAmpl;
    ros::Time timePublish;

    com_lib::Communication611 communication;

    void updateData();
    void setParameters();
    void updateDistanceFrame(const uint32_t *distance, const unsigned int numPixel);
    void updateDistanceAmplitudeFrame(const uint32_t *distance, const uint32_t *amplitude, const unsigned int numPixel);

};



#endif // CAMERA611_DRIVER_H
