#include <ros/ros.h>
#include <espros_cam611/cam611_frameConfig.h>
#include "camera611_driver.h"


ros::Publisher distancePublisher;
ros::Publisher amplitudePublisher;
Settings settings;

//===================================================================

void updateConfig(espros_cam611::cam611_frameConfig &config, uint32_t level)
{
    settings.runVideo = false;

    settings.iType              = config.image_type;
    settings.frameRate          = config.frame_rate;
    settings.startStream        = config.start_stream;
    settings.triggerSingleShot  = config.trigger_single_shot;
    settings.integrationTimeTOF = config.integration_time_tof;
    settings.kalmanFactor       = config.kalman_factor;
    settings.kalmanThreshold    = config.kalman_threshold;

    if(config.start_stream) settings.runVideo = true;

    settings.updateParam = true;
}


void initialise()
{
    ros::NodeHandle nh("~");

    nh.param("port_name", settings.port_name, std::string("/dev/ttyUSB0"));
    nh.param("image_type", settings.iType, 0);
    nh.param("frame_rate", settings.frameRate, 30.0);
    nh.param("start_stream", settings.startStream, false);
    nh.param("trigger_single_shot", settings.triggerSingleShot, false);
    nh.param("integration_time_tof", settings.integrationTimeTOF, 200);
    nh.param("kalman_factor", settings.kalmanFactor, 0.01);
    nh.param("kalman_threshold", settings.kalmanThreshold, 300);

    distancePublisher  = nh.advertise<sensor_msgs::Image>("distance_image",  1000);
    amplitudePublisher = nh.advertise<sensor_msgs::Image>("amplitude_image", 1000);
    ROS_INFO("Epc611 TOF driver version 1.0");
}

//======================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam611_frame");

    dynamic_reconfigure::Server<espros_cam611::cam611_frameConfig> server;
    dynamic_reconfigure::Server<espros_cam611::cam611_frameConfig>::CallbackType f;
    f = boost::bind(&updateConfig, _1, _2);
    server.setCallback(f);

    initialise();

    Camera611Driver cameraDriver(distancePublisher, amplitudePublisher, settings);

    while(ros::ok()){
        cameraDriver.update();
        ros::spinOnce();        
    }
}












