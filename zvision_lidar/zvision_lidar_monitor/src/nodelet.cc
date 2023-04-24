
#include<thread>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "monitor.h"

volatile sig_atomic_t flag = 1;
namespace zvision_lidar_driver
{
class MonitorNodelet : public nodelet::Nodelet
{
public:
  MonitorNodelet() : running_(false)
  {
  }

  ~MonitorNodelet()
  {
    if (running_)
    {
      NODELET_INFO("shutting down driver thread");
      running_ = false;
      deviceThread_->join();
      NODELET_INFO("driver thread stopped");
    }
  }

private:
  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_; 
  std::shared_ptr<std::thread> deviceThread_;
  std::shared_ptr<zvisionLidarMonitor> mor_;
};

void MonitorNodelet::onInit()
{
  // start the driver
  mor_.reset(new zvisionLidarMonitor(getNodeHandle(), getPrivateNodeHandle()));
  
  // spawn device poll thread
  running_ = true;
  deviceThread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&MonitorNodelet::devicePoll, this)));
}

/** @brief Device poll thread main loop. */
void MonitorNodelet::devicePoll()
{
  while (ros::ok())
  {
    mor_->pollMsg();
    ros::spinOnce();
  }
}
}

PLUGINLIB_EXPORT_CLASS(zvision_lidar_driver::MonitorNodelet, nodelet::Nodelet)
