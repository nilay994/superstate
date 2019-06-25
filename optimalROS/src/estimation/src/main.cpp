#include "poseEstimator_node.h"
#include "gate.h"
#include "wayPointPlan.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poseEstimator_node");

  ros::NodeHandle nh;

  mav_estimation::poseEstimator pose_estimator(nh);

  ros::Rate rate(960);

  while (ros::ok() && pose_estimator.shutDown == 0 && pose_estimator.crash == 0)
  {
    ros::spinOnce();
    rate.sleep() ;
  }
  
  return 0;
}