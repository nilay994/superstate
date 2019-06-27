#include "poseEstimator_node.h"

FILE *pnp_f;

namespace mav_estimation {

poseEstimator::poseEstimator(ros::NodeHandle nh)
{
  pnp_f = fopen("pnp_f.csv", "w+");
  // Init subscribers
  filteredState_sub = nh.subscribe("/gt_est", 1000, &poseEstimator::FilteredStateCallback, this);

  imu_sub = nh.subscribe("/uav/sensors/imu", 1000, &poseEstimator::IMUCallback, this);
  irBeacon_sub = nh.subscribe("/uav/camera/left/ir_beacons", 1000, &poseEstimator::IRMarkerCallback, this);
  crash_sub = nh.subscribe("/uav/collision", 1000, &poseEstimator::crashCallback, this);
  gt_sub = nh.subscribe("/tf", 1000, &poseEstimator::gtCallback, this);
  
  // Init publishers 
  odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1000);
  gt_pub = nh.advertise<nav_msgs::Odometry>("gt", 1000);
  // wayPoint_pub = nh.advertise<geometry_msgs::Vector3Stamped>("wayPoint", 50);
  wayPoint_pub = nh.advertise<nav_msgs::Odometry>("wayPoint", 1000);
  // vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vel", 50); pnpMeas
  pnpMeas_pub = nh.advertise<geometry_msgs::Vector3Stamped>("pnpMeas", 1000);

  finishTrack_pub = nh.advertise<std_msgs::Empty>("/finishTrack", 1000);

  gatePointer = new gate[23];  // 23 gates in total in the map, initialize object array

  flightPlanner = new wayPointPlan();

  flightPlanner->setGateInitialWayPoint(gatePointer);  // a pointer as a onput parameter
  // when initialize the planner, give the value to the gates' initial way points

  for(gateID = 0; gateID < 23; gateID++)  // set the global location of the corners
  {
    gatePointer[gateID].ID = gateID + 1;  // 1 ~ 23
    gatePointer[gateID].setMarkerLocation();
    gatePointer[gateID].markerLocationRealWorldFrame();
    //std::cout << "gateCenter_posi_w = " << std::endl << " " <<  gatePointer[gateID].gateCenter_posi_w << std::endl << std::endl;
    //std::cout << " gatePointer[gateID].wayPointInit = " << std::endl << " " <<  gatePointer[gateID].wayPointInit << std::endl << std::endl;
  }

  firstMsg = 1;
  imuFirstMsg = 1;

// drone's initial state  ***

  phi_f = 0;
  theta_f = 0;
  psi_f = 1.57049;

  vel_w_x = 0;
  vel_w_y = 0;
  vel_w_z = 0;

  pos_w_x = 18.0;
  pos_w_y = - 23.0;
  pos_w_z = 5.3;

// initial state  ***

  numPassedTargetGate = 0;

  shutDown = 0;
  crash = 0;

  x_est      = pos_w_x;
  y_est      = pos_w_y;
  height_est = pos_w_z;

  xVel_est = vel_w_x;
  yVel_est = vel_w_y;
  heightVel_est = vel_w_z;

  roll_est  = 0;  // in Rad
  pitch_est = 0;
  yaw_est   = 1.57049;

}

poseEstimator::~poseEstimator()
{
  printf("delete[] gatePointer  ");
  delete[] gatePointer;
  printf("delete flightPlanner");
  delete flightPlanner;
}

void poseEstimator::crashCallback(const std_msgs::Empty &crash_msg)
{
  crash = 1;
}

void poseEstimator::IMUCallback(const sensor_msgs::Imu & imuMsg)
{
  gyro_x = imuMsg.angular_velocity.x;
  gyro_y = imuMsg.angular_velocity.y;
  gyro_z = imuMsg.angular_velocity.z;

  acc_x = imuMsg.linear_acceleration.x;
  acc_y = imuMsg.linear_acceleration.y;
  acc_z = imuMsg.linear_acceleration.z;

  // imuOutput<<imuMsg.header.stamp<<"  "<<gyro_x<<"  "<<gyro_y<<"  "<<gyro_z<<"  "<<acc_x<<"  "<<acc_y<<"  "<<acc_z<<"\n";
  // ROS_INFO_STREAM("time: "<<imuMsg.header.stamp);

  if (imuFirstMsg) {
    dt_imu = 1 / 960;
    imuFirstMsg = 0;
  }
  else {
    dt_imu = imuMsg.header.stamp.toSec() - timeStamp_imu_old;
  }

  // ahrs
  ahrs(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, dt_imu);

  filter_predict(roll_est, pitch_est, yaw_est, dt_imu, acc_x, acc_y, acc_z);

  flightPlanner->switchNextGate(gatePointer[flightPlanner->nextGateID-1], x_est, y_est, height_est);

  if (flightPlanner->finishTrack == 1)
  {
    shutDown = 1;
  }

  // publish the next waypoint position and predicted states
  publish(imuMsg.header.stamp);
  timeStamp_imu_old = imuMsg.header.stamp.toSec();
}

double x_gt = 0;
double y_gt = 0;
double z_gt = 0;
double t_gt = 0;
void poseEstimator::gtCallback(const tf2_msgs::TFMessage &groundTruth_msg) 
{
  t_gt = groundTruth_msg.transforms[0].header.stamp.toSec();
  x_gt = groundTruth_msg.transforms[0].transform.translation.x;
  y_gt = groundTruth_msg.transforms[0].transform.translation.y;
  z_gt = groundTruth_msg.transforms[0].transform.translation.z;
}




void poseEstimator::FilteredStateCallback(const nav_msgs::Odometry &odom_msg)
{
  x_est      = odom_msg.pose.pose.position.x;
  y_est      = odom_msg.pose.pose.position.y;
  height_est = odom_msg.pose.pose.position.z;

  xVel_est = odom_msg.twist.twist.linear.x;
  yVel_est = odom_msg.twist.twist.linear.y;
  heightVel_est = odom_msg.twist.twist.linear.z;

  roll_est  = odom_msg.twist.twist.angular.x;  // in Rad
  pitch_est = odom_msg.twist.twist.angular.y;
  yaw_est   = odom_msg.twist.twist.angular.z;

  // ROS_INFO_STREAM("yaw_est: "<<yaw_est); //odom_msg.header.stamp
  // ROS_INFO_STREAM("Odom_Time: "<<odom_msg.header.stamp); //odom_msg.header.stamp

}

void poseEstimator::IRMarkerCallback(const estimation::IRMarkerArrayConstPtr & IRMarkerArrayMsg) //, const estimation::IRMarkerArrayConstPtr & MarkerAmount)
{

  // markerAmount = getArrayLen(IRMarkerArrayMsg.markers);  // the amount of marker in current image

  for(gateID = 0; gateID < 23; gateID++)
  {
    gatePointer[gateID].reset();
  }

  markerAmount = 0;
  visibleGateAmount = 0;

  //irMarker_timeStamp = IrMarkerMsg.header.stamp;

  //irMarker_timeStamp = IrMarkerMsg.markers[0].landmarkID;

  //irMarker_timeStamp = IrMarkerMsg.header.stamp.secs + IrMarkerMsg.header.stamp.nsecs / 1000000000;

  for (estimation::IRMarker marker : IRMarkerArrayMsg->markers)  // if there is no marker, will not go into the loop
  {  // for every marker

    gateIDmarker = std::stoi(marker.landmarkID.data.substr(4));  // 1 ~ 23
    markerIDmarker = std::stoi(marker.markerID.data); // 1 ~ 4

    // marker.llandmarkIDandmarkID
    // marker.markerID
    // ROS_INFO_STREAM("marker.x: "<<marker.x);
    // ROS_INFO_STREAM("landmarkID: "<<marker.landmarkID.data.substr(4));
    // ROS_INFO_STREAM("markerID: "<<marker.markerID.data);
    // gate obj_newGate();

    gatePointer[gateIDmarker - 1].markerVisibility[markerIDmarker - 1] = 1;

    // gatePointer[gateIDmarker - 1].ID = gateIDmarker;  // 1 ~ 23

    gatePointer[gateIDmarker - 1].markerPixel[markerIDmarker - 1][0] = marker.x;
    gatePointer[gateIDmarker - 1].markerPixel[markerIDmarker - 1][1] = marker.y;

    markerAmount = markerAmount + 1;
  }
  // ROS_INFO_STREAM("markerAmount:"<<markerAmount);


  for(gateID = 0; gateID < 23; gateID++)
  {
    gatePointer[gateID].visibleMarkerAmount = gatePointer[gateID].markerVisibility[0] + gatePointer[gateID].markerVisibility[1] + gatePointer[gateID].markerVisibility[2] + gatePointer[gateID].markerVisibility[3];
    // ROS_INFO_STREAM("gateID:"<<gateID+1<<"  visibleMarkerAmount:"<<gatePointer[gateID].visibleMarkerAmount);
    if(gatePointer[gateID].visibleMarkerAmount > 2)
    {
      gatePointer[gateID].gateVisibility = 1;
      if (gatePointer[gateID].visibleMarkerAmount == 3 && (gatePointer[gateID].ID == 23 || gatePointer[gateID].ID == 15)) // some bad gate openCV pnp is not working ...
      {
        gatePointer[gateID].gateVisibility = 0;
      }
      visibleGateAmount = visibleGateAmount + 1;
    }
  }

  for(gateID = 0; gateID < 23; gateID++)  // for all the visable gates inside image
  {
    // targeted moving gates***************************************************************************************
    if(gatePointer[gateID].gateVisibility == 1)// && gatePointer[gateID].fixedGate() == 0 && (gatePointer[gateID].ID == flightPlanner -> nextGateID))
    // TODO: ID should be the only one next gate
    // if the next target gate is in the image nextGateID
    { // map the gate's corners into the world frame (error including drone pose and pnp error)

      if(gatePointer[gateID].visibleMarkerAmount == 4)
      {
        //ROS_INFO_STREAM("Next Gate:"<<flightPlanner -> nextGateID<<"\n"<<"P4P"<<"\n");
        gatePointer[gateID].relativePoseP4P();
      }
      else
      {  // only 3 points visible
        // ROS_INFO_STREAM("gatePointer[gateID].firstImage: "<<gatePointer[gateID].firstImage);
        if(gatePointer[gateID].firstImage == 1)
        {
          // give initial guess from the filter
          cv::Mat droneLocationWorld= (cv::Mat_<double>(3,1) << x_est, y_est, height_est);

          // std::cout << "droneLocationWorld = " << std::endl << " " << droneLocationWorld << std::endl;

          gatePointer[gateID].tvecP3P = - gatePointer[gateID].Rc2b * eulerAnglesToRotationMatrix(roll_est, pitch_est, yaw_est) * droneLocationWorld;
          cv::Rodrigues(gatePointer[gateID].Rc2b * eulerAnglesToRotationMatrix(roll_est, pitch_est, yaw_est), gatePointer[gateID].rvecP3P);

          gatePointer[gateID].firstImage = 0;
          /*ROS_INFO_STREAM("tvecP3P init guess: "<<gatePointer[gateID].tvecP3P);
          ROS_INFO_STREAM("rvecP3P init guess: "<<gatePointer[gateID].rvecP3P);*/

          cv::Mat Rc2w;
          cv::Rodrigues(gatePointer[gateID].rvecP3P, Rc2w);
          cv::Mat Rw2c = Rc2w.t();
          cv::Mat Cam_posi_w = - Rw2c * gatePointer[gateID].tvecP3P;

          //std::cout << "Cam_posi_w = " << std::endl << " " << Cam_posi_w << std::endl << std::endl;

        }
        for(int i=0; i<4; i++)
        {
          if(gatePointer[gateID].markerVisibility[i] == 0)
          {
            //ROS_INFO_STREAM("Next Gate:"<<flightPlanner -> nextGateID<<"\n"<<"P3P invisible marker: "<<i<<"\n");
            gatePointer[gateID].relativePoseP3P(i);
            // std::cout << "Cam_posi_w = " << std::endl << " " << i << std::endl << std::endl;
          }
        }
      }
      // publish the only one next targeted gate's measurement of drone's pose

      /*cv::Mat cam2gate_fw;
      cam2gate_fw = gatePointer[gateID].gateCenter_posi_w - gatePointer[gateID].Cam_posi_fw;*/

      cv::Mat Rw2b;
      cv::Mat Rb2w;

      Rb2w = eulerAnglesToRotationMatrix(roll_est, pitch_est, yaw_est);
      Rw2b = Rb2w.t();
      cv::Mat cam2gate_w = Rw2b * gatePointer[gateID].Rb2c * gatePointer[gateID].gate_posi_cam;

      //std::cout<< "gatePointer[gateID].Rb2c * gatePointer[gateID].gate_posi_cam " <<"\n"<< gatePointer[gateID].Rb2c * gatePointer[gateID].gate_posi_cam <<"\n"<<"\n";

      cv::Mat drone_posi_w_pnp = gatePointer[gateID].gateCenter_posi_w - cam2gate_w;

      //std::cout<< "drone_posi_w_pnp " <<"\n"<< drone_posi_w_pnp <<"\n"<<"\n";

      pos_vision_x = drone_posi_w_pnp.at<double>(0, 0);
      pos_vision_y = drone_posi_w_pnp.at<double>(1, 0);
      pos_vision_z = gatePointer[gateID].Cam_posi_fw.at<double>(2, 0);

      
      fprintf(pnp_f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
        IRMarkerArrayMsg->header.stamp.toSec(), 
        pos_vision_x, pos_vision_y, pos_vision_z, 
        roll_est, pitch_est, yaw_est, 
        acc_x, acc_y, acc_z);
      

      // PnPOutput<<IRMarkerArrayMsg->header.stamp<<"  "<<flightPlanner->nextGateID<<"  "<<pos_vision_x<<"  "<<pos_vision_y<<"  "<<pos_vision_z<<"\n";

      // std::cout<<IRMarkerArrayMsg->header.stamp<<"  "<<pos_vision_x<<"  "<<pos_vision_y<<"  "<<pos_vision_z<<"\n";

      // gatePointer[gateID].resetMarkersLocation3dRealWorldFrame();
      flightPlanner->normDisGateCenter = 100;
      flightPlanner->wayPointGeneration(gatePointer[gateID], x_est, y_est, height_est);
      if (flightPlanner->normDisGateCenter > 5)
      {
        pnpMeasPublish(IRMarkerArrayMsg->header.stamp);


      }
    }
    // targeted moving gates*end***********************************************************************************
  }

  // void filter_vision_correct(void)


  // locationWorldVision = locationWorldVisionSum / visibleGateAmount;
  // ROS_INFO_STREAM("visibleGateAmount:"<<visibleGateAmount<<"\n");

  // ROS_INFO_STREAM("landmarkID: "<<);
  // ROS_INFO_STREAM("markerAmount: "<<sizeof(IRMarkerArrayMsg.markers[1000]));
//   ROS_INFO_STREAM("pitch_gt: "<<);
//   ROS_INFO_STREAM("yaw_gt: "<<);
//   ROS_INFO_STREAM(" ");
}


void poseEstimator::filter_predict(float phi, float theta, float psi, float dt, float acc_x, float acc_y, float acc_z)  // rad
{
  float roll = phi;
  float pitch = theta;
  float yaw = psi;

  // Body accelerations
  float a_thrust = DR_FILTER_GRAVITY / cos(theta * DR_FILTER_THRUSTCORR) / cos(phi * DR_FILTER_THRUSTCORR);

  // float abx =  sin(-theta) * az;
  // float aby =  sin(phi)   * az;

  // // motion accelerations in world frame
  // float ax =  cos(psi) * abx - sin(psi) * aby - vel_w_x * DR_FILTER_DRAG;
  // float ay =  sin(psi) * abx + cos(psi) * aby - vel_w_y * DR_FILTER_DRAG;

  float ax = (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) * a_thrust - vel_w_x * DR_FILTER_DRAG;
  ax = cos(pitch)*cos(yaw) * acc_x + (cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw)) * acc_y + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) * acc_z;
  float ay = (cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll)) * a_thrust - vel_w_y * DR_FILTER_DRAG;
  ay = cos(pitch)*sin(yaw) * acc_x + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) * acc_y + (cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll)) * acc_z;
  float az = -sin(pitch) * acc_x + cos(pitch)*sin(roll) * acc_y + cos(pitch)*cos(roll) * acc_z - DR_FILTER_GRAVITY;


  // get_time_stamp();
  // Velocity and Position in world frame
  vel_w_x += ax * dt;
  vel_w_y += ay * dt;
  vel_w_z += az * dt;

  pos_w_x += vel_w_x * dt;
  pos_w_y += vel_w_y * dt;
  pos_w_z += vel_w_z * dt;

//   Rbw =

// [                                cos(pitch)*cos(yaw),                                cos(pitch)*sin(yaw),          -sin(pitch)]
// [ cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw), cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw), cos(pitch)*sin(roll)]
// [ sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch), cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll), cos(pitch)*cos(roll)]

// Rwb =

// [ cos((pitch))*cos((yaw)), cos((yaw))*sin((pitch))*sin((roll)) - cos((roll))*sin((yaw)), sin((roll))*sin((yaw)) + cos((roll))*cos((yaw))*sin((pitch))]
// [ cos((pitch))*sin((yaw)), cos((roll))*cos((yaw)) + sin((pitch))*sin((roll))*sin((yaw)), cos((roll))*sin((pitch))*sin((yaw)) - cos((yaw))*sin((roll))]
// [               -sin((pitch)),    cos((pitch))*sin((roll)),   cos((pitch))*cos((roll))]

}

void poseEstimator::pnpMeasPublish(ros::Time timeStamp)
{
  pnpMeas.header.stamp = timeStamp;
  pnpMeas.vector.x = pos_vision_x;
  pnpMeas.vector.y = pos_vision_y;
  pnpMeas.vector.z = pos_vision_z;
  // check whether the result makes sense
  if (pos_vision_z > 30 || pos_vision_z < 0 || pos_vision_y > 65 || pos_vision_y < -50 || pos_vision_x > 30 || pos_vision_x < -20)
  {
    return;
  }

  pnpMeas_pub.publish(pnpMeas);
}

void poseEstimator::publish(ros::Time timeStamp)
{
  if (shutDown == 1)
  {
    finishTrack_pub.publish(finishTrack_msg);
    finishTrack_pub.publish(finishTrack_msg);
    finishTrack_pub.publish(finishTrack_msg);
    std::cout <<"Finished the whole track!" << std::endl;
    return;
  }

  wayPointToPass.header.stamp = timeStamp;
  wayPointToPass.pose.pose.position.x = flightPlanner->nextWayPointToPublish.at<double>(0, 0);
  wayPointToPass.pose.pose.position.y = flightPlanner->nextWayPointToPublish.at<double>(1, 0);
  wayPointToPass.pose.pose.position.z = flightPlanner->nextWayPointToPublish.at<double>(2, 0);

  switch(flightPlanner->nextGateID)
  {
    case 10:
      wayPointToPass.twist.twist.angular.y = 31.5;
      wayPointToPass.twist.twist.angular.z = pi/2;

      wayPointToPass.twist.twist.angular.x = 0;
      break;

    case 21:
      wayPointToPass.twist.twist.angular.y = 30.5;
      wayPointToPass.twist.twist.angular.z = 21*pi/40;

      wayPointToPass.twist.twist.angular.x = 0;
      break;

    case 2:
      wayPointToPass.twist.twist.angular.y = 32;
      wayPointToPass.twist.twist.angular.z = - 6*pi/11;

      wayPointToPass.twist.twist.angular.x = 1;
      break;

    case 13:
      wayPointToPass.twist.twist.angular.y = 27;
      wayPointToPass.twist.twist.angular.z = - pi/2;

      wayPointToPass.twist.twist.angular.x = 0;
      break;

    case 9:
      wayPointToPass.twist.twist.angular.y = 28;
      wayPointToPass.twist.twist.angular.z = - 4*pi/7;

      wayPointToPass.twist.twist.angular.x = 1;
      break;

    case 14:
      wayPointToPass.twist.twist.angular.y = 26;
      wayPointToPass.twist.twist.angular.z = -3*pi/7;


      wayPointToPass.twist.twist.angular.x = 1;
      break;

    case 1:
      wayPointToPass.twist.twist.angular.y = 25;
      wayPointToPass.twist.twist.angular.z = -1*pi/18;

      wayPointToPass.twist.twist.angular.x = 1;
      break;

    case 22:
      wayPointToPass.twist.twist.angular.y = 25;
      wayPointToPass.twist.twist.angular.z = 3*pi/16;

      wayPointToPass.twist.twist.angular.x = 1;
      break;

    case 15:
      wayPointToPass.twist.twist.angular.y = 30;
      wayPointToPass.twist.twist.angular.z = pi/2;

      wayPointToPass.twist.twist.angular.x = 0;
      break;

    case 23:
      wayPointToPass.twist.twist.angular.y = 33;
      wayPointToPass.twist.twist.angular.z = 11*pi/20;
      if (wayPointToPass.pose.pose.position.y < -5)
      {
        wayPointToPass.twist.twist.angular.x = 1;
      }
      else
      {
        wayPointToPass.twist.twist.angular.x = 0;
      }

      break;

    case 6:
      wayPointToPass.twist.twist.angular.y = 35;
      wayPointToPass.twist.twist.angular.z = pi/2;

      wayPointToPass.twist.twist.angular.x = 1;
      break;
  }

  wayPoint_pub.publish(wayPointToPass);

  //publish the odometry message over ROS
  /*odom.header.stamp = timeStamp; //groundTruthMsg.transforms[0].header.stamp;
  odom.header.frame_id = "odom";  // prediction

  //set the position
  odom.pose.pose.position.x = pos_w_x;
  odom.pose.pose.position.y = pos_w_y;
  odom.pose.pose.position.z = pos_w_z;
  // odom.pose.pose.orientation = odom_quat;

  //set the Euler
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vel_w_x;
  odom.twist.twist.linear.y = vel_w_y;
  odom.twist.twist.linear.z = vel_w_z;

  odom.twist.twist.angular.x = roll_est;
  odom.twist.twist.angular.y = pitch_est;
  odom.twist.twist.angular.z = yaw_est;*/
  // odom.twist.twist.angular.z = yaw_est;
  //publish the message
  // odom_pub.publish(odom);

  //publish the gt message over ROS
  gt.header.stamp = timeStamp; //groundTruthMsg.transforms[0].header.stamp;
  gt.header.frame_id = "gt";

  //set the position
  gt.pose.pose.position.x = x_gt;
  gt.pose.pose.position.y = y_gt;
  gt.pose.pose.position.z = height_gt;
  // odom.pose.pose.orientation = odom_quat;

  //set the velocity
  gt.child_frame_id = "base_link";
  gt.twist.twist.linear.x = xVel_gt;
  gt.twist.twist.linear.y = yVel_gt;
  gt.twist.twist.linear.z = heightVel_gt;

  gt.twist.twist.angular.x = roll_gt_r;
  gt.twist.twist.angular.y = pitch_gt_r;
  gt.twist.twist.angular.z = yaw_gt_r;
  // std::cout  <<"yaw_gt_r = " << yaw_gt_r << std::endl;

  //publish the message
  gt_pub.publish(gt);
}

void poseEstimator::ahrs(double gyro_x, double gyro_y, double gyro_z, double acc_x, double acc_y, double acc_z, double dt)
{

  tf2::Vector3 gB = {-sin(theta_f)*DR_FILTER_GRAVITY, sin(phi_f)*cos(theta_f)*DR_FILTER_GRAVITY, cos(phi_f)*cos(theta_f)*DR_FILTER_GRAVITY};
  float norm_gB = sqrt(tf2::tf2Dot(gB, gB));

  tf2::Vector3 gB_scaled = gB/norm_gB;

  tf2::Vector3 acc = {acc_x, acc_y, acc_z};
  float norm_acc = sqrt(tf2::tf2Dot(acc, acc));
  tf2::Vector3 acc_scaled = acc/norm_acc;

  tf2::Vector3 error;
  error = tf2::tf2Cross(acc_scaled, gB_scaled);
  //printf("error: %f,%f,%f\n", error.getX(), error.getY(), error.getZ());

  sum_error_arhs = sum_error_arhs + error*dt;

  float k_p = 0.05;
  float k_i = 0.00;

  tf2::Vector3 imu_pqr = {gyro_x, gyro_y, gyro_z};

  // verified operator overloading.
  tf2::Vector3 pqr = k_p * error + k_i * sum_error_arhs + imu_pqr;

  // give phi theta psi from gyro's p q r
  tf2::Vector3 RotMatVel;
  RotMatVel = {pqr.getX() + pqr.getY()*tan(theta_f)*sin(phi_f) + pqr.getZ()*tan(theta_f)*cos(phi_f),
              pqr.getY()*cos(phi_f) - pqr.getZ()*sin(phi_f),
              pqr.getY()*sin(phi_f)/cos(theta_f) + pqr.getZ()*cos(phi_f)/cos(theta_f)};

  tf2::Vector3 att = {phi_f, theta_f, psi_f};
  //printf("att: %f,%f,%f\n", att.getX(), att.getY(), att.getZ());

  att = att + RotMatVel * dt;
  //printf("RotMatVel: %f,%f,%f\n", RotMatVel.getX(), RotMatVel.getY(), RotMatVel.getZ());

  phi_f   = att.getX();
  theta_f = att.getY();
  psi_f   = att.getZ();

  if(psi_f > pi)
  {
    psi_f = psi_f - 2*pi;
  }
  else if (psi_f < -pi)
  {
    psi_f = psi_f + 2*pi;
  }
  else
  {
    psi_f = psi_f;
  }

  roll_est = phi_f;   //
  pitch_est = theta_f;
  yaw_est = psi_f;


  /* ROS_INFO_STREAM("dt_imu: "<<dt);
  ROS_INFO_STREAM("roll_est: "<<roll_est);
  ROS_INFO_STREAM("pitch_est: "<<pitch_est);
  ROS_INFO_STREAM("yaw_est: "<<yaw_est<<"\n"); */

}

/*void poseEstimator::changeNextGateID()
// every time this function is called, check the distance to the current targeted way point behind the gate,
// if close enough, move the target gate to the next one.
// call this every the estimator has a new result
{
  nextGateID = targetGateID[numPassedTargetGate];
  // ROS_INFO_STREAM("nextGateID: "<<nextGateID);
}*/

cv::Mat poseEstimator::eulerAnglesToRotationMatrix(double roll, double pitch, double yaw)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(roll),   sin(roll),
               0,       -sin(roll),   cos(roll)
               );

    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
               cos(pitch),    0,      -sin(pitch),
               0,             1,      0,
               sin(pitch),    0,      cos(pitch)
               );

    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
               cos(yaw),    sin(yaw),      0,
               -sin(yaw),    cos(yaw),     0,
               0,               0,         1
               );

    // Combined rotation matrix
    cv::Mat R = R_x * R_y * R_z;

    return R;
}

}
