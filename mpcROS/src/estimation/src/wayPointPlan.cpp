#include "wayPointPlan.h"

wayPointPlan::wayPointPlan()
{
  passedGateNum = 0;
  nextGateID = targetedGateID[passedGateNum];

  finishTrack = 0;

}

wayPointPlan::~wayPointPlan()
{

}

void wayPointPlan::setGateInitialWayPoint(mav_estimation::gate *gatePointer)
{
  // 10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6
  gatePointer[10-1].wayPointInit = (cv::Mat_<double>(3,1) << 18.15111, 3.631447, 7.229498);
  gatePointer[21-1].wayPointInit = (cv::Mat_<double>(3,1) << 15.15353, 37.63426, 8.3);
  gatePointer[2-1].wayPointInit = (cv::Mat_<double>(3,1) << 3.94025883196, 37.9998104049, 6.50142895792);
  gatePointer[13-1].wayPointInit = (cv::Mat_<double>(3,1) << -1.237332, 9.001728, 4.9625);
  // gatePointer[9-1].wayPointInit = (cv::Mat_<double>(3,1) << -7.29666570793, 1.06908989458, 3.05683742713);
  gatePointer[9-1].wayPointInit = (cv::Mat_<double>(3,1) << -6.40867, 0.13678, 8.152941);
  // gatePointer[14-1].wayPointInit = (cv::Mat_<double>(3,1) << -15.6707641403, -18.4360531809, 3.2969726723); // watch
  gatePointer[14-1].wayPointInit = (cv::Mat_<double>(3,1) << -9.0, -31.0, 3.76);  // pass without watching
  gatePointer[1-1].wayPointInit = (cv::Mat_<double>(3,1) << -6.84783800535, -30.7576196419, 4.45204979453);
  // gatePointer[22-1].wayPointInit = (cv::Mat_<double>(3,1) << 0.468364220337, -34.2257074263, 6.84781716967);
  gatePointer[22-1].wayPointInit = (cv::Mat_<double>(3,1) << 8.0, -28.0, 3.7);
  gatePointer[15-1].wayPointInit = (cv::Mat_<double>(3,1) << 8.10817764434, -28.0387956159, 3.54747641147);
  gatePointer[23-1].wayPointInit = (cv::Mat_<double>(3,1) << -8.40867, -5.13678, 6.152941); 
  gatePointer[6-1].wayPointInit = (cv::Mat_<double>(3,1) << -9.14867, 30.62316, 6.020941);
}

void wayPointPlan::wayPointGeneration(mav_estimation::gate& nextTargetedGate, double posi_x, double posi_y, double posi_z)
{// 2 way points, 2 meters from the gate center
  drone_posi_w = (cv::Mat_<double>(3,1) << posi_x, posi_y, posi_z);

  double wayPointDistance = WAYPOINT_DISTANCE;

  if (nextTargetedGate.ID == 10 || nextTargetedGate.ID == 2 || nextTargetedGate.ID == 13 || nextTargetedGate.ID == 9 || nextTargetedGate.ID == 15 || nextTargetedGate.ID == 23 || nextTargetedGate.ID == 6)
  {
    wayPointDistance = WAYPOINT_DISTANCE * 8;
  }

  cv::Mat disGateCenterVec = - nextTargetedGate.gateCenter_posi_w + drone_posi_w;
  normDisGateCenter = sqrt(disGateCenterVec.at<double>(0, 0)*disGateCenterVec.at<double>(0, 0) + disGateCenterVec.at<double>(1, 0)*disGateCenterVec.at<double>(1, 0) + disGateCenterVec.at<double>(2, 0)*disGateCenterVec.at<double>(2, 0));
  // If the drone is too close to the gate, stop generate new waypoint
  // std::cout  <<"normDisGateCenter = " << std::endl << normDisGateCenter << std::endl << std::endl;
  if (normDisGateCenter < DISTANCE_TO_GATE_STOP_RENEW_WP)
  {
    return;
  }

  cv::Mat WayPoint1 = nextTargetedGate.gateCenter_posi_w + wayPointDistance * nextTargetedGate.normalVec;
  cv::Mat disWayPoint1 = WayPoint1 - drone_posi_w;
  double normDisWayPoint1 = sqrt(disWayPoint1.at<double>(0, 0)*disWayPoint1.at<double>(0, 0) + disWayPoint1.at<double>(1, 0)*disWayPoint1.at<double>(1, 0) + disWayPoint1.at<double>(2, 0)*disWayPoint1.at<double>(2, 0));

  cv::Mat WayPoint2 = nextTargetedGate.gateCenter_posi_w - wayPointDistance * nextTargetedGate.normalVec;
  cv::Mat disWayPoint2 = WayPoint2 - drone_posi_w;
  double normDisWayPoint2 = sqrt(disWayPoint2.at<double>(0, 0)*disWayPoint2.at<double>(0, 0) + disWayPoint2.at<double>(1, 0)*disWayPoint2.at<double>(1, 0) + disWayPoint2.at<double>(2, 0)*disWayPoint2.at<double>(2, 0));

  if(normDisWayPoint1 < normDisWayPoint2)
  {
    nextTargetedGate.wayPointNear = nextTargetedGate.gateCenter_posi_w; //WayPoint1;
    nextTargetedGate.wayPointNear.at<double>(2, 0) = nextTargetedGate.gateHeight - 0.8;

    nextTargetedGate.wayPointFar = WayPoint2;
    nextTargetedGate.wayPointFar.at<double>(2, 0) = nextTargetedGate.gateHeight - 0.8;

    nextTargetedGate.wayPointVisual = 1;
  }
  else
  {
    nextTargetedGate.wayPointNear = nextTargetedGate.gateCenter_posi_w; //WayPoint2;
    nextTargetedGate.wayPointNear.at<double>(2, 0) = nextTargetedGate.gateHeight - 0.8;

    nextTargetedGate.wayPointFar = WayPoint1;
    nextTargetedGate.wayPointFar.at<double>(2, 0) = nextTargetedGate.gateHeight - 0.8;

    nextTargetedGate.wayPointVisual = 1;
  }
/*
  if(nextTargetedGate.ID == 9)
  {
    std::cout << nextTargetedGate.ID <<" wayPointFar = " << nextTargetedGate.wayPointFar << std::endl;
    std::cout << nextTargetedGate.ID <<" gateCenter_posi_w = " << nextTargetedGate.gateCenter_posi_w << std::endl;
    std::cout << nextTargetedGate.ID <<" normalVec = " << nextTargetedGate.normalVec << std::endl;
  }*/

  //std::cout << nextTargetedGate.ID <<" nextTargetedGate.gateCenter_posi_w = " << std::endl << " " << nextTargetedGate.gateCenter_posi_w << std::endl << std::endl;
  //std::cout << nextTargetedGate.ID <<" nextTargetedGate.normalVec = " << std::endl << " " << nextTargetedGate.normalVec << std::endl << std::endl;
  /*std::cout << nextTargetedGate.ID <<" WayPoint1 = " << std::endl << " " << WayPoint1 << std::endl << std::endl;
  std::cout << nextTargetedGate.ID <<" WayPoint2 = " << std::endl << " " << WayPoint2 << std::endl << std::endl;
  std::cout << nextTargetedGate.ID <<" wayPointNear = " << std::endl << " " << wayPointNear << std::endl << std::endl;
  std::cout << nextTargetedGate.ID <<" wayPointFar = " << std::endl << " " << wayPointFar << std::endl << std::endl; */

  // nextWayPoint

  // TODO: using the drone's location and next waypoint location, calculate the target yaw, as a command to the controller.
  // posi_x - nextWayPoint.at<double>(0, 0)
  // posi_y - nextWayPoint.at<double>(1, 0)
  // yawCommand = tan();

  // TODO: if the drone is safe to to directly to the wayPointFar, change the next way point to the wayPointFar
  
}

void wayPointPlan::switchNextGate(mav_estimation::gate& nextTargetedGate, double posi_x, double posi_y, double posi_z)
{
  // std::cout  <<"gate to pass ID = " << nextTargetedGate.ID << std::endl;

  drone_posi_w = (cv::Mat_<double>(3, 1) << posi_x, posi_y, posi_z);
  cv::Mat disWayPointInitVec = - nextTargetedGate.wayPointInit + drone_posi_w;
  double normDisWayPointInit = sqrt(disWayPointInitVec.at<double>(0, 0)*disWayPointInitVec.at<double>(0, 0) + disWayPointInitVec.at<double>(1, 0)*disWayPointInitVec.at<double>(1, 0) + disWayPointInitVec.at<double>(2, 0)*disWayPointInitVec.at<double>(2, 0));

  switch(nextTargetedGate.ID) 
  {
    case 2: 
      if(normDisWayPointInit > 12 && nextTargetedGate.reachWatchPoint == 0)
      {
        nextWayPointToPublish = nextTargetedGate.wayPointInit;
        // std::cout <<"Gate 2 wayPoint Init Distance: " << normDisWayPointInit << std::endl; 
        return;
      }
      break;

    case 9: 
      if(normDisWayPointInit > 10 && nextTargetedGate.reachWatchPoint == 0)
      {
        nextWayPointToPublish = nextTargetedGate.wayPointInit;
        // std::cout <<"Gate 9 wayPoint Init Distance: " << normDisWayPointInit << std::endl; 
        return;
      }
      break;

    case 23:
      if(normDisWayPointInit > 12 && nextTargetedGate.reachWatchPoint == 0)
      {
        nextWayPointToPublish = nextTargetedGate.wayPointInit;
        // std::cout <<"Gate 23 wayPoint Init Distance: " << normDisWayPointInit << std::endl; 
        // nextTargetedGate.wayPointInit.at<double>(2, 0)
        return;
      }
      break;

    case 14:
      if(normDisWayPointInit > 4)
      {
        // std::cout  <<"normDisWayPointInit = " << normDisWayPointInit << std::endl;
        /*nextWayPointToPublish = nextTargetedGate.wayPointInit;
        std::cout <<"Gate 14 wayPoint Init Distance: " << normDisWayPointInit << std::endl; 
        // nextTargetedGate.wayPointInit.at<double>(2, 0)
        return;*/
      }
      else
      {
        passedGateNum = passedGateNum + 1;
        nextGateID = targetedGateID[passedGateNum];
        // std::cout  <<"NEW !!! Next Gate = " << nextGateID << std::endl;
        
        return;
      }
      break;

    case 22:
      if(normDisWayPointInit > 4)
      {
        // std::cout  <<"normDisWayPointInit = " << normDisWayPointInit << std::endl;
        /* = nextTargetedGate.wayPointInit;
        std::cout <<"Gate 22 wayPoint Init Distance: " << normDisWayPointInit << std::endl; 
        // nextTargetedGate.wayPointInit.at<double>(2, 0)
        return;*/
      }
      else
      {
        passedGateNum = passedGateNum + 1;
        nextGateID = targetedGateID[passedGateNum];
        // std::cout  <<"NEW !!! Next Gate = " << nextGateID << std::endl;
        
        return;
      }
      break;
  }

  /*if(normDisWayPointInit < 1)
  {
    passedGateNum = passedGateNum + 1;
    nextGateID = targetedGateID[passedGateNum];
    std::cout  <<"NEW !!! Next Gate = " << nextGateID << std::endl;
    return;
  }*/

    
  if(nextTargetedGate.wayPointVisual == 0) 
  // the next gate is NOT in sight ever yet, or not arrived at the "watching point"
  {
    nextWayPointToPublish = nextTargetedGate.wayPointInit;
    // std::cout  <<"wayPoint Init, Gate Not in sight !" << std::endl; 
    // std::cout  <<"Gate " << nextTargetedGate.ID << " is not in sight!" << std::endl;
    return;
  }  // have not seen the next gate, fly to the gate and end this function

  nextTargetedGate.reachWatchPoint = 1;  // gate in sight and reached the initial way point

  cv::Mat disWayPointFarVec = - nextTargetedGate.wayPointFar + drone_posi_w;  // from wayPointFar to Drone
  cv::Mat disGateCenterVec = - nextTargetedGate.gateCenter_posi_w + drone_posi_w;  // from gateCenter to Drone

  safePassing = 0;
  double wayPointDistance = WAYPOINT_DISTANCE;

  if (nextTargetedGate.ID == 10 || nextTargetedGate.ID == 2 || nextTargetedGate.ID == 13 || nextTargetedGate.ID == 9 || nextTargetedGate.ID == 15 || nextTargetedGate.ID == 23 || nextTargetedGate.ID == 6)
  {
    wayPointDistance = WAYPOINT_DISTANCE * 8;
  }
  // calculate the safe pass
  cv::Mat gatePlanePoint = wayPointDistance * disWayPointFarVec / abs(disWayPointFarVec.dot(nextTargetedGate.normalVec)) + nextTargetedGate.wayPointFar;
  // std::cout  <<"disWayPointFarVec = " << std::endl << disWayPointFarVec << std::endl;
  // std::cout  <<"abs(disWayPointFarVec.dot(nextTargetedGate.normalVec)) = " << std::endl << abs(disWayPointFarVec.dot(nextTargetedGate.normalVec)) << std::endl;
  // std::cout  <<"nextTargetedGate.wayPointFar = " << std::endl << nextTargetedGate.wayPointFar << std::endl;
  // std::cout  <<"gatePlanePoint = " << std::endl << gatePlanePoint << std::endl;

  cv::Mat vecInGatePlane = nextTargetedGate.gateCenter_posi_w - gatePlanePoint;
  // std::cout  <<"vecInGatePlane = " << std::endl << vecInGatePlane << std::endl;

  double normVecInGatePlane = sqrt(vecInGatePlane.at<double>(0, 0)*vecInGatePlane.at<double>(0, 0) + vecInGatePlane.at<double>(1, 0)*vecInGatePlane.at<double>(1, 0)); // + vecInGatePlane.at<double>(2, 0)*vecInGatePlane.at<double>(2, 0));

  //std::cout  <<"normVecInGatePlane = " << normVecInGatePlane << std::endl;
  //std::cout  <<"nextTargetedGate.gateHeightHalf - 0.3 = " << nextTargetedGate.gateHeightHalf - 0.3 << std::endl;
  //std::cout  <<"height error = " << abs(disWayPointFarVec.at<double>(2, 0)) << std::endl;
  if (normVecInGatePlane < (nextTargetedGate.gateWidthHalf - narrowPath)) 
  {
    safePassing = 1;
  }
  // std::cout <<"safePassing = " << safePassing << std::endl;

  if(safePassing == 1) // passed the next gate's far point
  {
    nextWayPointToPublish = nextTargetedGate.wayPointFar;
    double normDisWayPointFar = sqrt(disWayPointFarVec.at<double>(0, 0)*disWayPointFarVec.at<double>(0, 0) + disWayPointFarVec.at<double>(1, 0)*disWayPointFarVec.at<double>(1, 0) + disWayPointFarVec.at<double>(2, 0)*disWayPointFarVec.at<double>(2, 0));
    double normDisGateCenter = sqrt(disGateCenterVec.at<double>(0, 0)*disGateCenterVec.at<double>(0, 0) + disGateCenterVec.at<double>(1, 0)*disGateCenterVec.at<double>(1, 0) + disGateCenterVec.at<double>(2, 0)*disGateCenterVec.at<double>(2, 0));
    // std::cout  <<"wayPoint  Far" << std::endl; 
    // std::cout  << nextTargetedGate.ID << " Distance to the far Point: " << abs(disWayPointFarVec.dot(nextTargetedGate.normalVec)) << std::endl << std::endl;
    
    if ( abs(disWayPointFarVec.dot(nextTargetedGate.normalVec)) < (wayPointDistance - 0.1) ) // already passed the gate
    {
      if (nextTargetedGate.ID == 6) // if passed the last gate, kill the flight system
      {
        finishTrack = 1;
        return;
        // ros::shutdown();
      }
      passedGateNum = passedGateNum + 1;
      nextGateID = targetedGateID[passedGateNum];
      // std::cout  <<"NEW !!! Next Gate = " << nextGateID << std::endl;
    }
    // std::cout  <<"wayPointFar" << std::endl;
  }
  else
  {
    // std::cout  <<"wayPoint  Near" << std::endl; 
    // std::cout  << nextTargetedGate.ID << " Distance to the far Point = " << abs(disWayPointFarVec.dot(nextTargetedGate.normalVec)) << std::endl;
    nextWayPointToPublish = nextTargetedGate.wayPointNear;
    
    if ( abs(disWayPointFarVec.dot(nextTargetedGate.normalVec)) < (wayPointDistance - 0.1) ) // already passed the gate
    {
      if (nextTargetedGate.ID == 6) // if passed the last gate, kill the flight system
      {
        finishTrack = 1;
        return;
        // ros::shutdown();
      }
      passedGateNum = passedGateNum + 1;
      nextGateID = targetedGateID[passedGateNum];
      // std::cout  <<"NEW !!! Next Gate = " << nextGateID << std::endl;
    }
  }

  double horizonDis2Gate = abs(disGateCenterVec.dot(nextTargetedGate.normalVec));

  if (horizonDis2Gate > 7 && nextTargetedGate.ID == 6) 
  {
    nextWayPointToPublish.at<double>(2, 0) = sin(6.0*D2R) * horizonDis2Gate + nextTargetedGate.gateHeight;
    // std::cout << nextTargetedGate.ID <<" nextTargetedGate.gateHeight = " << nextTargetedGate.gateHeight <<std::endl; 
    // std::cout << nextTargetedGate.ID <<" horizonDis2Gate = " << horizonDis2Gate <<std::endl; 
  }

  if (horizonDis2Gate > 7 && nextTargetedGate.ID == 23) 
  {

    nextWayPointToPublish.at<double>(2, 0) = sin(6.0*D2R) * horizonDis2Gate + nextTargetedGate.gateHeight;
    // std::cout << nextTargetedGate.ID <<" nextTargetedGate.gateHeight = " << nextTargetedGate.gateHeight <<std::endl; 
    // std::cout << nextTargetedGate.ID <<" horizonDis2Gate = " << horizonDis2Gate <<std::endl; 
  }

  if (horizonDis2Gate > 7 && (nextTargetedGate.ID == 9 || nextTargetedGate.ID == 13))
  {

    nextWayPointToPublish.at<double>(2, 0) = sin(6.0*D2R) * horizonDis2Gate + nextTargetedGate.gateHeight;
    // std::cout << nextTargetedGate.ID <<" nextTargetedGate.gateHeight = " << nextTargetedGate.gateHeight <<std::endl; 
    // std::cout << nextTargetedGate.ID <<" horizonDis2Gate = " << horizonDis2Gate <<std::endl; 
  }

  if (horizonDis2Gate > 7 && nextTargetedGate.ID == 10)
  {

    //std::cout << nextTargetedGate.ID <<" nextTargetedGate.gateHeight = " << nextTargetedGate.gateHeight <<std::endl; 
    nextWayPointToPublish.at<double>(2, 0) = sin(2*D2R) * horizonDis2Gate + nextTargetedGate.gateHeight;
    //std::cout << nextTargetedGate.ID <<" horizonDis2Gate = " << horizonDis2Gate <<std::endl; 
  }

  if (horizonDis2Gate > 7 && nextTargetedGate.ID == 2)
  {
    nextWayPointToPublish.at<double>(2, 0) = nextTargetedGate.gateHeight + 1.8;
    // std::cout << nextTargetedGate.ID <<" nextWayPointToPublish = " <<std::endl << nextWayPointToPublish <<std::endl; 
  }

  if (horizonDis2Gate > 7 && nextTargetedGate.ID == 21)
  {
    nextWayPointToPublish.at<double>(2, 0) = nextTargetedGate.gateHeight - 0.4;
  }

  if (horizonDis2Gate > 7 && nextTargetedGate.ID == 15)
  {
    nextWayPointToPublish.at<double>(2, 0) = sin(10*D2R) * horizonDis2Gate + nextTargetedGate.gateHeight;
  }

  if (nextTargetedGate.ID == 1)
  {
    nextWayPointToPublish.at<double>(2, 0) = nextTargetedGate.gateHeight - nextTargetedGate.gateHeightHalf + 0.1;
  }

  // std::cout << nextTargetedGate.ID <<" nextWayPoint Height = " << nextWayPointToPublish.at<double>(2, 0) <<std::endl;

}