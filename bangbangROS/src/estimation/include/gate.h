#ifndef GATE_H
#define GATE_H

//#include <stdio.h>
//#include <memory.h>
//#include <ros/callback_queue.h>
//#include <Eigen/Eigen>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// Messages
#include <mav_msgs/RateThrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <rosgraph_msgs/Clock.h>

#include <sstream>
#include <fstream>
#include <string>
#include <iostream>

#include <estimation/IRMarkerArray.h>
#include <estimation/IRMarker.h>

// #include <cv/calib3d.h>
#include <opencv2/opencv.hpp>
#include <vector>
// using namespace cv;

# define pi 3.14159265
# define R2D 57.2957795
# define D2R 0.01745329

#define DR_FILTER_GRAVITY  9.81
#define DR_FILTER_DRAG  0.5
#define DR_FILTER_THRUSTCORR  1.0

namespace mav_estimation {

class gate
{
public:
    gate();
    ~gate();

    void reset();
    void setMarkerLocation();

    void relativePoseP4P();
    void relativePoseP3P(int invisibleMarker);

    bool fixedGate();


    std::string landmarkID; // std_msgs/String

    std::string markerID; // std_msgs/String

    double directionVecCamFrame[3];
    double positionVecCamFrame[3];

    double gateHeightHalf;
    double gateWidthHalf;
    double gateHeight;

    // float32 x_pixel;
    // float32 y_pixel;

    int ID;
    int visibleMarkerAmount;
    bool markerVisibility[4];
    bool gateVisibility;
    double markerPixel[4][2];
    double markerGateFrame[4][3];

    bool firstImage;

    cv::Mat tvec; //(3, 1, CV_64FC1);  in Cam frame, vector from camera to  "fake" world(0, 0)
    cv::Mat Cam_posi_fw;  // in "fake" world frame, camera's position
    cv::Mat gateCenter_posi_fw; // in "fake" world frame, gate Center's position
    cv::Mat gateCenter_posi_cam; // in camera frame, gate Center's position

    cv::Mat Marker1_posi_cam;
    cv::Mat Marker2_posi_cam;
    cv::Mat Marker3_posi_cam;
    cv::Mat Marker4_posi_cam;

    cv::Mat Marker1Location3d;
    cv::Mat Marker2Location3d;
    cv::Mat Marker3Location3d;
    cv::Mat Marker4Location3d;

    cv::Mat marker1LocationRealWorldFrame;
    cv::Mat marker2LocationRealWorldFrame;
    cv::Mat marker3LocationRealWorldFrame;
    cv::Mat marker4LocationRealWorldFrame;

    cv::Mat gateCenter_posi_w;

    cv::Mat gate_posi_cam;

    cv::Mat normalVec;

    cv::Mat Rb2w;
    cv::Mat Rw2b;

    cv::Mat Rc2b;
    cv::Mat Rb2c;

    cv::Mat body_posi_w;

    //  cv::Mat Markers3Location3d;
    //  cv::Mat Markers3Pixel2d;

    void markerLocationRealWorldFrame();
    void resetMarkersLocation3dRealWorldFrame();
    cv::Mat eulerAnglesToRotationMatrix(double roll, double pitch, double yaw);

// waypoints for the gate
    cv::Mat wayPointInit;
    cv::Mat wayPointNear;
    cv::Mat wayPointFar;
    bool wayPointVisual;

    bool reachWatchPoint;
    // if wayPointVisual == 1, means the targeted gate was already in sight and the wayPointNear and wayPointFar are ready.
// waypoints for the gate *end



    // cv::Mat cameraMatrix;
    cv::Mat cameraMatrix;

    cv::Mat allMarkersLocation3d;
    // cv::Mat MarkerPixel2d;
    // std::vector<double> rvec;
    // std::vector<double> tvec;

    cv::Mat cameraDistCoeffs;

    cv::Mat rvec;  //(3, 1, CV_64FC1);
    cv::Mat Rfw2c;  // transfer vector from camera to world
    cv::Mat Rc2fw;  // transfer vector from world to camera
    // cv::Mat tvec; //(3, 1, CV_64FC1);

    cv::Mat rvecP3P;
    cv::Mat tvecP3P;
  private:
    double *gateMarker[23];
// using the norminal gate locations file:  still gates
    double gate3Marker[12] = {-0.3703381, -12.99827, 4.728, -0.3699976, -11.45827, 4.728, -0.3699976, -11.45827, 3.177999, -0.3703381, -12.99827, 3.177999};
    double gate4Marker[12] = {-31.56487, -13.929, 6.99714, -30.04735, -13.929, 7.259325, -29.78347, -13.929, 5.731955, -31.30098, -13.929, 5.469769};
    double gate5Marker[12] = {-20.94317, 14.607, 2.515013, -20.94283, 16.147, 2.515013, -20.94283, 16.147, 0.965013, -20.94317, 14.607, 0.965013};
    double gate7Marker[12] = {16.2506, 41.63, 11.32092, 17.79061, 41.63, 11.32092, 17.79061, 41.63, 9.770914, 16.2506, 41.63, 9.770914};
    double gate8Marker[12] = {-3.430186, 11.79953, 6.122593, -3.430186, 12.97997, 6.122593, -3.430186, 11.79952, 4.911986, -3.430186, 12.97997, 4.911986};
    double gate11Marker[12] = {-27.01367, 2.543159, 7.515942, -28.81367, 2.543289, 7.515942, -27.01367, 2.543159, 5.669941, -28.81367, 2.543289, 5.669941};
    double gate12Marker[12] = {18.40823, 13.60605, 9.143385, 16.33437, 14.09245, 9.143507, 18.90706, 15.73291, 9.14324, 16.83319, 16.21931, 9.143364};
    double gate16Marker[12] = {-13.73218, 28.64197, 12.65825, -13.73218, 26.23396, 12.65825, -13.73218, 26.23396, 10.07625, -13.73218, 28.64197, 10.07625};
    double gate17Marker[12] = {-20.57, -13.29803, 3.46425, -20.57, -15.70603, 3.46425, -20.57, -15.70603, 0.8822498, -20.57, -13.29803, 0.8822498};
    double gate18Marker[12] = {13.51782, 21.76397, 3.61825, 13.51782, 19.35596, 3.61825, 13.51782, 19.35596, 1.03625, 13.51782, 21.76397, 1.03625};
    double gate19Marker[12] = {15.96136, -17.26184, 12.31488, 18.36892, -17.26184, 12.36067, 18.41802, -17.26184, 9.779139, 16.01046, -17.26184, 9.733349};
    double gate20Marker[12] = {-25.39118, 25.019, 7.856133, -22.98318, 25.019, 7.856133, -22.98318, 25.019, 5.274133, -25.39118, 25.019, 5.274133};
// using the norminal gate locations file:  moving gates' norminal locations
    double gate1Marker[12] = {-0.009000421, -32.9505, 3.071861, -0.009000659, -34.8755, 3.071861, -0.009000659, -34.8755, 1.134362, -0.009000421, -32.9505, 1.134362};
    double gate2Marker[12] = {-0.2551794, 27.86797, 4.16025, 4.433571, 27.86797, 4.16025, 4.433571, 27.86797, 0.9327497, -0.2551794, 27.86797, 0.9327497};
    //{4.431075, 28.28157, 4.16025, -0.2521133, 28.50989, 4.16025, -0.2521133, 28.50989, 0.9327497, 4.431075, 28.28157, 0.9327497};

    double gate6Marker[12] = {-9.14867, 30.62316, 3.820941, -10.94867, 30.62329, 3.820941, -9.148668, 30.62316, 1.974941, -10.94867, 30.62329, 1.974941};
    double gate9Marker[12] = {-6.40867, -12.13678, 4.152941, -8.208672, -12.13678, 4.152941, -6.408669, -12.13678, 2.306941, -8.208672, -12.13678, 2.306941};

    double gate10Marker[12] = {18.15111, 3.631447, 7.229498, 16.35111, 3.63155, 7.229498, 18.15111, 3.631447, 5.383497, 16.35111, 3.63155, 5.383497};
    double gate13Marker[12] = {1.237332, 9.001728, 2.9625, 3.162332, 9.001728, 2.9625, 3.162332, 9.001728, 1.025, 1.237332, 9.001728, 1.025};
    double gate14Marker[12] = {-10.4501, -31.31195, 4.25625, -6.545984, -27.40784, 4.25625, -6.545984, -27.40784, 1.02875, -10.4501, -31.31195, 1.02875};
    double gate15Marker[12] = {5.744822, -11.79203, 4.16025, 8.75482, -11.79203, 4.16025, 8.75482, -11.79203, 0.9327497, 5.744822, -11.79203, 0.9327497};
    double gate21Marker[12] = {15.15353, 37.63426, 8.376249, 18.43008, 41.25023, 8.376249, 18.43008, 41.25023, 5.14875, 15.15353, 37.63426, 5.14875};
    double gate22Marker[12] = {4.798321, -27.37053, 4.18125, 8.70332, -31.27553, 4.18125, 8.70332, -31.27553, 0.9537499, 4.798321, -27.37053, 0.9537499};

    double gate23Marker[12] = {-9.328671, 7.773174, 2.942941, -11.12867, 7.773277, 2.942941, -9.328669, 7.773174, 1.096941, -11.12867, 7.773277, 1.096941};
    double *markerLocationWorld;

    // double gate15Marker[12];
// #ifndef gate_Marker  // static ??
// #define gate_Marker
    // double gate15Marker[12] = {6.04582, -11.79203, 3.30625, 8.453821, -11.79203, 3.30625, 8.453821, -11.79203, 0.7242498, 6.04582, -11.79203, 0.7242498};
// #endif



    // void positionVecCamFramePnP(double & markerPixel[4][2]);


};

}

#endif
