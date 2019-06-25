#include "gate.h"

namespace mav_estimation {

gate::gate()// initialize a gate object, when there is one (or more) marker in the image.
{
  // gateID = ID;
  // for(int a = 0; a < 4; a++)
  // {
  //   markerPixel[a][0] = -1;
  //   markerPixel[a][1] = -1;
  // }

  //rvector.assign(3,0);
  //tvec.assign(3,0);

  // cv::Mat cameraMatrix(3, 3, CV_64FC1);// (3, 3, CV_8UC1, 0);
  // cv::Mat allMarkersLocation3d(4, 3, CV_64FC1);
  // cv::Mat MarkerPixel2d(4, 2, CV_64FC1);

  // cameraDistCoeffs = cv::Mat::zeros(8, 1, CV_64FC1);  //zeros
  // cameraDistCoeffs.assign(4,0.0);

  cameraMatrix = (cv::Mat_<float>(3,3) << 548.4088134765625, 0.0, 512.0, 0.0, 548.4088134765625, 384.0, 0.0, 0.0, 1.0);
  
  cameraDistCoeffs = (cv::Mat_<float>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);


  // gate15Marker[12] = {6.04582, -11.79203, 3.30625, 8.453821, -11.79203, 3.30625, 8.453821, -11.79203, 0.7242498, 6.04582, -11.79203, 0.7242498};
  
  gateMarker[0] = gate1Marker;
  gateMarker[1] = gate2Marker;
  gateMarker[2] = gate3Marker;
  gateMarker[3] = gate4Marker;
  gateMarker[4] = gate5Marker;
  gateMarker[5] = gate6Marker;
  gateMarker[6] = gate7Marker;
  gateMarker[7] = gate8Marker;
  gateMarker[8] = gate9Marker;
  gateMarker[9] = gate10Marker;
  gateMarker[10] = gate11Marker;
  gateMarker[11] = gate12Marker;
  gateMarker[12] = gate13Marker;
  gateMarker[13] = gate14Marker;
  gateMarker[14] = gate15Marker;
  gateMarker[15] = gate16Marker;
  gateMarker[16] = gate17Marker;
  gateMarker[17] = gate18Marker;
  gateMarker[18] = gate19Marker;
  gateMarker[19] = gate20Marker;
  gateMarker[20] = gate21Marker;
  gateMarker[21] = gate22Marker;
  gateMarker[22] = gate23Marker;

  //cv::Mat allMarkersLocation3d(4, 3, CV_64FC1, markerGateFrame);

  //std::cout << "allMarkersLocation3d = " << std::endl << " " << allMarkersLocation3d << std::endl << std::endl;

  /*rvecP3P = (cv::Mat_<double>(3,1) << 1.573562545599955, -0.1019043040809555, 0.1218677196514226);  // p3p initial guess of pose
  tvecP3P = (cv::Mat_<double>(3,1) << -16.31899287936908, 6.107653077275284, -10.05880465275848);

  rvec = (cv::Mat_<double>(3,1) << 0, 0, 0);  // p4p initial guess of pose
  tvec = (cv::Mat_<double>(3,1) << 0, 0, 0);*/

  Rc2b = eulerAnglesToRotationMatrix(-90 * D2R, 0, -90 * D2R);
  Rb2c = Rc2b.t();

  // std::cout<< "Rb2c " <<"\n"<< Rb2c <<"\n"<<"\n";

  firstImage = 1;

  wayPointVisual = 0;

  reachWatchPoint = 0;
}

gate::~gate()
{
}

void gate::setMarkerLocation() // MarkerLocation using the yaml file (fake world frame)
{
  markerLocationWorld = gateMarker[ID-1];
  allMarkersLocation3d = (cv::Mat_<double>(4,3) << markerLocationWorld[0], markerLocationWorld[1], markerLocationWorld[2], markerLocationWorld[3], markerLocationWorld[4], markerLocationWorld[5], markerLocationWorld[6], markerLocationWorld[7], markerLocationWorld[8], markerLocationWorld[9], markerLocationWorld[10], markerLocationWorld[11]);
  //gateCenter_posi_fw = (cv::Mat_<double>(3,1) << (allMarkersLocation3d.at<double>(0, 0) + allMarkersLocation3d.at<double>(2, 0))/2, (allMarkersLocation3d.at<double>(0, 1) + allMarkersLocation3d.at<double>(2, 1))/2, (allMarkersLocation3d.at<double>(0, 2) + allMarkersLocation3d.at<double>(2, 2))/2);
  
  Marker1Location3d = (cv::Mat_<double>(3,1) << markerLocationWorld[0], markerLocationWorld[1], markerLocationWorld[2]);
  Marker2Location3d = (cv::Mat_<double>(3,1) << markerLocationWorld[3], markerLocationWorld[4], markerLocationWorld[5]);
  Marker3Location3d = (cv::Mat_<double>(3,1) << markerLocationWorld[6], markerLocationWorld[7], markerLocationWorld[8]);
  Marker4Location3d = (cv::Mat_<double>(3,1) << markerLocationWorld[9], markerLocationWorld[10], markerLocationWorld[11]);

}

bool gate::fixedGate()  // input is the real ID starts from 1
{
  int gateID = ID - 1;
  if (gateID == 2 || gateID == 3 || gateID == 4 || gateID == 6 || gateID == 7 || gateID == 10 || gateID == 11 || (gateID >= 15 && gateID <= 19))
  {
    return 1;
  }else{
    return 0;
  }
}

void gate::relativePoseP4P()  // for single visible gate 
{
  // pnp function  markerPixel

  // MarkerPixel2d = (cv::Mat_<double>(4,2) << markerPixel[0][0], markerPixel[0][1], markerPixel[1][0], markerPixel[1][1], markerPixel[2][0], markerPixel[2][1], markerPixel[3][0], markerPixel[3][1]);
  // MarkerPixel2d.at<double>(0,0) = markerPixel[0][0];
  cv::Mat MarkerPixel2d(4, 2, CV_64FC1, markerPixel);  // TODO ??

  //std::cout << "MarkerPixel2d = " << std::endl << " " << MarkerPixel2d << std::endl << std::endl;
  //std::cout << "allMarkersLocation3d = " << std::endl << " " << allMarkersLocation3d << std::endl << std::endl;
  // copy (cameraDistCoeffs.begin(), cameraDistCoeffs.end(), std::ostream_iterator<int> (std::cout, "\n"));

  cv::solvePnP(allMarkersLocation3d, MarkerPixel2d, cameraMatrix, cameraDistCoeffs, rvec, tvec, 0, cv::SOLVEPNP_ITERATIVE);  // cv::SOLVEPNP_ITERATIVE
  // tvec : in Cam frame, vector from camera to "fake" world(0, 0), this "fake" world frame is the frame in which the corner locations are defined
  // NOT the real world frame for navigation
  cv::Rodrigues(rvec, Rc2fw);
  Rfw2c = Rc2fw.t();
  // std::cout << "Rfw2c = " << std::endl << " " << Rfw2c << std::endl << std::endl;
  Cam_posi_fw = - Rfw2c * tvec;  // camera location in the "fake" world frame
  // std::cout << ID <<" Cam_posi_fw = " << std::endl << " " << Cam_posi_fw << std::endl << std::endl;

  //gateCenter_posi_cam = Rc2fw * (gateCenter_posi_fw - Cam_posi_fw);  
  // Marker1Location3d: the "fake" world frame in which the corner locations are defined
  Marker1_posi_cam = Rc2fw * (Marker1Location3d - Cam_posi_fw);
  Marker2_posi_cam = Rc2fw * (Marker2Location3d - Cam_posi_fw);
  Marker3_posi_cam = Rc2fw * (Marker3Location3d - Cam_posi_fw);
  Marker4_posi_cam = Rc2fw * (Marker4Location3d - Cam_posi_fw);

  gate_posi_cam = Rc2fw * (gateCenter_posi_w - Cam_posi_fw);

  // std::cout << "tvecP4P = " << std::endl << " " << tvec << std::endl << std::endl;
  // std::cout << "rvecP4P = " << std::endl << " " << rvec << std::endl << std::endl;
/*
  std::cout << ID <<" gateCenter_posi_cam = " << std::endl << " " << gateCenter_posi_cam << std::endl << std::endl;

  std::cout << ID <<" Marker1_posi_cam = " << std::endl << " " << Marker1_posi_cam << std::endl << std::endl;
  std::cout << ID <<" Marker2_posi_cam = " << std::endl << " " << Marker2_posi_cam << std::endl << std::endl;
  std::cout << ID <<" Marker3_posi_cam = " << std::endl << " " << Marker3_posi_cam << std::endl << std::endl;
  std::cout << ID <<" Marker4_posi_cam = " << std::endl << " " << Marker4_posi_cam << std::endl << std::endl;  */

  // copy (rvector.begin(), rvector.end(), std::ostream_iterator<int> (std::cout, "\n"));
  // copy (tvec.begin(), tvec.end(), std::ostream_iterator<int> (std::cout, "\n"));

  // cv::solvePnP(allMarkersLocation3d, MarkerPixel2d, cameraMatrix, InputArray cameraDistCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=SOLVEPNP_ITERATIVE );
}

void gate::relativePoseP3P(int invisibleMarker)  // invisibleMarker: 0 ~ 3
{
  // pnp function  markerPixel

  // MarkerPixel2d = (cv::Mat_<double>(4,2) << markerPixel[0][0], markerPixel[0][1], markerPixel[1][0], markerPixel[1][1], markerPixel[2][0], markerPixel[2][1], markerPixel[3][0], markerPixel[3][1]);
  // MarkerPixel2d.at<double>(0,0) = markerPixel[0][0];
  cv::Mat MarkerPixel2d(4, 2, CV_64FC1, markerPixel);  // TODO ??

  cv::Mat Markers3Pixel2d;
  cv::Mat Markers3Location3d;

  //std::cout << "MarkerPixel2d = " << std::endl << " " << MarkerPixel2d << std::endl << std::endl;
  // std::cout << "allMarkersLocation3d = " << std::endl << " " << allMarkersLocation3d << std::endl << std::endl;
  // copy (cameraDistCoeffs.begin(), cameraDistCoeffs.end(), std::ostream_iterator<int> (std::cout, "\n"));

  // std::cout << "P3P allMarkersLocation3d = " << ID << std::endl << " " << allMarkersLocation3d << std::endl << std::endl;

  if (invisibleMarker == 0){
    Markers3Location3d = allMarkersLocation3d.rowRange(1, 4);
    Markers3Pixel2d = MarkerPixel2d.rowRange(1, 4);
    // std::cout << "MarkerPixel2d = " << std::endl << " " << MarkerPixel2d << std::endl << std::endl;
  } else if(invisibleMarker == 3){
    Markers3Location3d = allMarkersLocation3d.rowRange(0, 3);
    Markers3Pixel2d = MarkerPixel2d.rowRange(0, 3);
  } else{
    cv::Mat MarkerPixel2dPart1 = MarkerPixel2d.rowRange(0, invisibleMarker);
    //std::cout << "MarkerPixel2dPart1 = " << std::endl << " " << MarkerPixel2dPart1 << std::endl << std::endl;
    cv::Mat MarkerPixel2dPart2 = MarkerPixel2d.rowRange(invisibleMarker+1, 4);
    //std::cout << "MarkerPixel2dPart2 = " << std::endl << " " << MarkerPixel2dPart2 << std::endl << std::endl;
    cv::vconcat(MarkerPixel2dPart1, MarkerPixel2dPart2, Markers3Pixel2d);
    //std::cout << "Markers3Pixel2d = " << std::endl << " " << Markers3Pixel2d << std::endl << std::endl;
    
    //std::cout << "allMarkersLocation3d = " << std::endl << " " << allMarkersLocation3d << std::endl << std::endl;
    cv::Mat allMarkersLocation3dPart1 = allMarkersLocation3d.rowRange(0, invisibleMarker);
    cv::Mat allMarkersLocation3dPart2 = allMarkersLocation3d.rowRange(invisibleMarker+1, 4);
    
    cv::vconcat(allMarkersLocation3dPart1, allMarkersLocation3dPart2, Markers3Location3d);  
    // this cv function seems to has a bug ... define the cv::Mat Markers3Pixel2d and cv::Mat Markers3Location3d in this function can solve it ... strange ...
    //std::cout << "allMarkersLocation3d = " << std::endl << " " << allMarkersLocation3d << std::endl << std::endl;
  }
  
  
  //std::cout << "Markers3Location3d = " << std::endl << " " << Markers3Location3d << std::endl << std::endl;

// (( (npoints >= 4) || (npoints == 3 && flags == SOLVEPNP_ITERATIVE && useExtrinsicGuess) ) && npoints == std::max(ipoints.checkVector(2, 5), ipoints.checkVector(2, 6)))
  
  /*if (ID == 21)
  {
    rvecP3P = (cv::Mat_<double>(3,1) << 1.573562545599955, -0.1019043040809555, 0.1218677196514226);  // p3p initial guess of pose
    tvecP3P = (cv::Mat_<double>(3,1) << -16.31899287936908, 6.107653077275284, -10.05880465275848);
  }

  if (ID == 14)
  {
    rvecP3P = (cv::Mat_<double>(3,1) << 0.07405566459568054, 2.181839048851835, -2.190959239264372);  // p3p initial guess of pose
    tvecP3P = (cv::Mat_<double>(3,1) << -11.31766691663873, 3.446645218871922, -15.36615719052386);
  }*/

  cv::solvePnP(Markers3Location3d, Markers3Pixel2d, cameraMatrix, cameraDistCoeffs, rvecP3P, tvecP3P, true, cv::SOLVEPNP_ITERATIVE);  // cv::SOLVEPNP_ITERATIVE
  // tvec : in Cam frame, vector from camera to "fake" world(0, 0), this "fake" world frame is the frame in which the corner locations are defined
  // NOT the real world frame for navigation
  cv::Rodrigues(rvecP3P, Rc2fw);
  Rfw2c = Rc2fw.t();

  // std::cout << "tvecP3P = " << std::endl << " " << tvecP3P << std::endl << std::endl;
  // std::cout << "rvecP3P = " << std::endl << " " << rvecP3P << std::endl << std::endl;

  // std::cout << "Rfw2c = " << std::endl << " " << Rfw2c << std::endl << std::endl;
  Cam_posi_fw = - Rfw2c * tvecP3P;  // camera location in the "fake" world frame
  // std::cout << ID <<" Cam_posi_fw = " << std::endl << " " << Cam_posi_fw << std::endl << std::endl;

  //gateCenter_posi_cam = Rc2fw * (gateCenter_posi_fw - Cam_posi_fw);  
  // Marker1Location3d: the "fake" world frame in which the corner locations are defined
  Marker1_posi_cam = Rc2fw * (Marker1Location3d - Cam_posi_fw); // colume vector
  Marker2_posi_cam = Rc2fw * (Marker2Location3d - Cam_posi_fw);
  Marker3_posi_cam = Rc2fw * (Marker3Location3d - Cam_posi_fw);
  Marker4_posi_cam = Rc2fw * (Marker4Location3d - Cam_posi_fw);
  gate_posi_cam = Rc2fw * (gateCenter_posi_w - Cam_posi_fw);
}


/*void gate::targetGateLocation()
{
  gateCenter_posi_cam = Rc2w * (gateCenter_posi_fw - Cam_posi_fw);

}*/

void gate::reset()
{
  // directionVecCamFrame[3];
  // positionVecCamFrame[3];
  // gateID = 0;
  gateVisibility = 0;
  visibleMarkerAmount = 0;
  markerVisibility[0] = 0;
  markerVisibility[1] = 0;
  markerVisibility[2] = 0;
  markerVisibility[3] = 0;
  // markerPixel[4][2];
  // markerGateFrame[4][2];
 
}

void gate::markerLocationRealWorldFrame()
{
  
  //Rb2w = eulerAnglesToRotationMatrix(roll_drone_r, pitch_drone_r, yaw_drone_r);
  //Rw2b = Rb2w.t(); // Rotation matrix transfer vector from body frame to real world frame

  //body_posi_w = (cv::Mat_<double>(3,1) << x_drone, y_drone, height_drone);
  /*marker1LocationRealWorldFrame = Rw2b * Rb2c * Marker1_posi_cam + body_posi_w; // 3 * 1 colume vector
  marker2LocationRealWorldFrame = Rw2b * Rb2c * Marker2_posi_cam + body_posi_w;
  marker3LocationRealWorldFrame = Rw2b * Rb2c * Marker3_posi_cam + body_posi_w;
  marker4LocationRealWorldFrame = Rw2b * Rb2c * Marker4_posi_cam + body_posi_w;*/

  marker1LocationRealWorldFrame = Marker1Location3d; // 3 * 1 colume vector, pre-fixed value
  marker2LocationRealWorldFrame = Marker2Location3d;
  marker3LocationRealWorldFrame = Marker3Location3d;
  marker4LocationRealWorldFrame = Marker4Location3d;

  gateCenter_posi_w = (marker1LocationRealWorldFrame + marker2LocationRealWorldFrame + marker3LocationRealWorldFrame + marker4LocationRealWorldFrame) / 4;

  gateHeightHalf = std::abs(marker1LocationRealWorldFrame.at<double>(2, 0) - gateCenter_posi_w.at<double>(2, 0));

  gateWidthHalf = gateHeightHalf;

  if (ID == 14 || ID == 22)
  {
    gateWidthHalf = 2.0;
  }

  if (ID == 2)
  {
    gateWidthHalf = 1.5;
  }

  if (ID == 21)
  {
    gateWidthHalf = 1.2;
  }

  // std::cout << ID << " " <<marker1LocationRealWorldFrame.at<double>(2, 0) << "  " <<gateCenter_posi_w.at<double>(2, 0) << std::endl;
  // std::cout << ID <<" gateHeightHalf = " << gateHeightHalf << std::endl;

  gateHeight = gateCenter_posi_w.at<double>(2, 0) + gateHeightHalf;

  
  normalVec = (marker1LocationRealWorldFrame - marker3LocationRealWorldFrame).cross(marker1LocationRealWorldFrame - marker2LocationRealWorldFrame);
  double normNormalVec = sqrt(normalVec.at<double>(0, 0)*normalVec.at<double>(0, 0) + normalVec.at<double>(0, 1)*normalVec.at<double>(0, 1) + normalVec.at<double>(0, 2)*normalVec.at<double>(0, 2));
  normalVec = normalVec / normNormalVec;

  /*std::cout  <<" Rw2b = " << Rw2b << std::endl;
  std::cout  <<" Rb2c = " << Rb2c << std::endl;

  std::cout  <<" body_posi_w = " << body_posi_w << std::endl;*/
  // std::cout  <<" gateCenter_posi_w = " << gateCenter_posi_w << std::endl;

  /*std::cout << gateID+1 <<" marker1LocationRealWorldFrame = " << std::endl << " " << marker1LocationRealWorldFrame << std::endl << std::endl;
  std::cout << gateID+1 <<" marker2LocationRealWorldFrame = " << std::endl << " " << marker2LocationRealWorldFrame << std::endl << std::endl;
  std::cout << gateID+1 <<" marker3LocationRealWorldFrame = " << std::endl << " " << marker3LocationRealWorldFrame << std::endl << std::endl;
  std::cout << gateID+1 <<" marker4LocationRealWorldFrame = " << std::endl << " " << marker4LocationRealWorldFrame << std::endl << std::endl;
*/
}

void gate::resetMarkersLocation3dRealWorldFrame()
{
  allMarkersLocation3d = (cv::Mat_<double>(4,3) << marker1LocationRealWorldFrame.at<double>(0, 0), marker1LocationRealWorldFrame.at<double>(1, 0), marker1LocationRealWorldFrame.at<double>(2, 0), marker2LocationRealWorldFrame.at<double>(0, 0), marker2LocationRealWorldFrame.at<double>(1, 0), marker2LocationRealWorldFrame.at<double>(2, 0), marker3LocationRealWorldFrame.at<double>(0, 0), marker3LocationRealWorldFrame.at<double>(1, 0), marker3LocationRealWorldFrame.at<double>(2, 0), marker4LocationRealWorldFrame.at<double>(0, 0), marker4LocationRealWorldFrame.at<double>(1, 0), marker4LocationRealWorldFrame.at<double>(2, 0));
  // std::cout  <<"Reset allMarkersLocation3d = " << ID << std::endl << allMarkersLocation3d << std::endl;
  // allMarkersLocation3d = (cv::Mat_<double>(4,3) << markerLocationWorld[0], markerLocationWorld[1], markerLocationWorld[2], markerLocationWorld[3], markerLocationWorld[4], markerLocationWorld[5], markerLocationWorld[6], markerLocationWorld[7], markerLocationWorld[8], markerLocationWorld[9], markerLocationWorld[10], markerLocationWorld[11]);
  //gateCenter_posi_fw = (cv::Mat_<double>(3,1) << (allMarkersLocation3d.at<double>(0, 0) + allMarkersLocation3d.at<double>(2, 0))/2, (allMarkersLocation3d.at<double>(0, 1) + allMarkersLocation3d.at<double>(2, 1))/2, (allMarkersLocation3d.at<double>(0, 2) + allMarkersLocation3d.at<double>(2, 2))/2);
}

cv::Mat gate::eulerAnglesToRotationMatrix(double roll, double pitch, double yaw)
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
               -sin(yaw),    cos(yaw),       0,
               0,               0,             1
               );
     
    // Combined rotation matrix
    cv::Mat R = R_x * R_y * R_z;
     
    return R;
}






}