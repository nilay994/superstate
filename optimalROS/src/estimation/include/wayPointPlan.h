#ifndef WAYPOINT_PLAN_H
#define WAYPOINT_PLAN_H

#include "gate.h"

#define WAYPOINT_DISTANCE  1.5 // WAYPOINT_DISTANCE to the gate center
#define DISTANCE_TO_GATE_STOP_RENEW_WP  3 // WAYPOINT_DISTANCE to the gate center

#define narrowPath 0.4

using namespace std;

class wayPointPlan
{
public:
    wayPointPlan();
    ~wayPointPlan();

    void setGateInitialWayPoint(mav_estimation::gate *gatePointer);

    void wayPointGeneration(mav_estimation::gate& nextTargetedGate, double posi_x, double posi_y, double posi_z);
    void switchNextGate(mav_estimation::gate& nextTargetedGate, double posi_x, double posi_y, double posi_z);

    cv::Mat nextWayPointToPublish;
    cv::Mat drone_posi_w;

    int targetedGateID[11] = {10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6};  // in sequence, 0 means finished

    int nextGateID;  // next gate to pass

    cv::Mat wayPointToSeeGates[11];

    int passedGateNum;

    bool safePassing;

    double normDisGateCenter;

    bool finishTrack;

    // flight plan
    // array of way points: 0. fate ID, 1. the point to see the gate, 2. the point in front of the gate, and 3. behind the gate.
    // gate *gateWayPointPointer;
};

#endif
