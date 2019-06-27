#include "stdio.h"
#include "iostream"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

bool imuFirstMsg = 1;
double R2D = 57.2957795;
double D2R = 0.01745329;

#define DR_FILTER_GRAVITY  9.81
#define DR_FILTER_DRAG  0.5
#define DR_FILTER_THRUSTCORR  1.0
void ahrs(double gyro_x, double gyro_y, double gyro_z, double acc_x, double acc_y, double acc_z, double dt);
void filter_predict(double roll, double pitch, double yaw, double dt, double acc_x, double acc_y, double acc_z);


// RANSAC Measurement buffer size
#define  RANSAC_BUF_SIZE   50

// NOTE: the following defines determine how often there will be a RANSAC fit and a correction.

// number of measurements necessary for a fit:
#define RANSAC_MIN_SAMPLES_FOR_FIT 8
// used to be 1.0 for max and 0.5 for no vision
// max delta time (DT) for a measurement to be considered in the RANSAC buffer
#define RANSAC_DT_MAX 2.5
// delta time (DT) after which RANSAC decides that there are no vision measurements any more
// the correction will then be applied and the buffer emptied
#define RANSAC_DT_NO_VISION 0.5

// void odom_cb(const nav_msgs::Odometry &odom);
void pnp_cb(const geometry_msgs::Vector3Stamped &pnp);
void imu_cb(const sensor_msgs::Imu &imuMsg);
// void tf_cb(const tf2_msgs::TFMessage &tf);

bool crash = 0;
void crash_cb(const std_msgs::Empty &crash_msg);

bool finishTrack = 0;
void finishTrack_cb(const std_msgs::Empty &finishTrack_msg);

// Buffer Data Type
struct ransac_buffer_t
{
    // Settings
    double time;

    // Predicted States
    double x;
    double y;
    double z;

    // Measured States
    double mx;
    double my;
    double mz;
};

// Datatype
struct ransac_t
{
    // Settings
    double dt_max;       ///< Maximum time that a vision sample can stay in the fitting buffer
    double dt_novision;    ///< Maximum time that no vision updates are received before the ransac is reset

    // States
    int buf_index_of_last;  ///< index of the last vision sample in the rolling buffer
    int buf_size;       ///< nr of elements in the rolling vision sample buffer

    double corr_x;       ///< result of RANSAC: a correction on the current state
    double corr_y;
    double corr_z;

    double corr_vx;
    double corr_vy;
    double corr_vz;

    int ransac_cnt;     ///< How many times was RANSAC run
};

/* filter.h */
struct state_t {
  // Time
  double time;

  // Positon
  double x;
  double y;
  double z;

  // Speed
  double vx;
  double vy;
  double vz;

  double yaw;
  double pitch;
  double roll;

  // Heading
  double psi;

  // Logic
  int assigned_gate_index;
};

struct state_t dr_state;
struct ransac_t dr_ransac;
    
// Variables
struct ransac_buffer_t ransac_buf[RANSAC_BUF_SIZE];

// Reset
void ransac_reset(void);

// Correct the state predictions with the help of
void correct_state(void);

// On new IMU measurement: PREDICT
void ransac_propagate(void);

// On new vision update: PUSH a measurement update
void ransac_push(double mt, double mx, double my, double mz);

/** Perform RANSAC to fit a linear model.
 *
 * @param[in] n_samples The number of samples to use for a single fit
 * @param[in] n_iterations The number of times a linear fit is performed
 * @param[in] error_threshold The threshold used to cap errors in the RANSAC process
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit
 */
void ransac_linear_model(int n_samples, int n_iterations, double error_threshold, double *targets,
                         double *samples, unsigned int count, double *params, double *fit_error);

/** Get indices without replacement.
 *
 * @param[out] indices_subset This will be filled with the sampled indices
 * @param[in] n_samples The number of samples / indices.
 * @param[in] count The function will sample n_sample numbers from the range 1, 2, 3,..., count
 */
void get_indices_without_replacement(int *indices_subset, int n_samples, int count);

/* modifies global variables of dronestate and ransac buffer*/
void ransac_correct();