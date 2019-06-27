#include <ros/ros.h>

#include "ransac.h"
// #include "fifo.h"

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

ros::Publisher gt_est;
ros::Publisher odom_pub;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ransac_node");

  ros::NodeHandle nh;

  // initialize publisher 
  // publish filtered states
  
  gt_est = nh.advertise<nav_msgs::Odometry>("gt_est", 1000);

  dr_state.vx = 0;
  dr_state.vy = 0;
  dr_state.vz = 0;

  dr_state.x  = 18.0;
  dr_state.y  = -23.0;
  dr_state.z  = 5.3;

  dr_state.yaw   = 1.57049;
  dr_state.pitch = 0;
  dr_state.roll  = 0;

  // populate ransac settings 
  ransac_reset();

  // initialize subscribers
  ros::Subscriber pnp_sub;
  // ros::Subscriber tf_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber crash_sub;
  ros::Subscriber finishTrack_sub;

  pnp_sub  = nh.subscribe("/pnpMeas", 1000, pnp_cb);
  // tf_sub   = nh.subscribe("/tf", 1000, tf_cb);
  imu_sub  = nh.subscribe("/uav/sensors/imu", 1000, imu_cb);
  crash_sub = nh.subscribe("/uav/collision", 1000, crash_cb);
  finishTrack_sub = nh.subscribe("/finishTrack", 1000, finishTrack_cb);

  finishTrack = 0; 
  crash = 0;

  while (ros::ok() && finishTrack == 0 && crash == 0)  { 
    ros::spinOnce();
  }
}

void crash_cb(const std_msgs::Empty &crash_msg) {
  crash = 1;
}

void finishTrack_cb(const std_msgs::Empty &finishTrack_msg) {
  finishTrack = 1;
}

/*void tf_cb(const tf2_msgs::TFMessage &tf)
{
  double vtime = tf.transforms[0].header.stamp.toSec();
  double x     = tf.transforms[0].transform.translation.x; 
  double y     = tf.transforms[0].transform.translation.y;
  double z     = tf.transforms[0].transform.translation.z;

  double qx_gt = tf.transforms[0].transform.rotation.x;
  double qy_gt = tf.transforms[0].transform.rotation.y;
  double qz_gt = tf.transforms[0].transform.rotation.z;
  double qw_gt = tf.transforms[0].transform.rotation.w;

  double roll_gt_d = atan2(2*qx_gt*qw_gt+2*qy_gt*qz_gt , 1 - 2*qx_gt*qx_gt - 2*qy_gt*qy_gt) * R2D;
  double pitch_gt_d = asin(2*qw_gt*qy_gt - 2*qz_gt*qx_gt) * R2D;
  double yaw_gt_d = atan2(2*qy_gt*qx_gt+2*qw_gt*qz_gt , 1 - 2*qy_gt*qy_gt - 2*qz_gt*qz_gt) * R2D;

  fprintf(tf_f, "%f,%f,%f,%f,%f,%f,%f\n", vtime, x, y, z, roll_gt_d, pitch_gt_d, yaw_gt_d); //, vx, vy, vz);
}*/


double dt_imu = 1/960;
double timeStamp_imu_old = 0;

/* imu callback for model prediction */
void imu_cb(const sensor_msgs::Imu &imuMsg)
{
  // imu_time = imuMsg.header.stamp.toSec(); 
  double gyro_x = imuMsg.angular_velocity.x;
  double gyro_y = imuMsg.angular_velocity.y;
  double gyro_z = imuMsg.angular_velocity.z;

  double acc_x = imuMsg.linear_acceleration.x;
  double acc_y = imuMsg.linear_acceleration.y;
  double acc_z = imuMsg.linear_acceleration.z;

  // don't use static implementations
  if (imuFirstMsg) {
    dt_imu = 1 / 960;
    imuFirstMsg = 0;

    dr_state.vx = 0;
    dr_state.vy = 0;
    dr_state.vz = 0;

    dr_state.x  = 18.0;
    dr_state.y  = -23.0;
    dr_state.z  = 5.3;

    dr_state.yaw   = 1.57049;
    dr_state.pitch = 0;
    dr_state.roll  = 0;
  }
  else {
    dt_imu = imuMsg.header.stamp.toSec() - timeStamp_imu_old;
  }

  dr_state.time = imuMsg.header.stamp.toSec();

  // ahrs update dr.state: yaw pitch roll
  ahrs(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, dt_imu);

  // update the global struct with linear positions and velocities in world frame
  //  with current predictions using orientations
  filter_predict(dr_state.roll, dr_state.pitch, dr_state.yaw, dt_imu, acc_x, acc_y, acc_z);  

  // consider measurement which is only 2seconds (dt_max) seconds old
  ransac_propagate();

  nav_msgs::Odometry pos;

  pos.header.stamp = imuMsg.header.stamp;
  pos.pose.pose.position.x = dr_state.x;
  pos.pose.pose.position.y = dr_state.y;
  pos.pose.pose.position.z = dr_state.z;
  pos.twist.twist.linear.x = dr_state.vx;
  pos.twist.twist.linear.y = dr_state.vy;
  pos.twist.twist.linear.z = dr_state.vz;
  pos.twist.twist.angular.x = dr_state.roll; 
  pos.twist.twist.angular.y = dr_state.pitch; 
  pos.twist.twist.angular.z = dr_state.yaw;
  gt_est.publish(pos);
  
  timeStamp_imu_old = imuMsg.header.stamp.toSec();

}

/* ransac reverse buffer. get_index(1) would imply the most previous element,
/ corresponding to 30 - 1 = 29th element in the ransac buffer */
inline int get_index(int element)
{
    int ind = dr_ransac.buf_index_of_last - element;

    // wrap around by max buffer length if element is older than current bufferlength
    if (ind < 0) { 
      ind += RANSAC_BUF_SIZE; 
    }
    return ind;
}

/* predict position and velocity in world frame given the orientations */
void filter_predict(double roll, double pitch, double yaw, double dt, double acc_x, double acc_y, double acc_z)  // rad
{

  // Body accelerations
  double a_thrust = DR_FILTER_GRAVITY / cos(pitch * DR_FILTER_THRUSTCORR) / cos(roll * DR_FILTER_THRUSTCORR);

  // convert body to world
  double ax = (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) * a_thrust - dr_state.vx * DR_FILTER_DRAG;
  ax = cos(pitch)*cos(yaw) * acc_x + (cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw)) * acc_y + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) * acc_z;
  double ay = (cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll)) * a_thrust - dr_state.vy * DR_FILTER_DRAG;
  ay = cos(pitch)*sin(yaw) * acc_x + (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) * acc_y + (cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll)) * acc_z;
  double az = -sin(pitch) * acc_x + cos(pitch)*sin(roll) * acc_y + cos(pitch)*cos(roll) * acc_z - DR_FILTER_GRAVITY;
  
  // dr_state.time = imutime;
  dr_state.vx += ax * dt;
  dr_state.vy += ay * dt;
  dr_state.vz += az * dt;
  dr_state.x += dr_state.vx * dt;
  dr_state.y += dr_state.vy * dt;
  dr_state.z += dr_state.vz * dt;

  // fprintf(predict, "%f,%f,%f,%f\n", dr_state.time, dr_state.x, dr_state.y, dr_state.z);

}


/* ahrs converts the IMU data to rotation matrix between body and world */
void ahrs(double gyro_x, double gyro_y, double gyro_z, double acc_x, double acc_y, double acc_z, double dt)
{
  // gravity in body frame
  tf2::Vector3 gB = {-sin(dr_state.pitch)*DR_FILTER_GRAVITY, sin(dr_state.roll)*cos(dr_state.pitch)*DR_FILTER_GRAVITY, cos(dr_state.roll)*cos(dr_state.pitch)*DR_FILTER_GRAVITY};
  double norm_gB = sqrt(tf2::tf2Dot(gB, gB));

  tf2::Vector3 gB_scaled = gB/norm_gB;
  
  tf2::Vector3 acc = {acc_x, acc_y, acc_z};
  double norm_acc = sqrt(tf2::tf2Dot(acc, acc));
  tf2::Vector3 acc_scaled = acc/norm_acc;

  tf2::Vector3 error = tf2::tf2Cross(acc_scaled, gB_scaled);
  //printf("error: %f,%f,%f\n", error.getX(), error.getY(), error.getZ());
  static tf2::Vector3 sum_error_ahrs = {0,0,0};
  sum_error_ahrs = sum_error_ahrs + error*dt;

  float k_p = 0.0001;
  float k_i = 0.0;


  if ( norm_acc > 8.5 && norm_acc < 10.8 )
  {
    k_p = 0.1;
  }

  tf2::Vector3 imu_pqr = {gyro_x, gyro_y, gyro_z};

  // verified operator overloading.
  tf2::Vector3 pqr = k_p * error + k_i * sum_error_ahrs + imu_pqr;

  // give phi theta psi from gyro's p q r 
  tf2::Vector3 RotMatVel;
  RotMatVel = {pqr.getX() + pqr.getY()*tan(dr_state.pitch)*sin(dr_state.roll) + pqr.getZ()*tan(dr_state.pitch)*cos(dr_state.roll), 
              pqr.getY()*cos(dr_state.roll) - pqr.getZ()*sin(dr_state.roll),
              pqr.getY()*sin(dr_state.roll)/cos(dr_state.pitch) + pqr.getZ()*cos(dr_state.roll)/cos(dr_state.pitch)};

  tf2::Vector3 att = {dr_state.roll, dr_state.pitch, dr_state.yaw};
  // printf("att: %f,%f,%f\n", att.getX(), att.getY(), att.getZ());
  
  att = att + RotMatVel * dt;
  //printf("RotMatVel: %f,%f,%f\n", RotMatVel.getX(), RotMatVel.getY(), RotMatVel.getZ());

  dr_state.roll  = att.getX();
  dr_state.pitch = att.getY();
  dr_state.yaw   = att.getZ();
  float pi = 3.14159265;
  if(dr_state.yaw > pi)
  {
    dr_state.yaw = dr_state.yaw - 2*pi;
  }
  else if (dr_state.yaw < -pi)
  {
    dr_state.yaw = dr_state.yaw + 2*pi;
  }
  else
  {
    dr_state.yaw = dr_state.yaw + 0;
  }
  // fprintf(imu_f, "%f,%f,%f,%f\n", dr_state.time, dr_state.roll * R2D, dr_state.pitch * R2D,  dr_state.yaw * R2D);
}


void pnp_cb(const geometry_msgs::Vector3Stamped &pnp)
{
  // receive IR markers and store them via ransac_push
  double meas_t = pnp.header.stamp.toSec();
  double meas_x = pnp.vector.x;
  double meas_y = pnp.vector.y;
  double meas_z = pnp.vector.z;
  // fprintf(pnp_f, "%f,%f,%f,%f\n", meas_t, meas_x, meas_y, meas_z);
  
  // TODO: suspected bug, ransac_correct might not be called for every ransac_push
  // push measurements to a buffer
  ransac_push(meas_t, meas_x, meas_y, meas_z);
  
  // call the SVD functions, gather if the fit is within range, update the intial conditions of the integrator in filter_predict()
  ransac_correct();
  // corr_x and corr_vx are now updated

}

void ransac_reset()
{
    // populate the setting
    dr_ransac.dt_max      = RANSAC_DT_MAX;
    dr_ransac.dt_novision = RANSAC_DT_NO_VISION;

    // Reset the buffer
    dr_ransac.buf_index_of_last = RANSAC_BUF_SIZE-1;
    dr_ransac.buf_size = 0;

    for (int i=0; i<RANSAC_BUF_SIZE; i++) {
        ransac_buf[i].time = 0;
        
        ransac_buf[i].x = 0;
        ransac_buf[i].y = 0;
        ransac_buf[i].z = 0;

        ransac_buf[i].mx = 0;
        ransac_buf[i].my = 0;
        ransac_buf[i].mz = 0;
    }

    // Reset the corrections
    dr_ransac.corr_x  = 0; 
    dr_ransac.corr_y  = 0; 
    dr_ransac.corr_z  = 0;

    dr_ransac.corr_vx = 0; 
    dr_ransac.corr_vy = 0; 
    dr_ransac.corr_vz = 0;

}

/* suspect problems in this function ...*/
void ransac_propagate()
{
    // step1: Remove Old Vision updates from the buffer (if any)

    // Update buffer size
    dr_ransac.buf_size = 0;
    for (int i=0; i<RANSAC_BUF_SIZE; i++)
    {
        double mt = ransac_buf[get_index(i)].time; 

        // printf(mt); // print measurement arrival time 
        // max delta time (DT) for a measurement to be considered in the RANSAC buffer
        // ie, if any measurement is too old, don't increament buffer length for it.

        // ************x*x*||xx*****x**********x****||
        //              dt_max   buff_len   curr_time
        if ((mt == 0) || (dr_state.time - mt) > dr_ransac.dt_max) {
            break;
        }
        dr_ransac.buf_size++;
    }
    
    // currently we correct for every measurement made
    // If no vision for long time, its time to 
    // 1) check if there are enough measurements in the previous time steps 
    // 2) use ransac to extract the best predicition out of the buffers
    // 3) apply the corrections to the current drone state and start integrating from there
    // keep adding the corrections if the measurent and current time is close enough
    if (dr_ransac.buf_size > 0)
    {
        double age = (dr_state.time - ransac_buf[dr_ransac.buf_index_of_last].time);
        if (age > dr_ransac.dt_novision)
        {
          // printf("\n\n*** correction applied, ransac reset, buffers are cleared ***\n\n");
          // prediction_correct();
          // ransac_reset();
        }
    }
    
}


void ransac_push(double mt, double mx, double my, double mz)
{
    // Insert the new sample in the buffer
    dr_ransac.buf_index_of_last = dr_ransac.buf_index_of_last + 1;

    if (dr_ransac.buf_index_of_last >= RANSAC_BUF_SIZE)
    {
        dr_ransac.buf_index_of_last = 0;
    }
    ransac_buf[dr_ransac.buf_index_of_last].time = mt;
    
    ransac_buf[dr_ransac.buf_index_of_last].x = dr_state.x;
    ransac_buf[dr_ransac.buf_index_of_last].y = dr_state.y;
    ransac_buf[dr_ransac.buf_index_of_last].z = dr_state.z;

    ransac_buf[dr_ransac.buf_index_of_last].mx = mx;
    ransac_buf[dr_ransac.buf_index_of_last].my = my;
    ransac_buf[dr_ransac.buf_index_of_last].mz = mz;
} 

void ransac_correct() 
{
    // If sufficient items in buffer
    if (dr_ransac.buf_size > RANSAC_MIN_SAMPLES_FOR_FIT)
    {
        // Variables
        int n_samples = ((int)(dr_ransac.buf_size * 0.4));
        int n_iterations = 200;
        double error_threshold = 1.0;
        // int Dimension = 1;
        int count = dr_ransac.buf_size;

        double targets_x[RANSAC_BUF_SIZE];
        double targets_y[RANSAC_BUF_SIZE];
        double targets_z[RANSAC_BUF_SIZE];

        double samples[RANSAC_BUF_SIZE];

        double params_x[2];
        double params_y[2];
        double params_z[2];

        double fit_error_x = 0;
        double fit_error_y = 0;
        double fit_error_z = 0;

        // Fill LINEAR MODEL to fit
        for (int i=0; i<count; i++)
        {
            struct ransac_buffer_t* r = &ransac_buf[get_index(i)];
            
            targets_x[i] = r->x - r->mx;
            targets_y[i] = r->y - r->my;
            targets_z[i] = r->z - r->mz;

            samples[i] = r->time - dr_state.time;   
            //printf("Fit %f to %f\n", targets_x[i], samples[i][0]);
        }
        
        // printf("Running RANSAC with %d points and %d samples, err_max %f\n", count, n_samples, error_threshold);
        
        ransac_linear_model(n_samples, n_iterations, error_threshold, targets_x,
                                samples, count, params_x, &fit_error_x);

        ransac_linear_model(n_samples, n_iterations, error_threshold, targets_y,
                                samples, count, params_y, &fit_error_y);

        ransac_linear_model(n_samples, n_iterations, error_threshold, targets_z,
                                samples, count, params_z, &fit_error_z);

        // printf("fit err: %f,%f,%f,%f\n", dr_state.time, fit_error_x, fit_error_y, fit_error_z);
        
        // Export the RANSAC corrections
        dr_ransac.corr_x = -params_x[1];
        dr_ransac.corr_y = -params_y[1];
        dr_ransac.corr_z = -params_z[1];
        dr_ransac.corr_vx = -params_x[0];
        dr_ransac.corr_vy = -params_y[0];
        dr_ransac.corr_vz = -params_z[0];
        // printf("params: %f,%f,%f,%f,%f,%f,%f\n", dr_state.time, params_x[1], params_y[1], params_z[1], params_x[0], params_y[0],params_z[0]);
        if (abs(fit_error_x) < 0.0003 && abs(fit_error_y) < 0.0003 && abs(fit_error_z) < 0.0003 ) {
          double deltaT  = dr_state.time - ransac_buf[get_index(0)].time;

          dr_state.x = dr_state.x   + dr_ransac.corr_x + dr_ransac.corr_vx * deltaT;   // double filtered_x   
          dr_state.y = dr_state.y   + dr_ransac.corr_y + dr_ransac.corr_vy * deltaT;   // double filtered_y   
          dr_state.z = dr_state.z   + dr_ransac.corr_z + dr_ransac.corr_vz * deltaT;    // double filtered_z   
          dr_state.vx = dr_state.vx + dr_ransac.corr_vx;   // double filtered_vx  
          dr_state.vy = dr_state.vy + dr_ransac.corr_vy;   // double filtered_vy  
          dr_state.vz = dr_state.vz + dr_ransac.corr_vz;   // double filtered_vz 

          // fprintf(corr, "%f,%f,%f,%f\n", dr_state.time, dr_state.x, dr_state.y, dr_state.z)
        }
        ransac_reset();
        dr_ransac.ransac_cnt++;

    }
}


void ransac_linear_model(int n_samples, int n_iterations, double error_threshold, double *targets,
                         double *samples, unsigned int count, double *params, double *fit_error)
{

  double err;
  double errors[n_iterations];
  int indices_subset[n_samples];

  double subset_targets[n_samples];
  double subset_samples[n_samples];
  double subset_params[n_iterations];
  
  MatrixXf subset_params_mat(n_iterations, 2);

  bool use_bias = true;

  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < 2) ? 2 : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;

  // do the RANSAC iterations:
  for (int i = 0; i < n_iterations; i++) {

    // get a subset of indices - randomizer
    get_indices_without_replacement(indices_subset, n_samples, count);

    for (int j = 0; j < n_samples; j++) {
      subset_targets[j] = targets[indices_subset[j]];
      subset_samples[j] = samples[indices_subset[j]];
    }

    MatrixXf A(n_samples, 2);
    VectorXf b(n_samples); 
    for (int i = 0; i < n_samples; i++) {
          b(i)   = subset_targets[i];
          A(i,0) = subset_samples[i];
          A(i,1) = 1;
    }

    // cout << "Here is the matrix A:\n" << A << endl;
    // cout << "Here is the right hand side b:\n" << b << endl;

    VectorXf x;
    x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
    //cout << "least sqaures param x: \n" << x << endl;
    for (int d = 0; d < 2; d++) {
        subset_params[d] = x(d);
    }

    VectorXf residual(n_samples); 
    residual = A*x - b;

    *fit_error = 0;
    for (int sam = 0; sam < n_samples; sam++) {
      *fit_error += fabsf(residual(sam));
    }

    if (n_samples > 0) {
      *fit_error /= n_samples;
    }
    
    subset_params_mat(i,0) = subset_params[0];  // trend of correction
    subset_params_mat(i,1) = subset_params[1];  // bias of correction


    errors[i] = *fit_error;
  }

  // determine the minimal error:
  double min_err = errors[0];
  int min_ind = 0;
  for (int i = 1; i < n_iterations; i++) {
    if (errors[i] < min_err) {
      min_err = errors[i];
      min_ind = i;
    }
  }

  // copy the parameters:
  for (int d = 0; d < 2; d++) {
    params[d] = subset_params_mat(min_ind, d);
  }

  *fit_error = min_err;
  // printf("RANSAC done: %f, %f\n", params[0], params[1]);
}


void get_indices_without_replacement(int *indices_subset, int n_samples, int count)
{

  int index;

  for (int j = 0; j < n_samples; j++) {
    bool picked_number = false;
    while (!picked_number) {
      index = rand() % count;
      bool new_val = true;
      for (int k = 0; k < j; k++) {
        if (indices_subset[k] == index) {
          new_val = false;
          break;
        }
      }
      if (new_val) {
        indices_subset[j] = index;
        picked_number = true;
      }
    }
  }
}