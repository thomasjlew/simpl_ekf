/*
    File Name:      main.cpp
    ROS module:     simpl_ekf
    Description:    Implements an extended Kalman Filter for attitude estimation.
                    Only uses the accelerometer measurements with a simplified
                    model to compute pitch and roll of the device.
    Author:         Thomas Lew
    Last update:    2017.03.13
    Comments:       -Assumptions: - No movement (linear & angular) of the device
                                  - The model is therefore overly simplified.
                    -Known bug: The pitch value is always negative regardless of
                      (solved)  the rotation around y-axis. The cause is a wrong
                                z-value of vector "y" in update function.
                                This problem is solved in publishOdometryResult()
                                by using accel-x (ax) to determine the direction 
                                of the pitch angle.
                                                                                */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

//  Library for linear algebra
#include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include <math.h>

using namespace std;
using namespace Eigen;

//  Debugging boolean for printing EKF variables during execution
#define EKF_DEBUG 1

//  Number of IMU measurements used for calibration
#define IMU_COUNT_CALIBRATION 30

//  Euler angles values limits for debugging
#define PITCH_MAX M_PI/2
#define ROLL_MAX  M_PI/2
#define YAW_MAX   M_PI/2

/*  ------------------------------------------------------------
    Parameters of the Extended Kalman Filter. Depends on the IMU
    ------------------------------------------------------------                        */
/*//  Python Script simulated IMU measurements:
#define FREQ_HZ         50      // 50Hz for python Script
#define PERIOD_SEC      0.02    // 1/50 for python Script
#define SIGMA_SENSORS   0.1     // Variance of python script IMU data (imu_publisher)   */

//  Nvdia Shield Tablet IMU (only accelerometer) measurements:
#define FREQ_HZ       50        // 50Hz for Nvdia Shield Tablet
#define PERIOD_SEC    0.02      // 1/50 for Nvdia Shield Tablet
//  Computed with imu_variance.py : Variance_Accelerometer = 0.0004
#define SIGMA_SENSORS 0.02      // Variance of Nvdia Shield Tablet IMU                  */

/*  -----------------------------------------------------------------------------
    ROS Topics used for publishing the results and receiving the IMU measurements
    -----------------------------------------------------------------------------       */
ros::Publisher pub_odometry;
ros::Subscriber sub_imu;

const char imu_topic_name[] = "/imu_3dm_gx4/imu";
//const char imu_topic_name[] = "/python_IMU";


/*  ----------------------------------
    State vector

    Representation using Euler angles:

                [ Yaw ]
            x = [Pitch]
                [Roll ]

    ----------------------------------   */
VectorXd x(VectorXd::Zero(3));

//  Bias of sensors
const MatrixXd Q(PERIOD_SEC * PERIOD_SEC * MatrixXd::Identity(3,3));

//  Noise of sensors. We assume same value for accelerometer and gyroscope
const MatrixXd R(SIGMA_SENSORS * MatrixXd::Identity(3,3));

/*  --------------------
    Prediction variables
    --------------------    */

/*  Transition matrix A
    We assume a linear model with no motion, no rotation.   */
MatrixXd A(MatrixXd::Identity(3,3));

//  Covariance estimate matrix P
MatrixXd P(MatrixXd::Zero(3,3));



/*  --------------------
    Update variables
    --------------------    */

/*  Form of the measurements z:
    
        [accel-x]
    z = [accel-y]   with (-1 <= z <= 1)
        [accel-z]
    Note. These accelerometer measurements values are normalised.   */

/*  Measurement prediction vector H(x). It depends on the current
    Yaw, Pitch and Roll values of the state.                        */
VectorXd H(VectorXd::Zero(3));
/*  Jacobian matrix of H in regard of x                             */
MatrixXd J(MatrixXd::Zero(3,3));

/*  Accelerometer and gyroscope measurement for IMU calibration.
    These values are not used in the EKF but can be useful for
    further implementations.                                        */
double gx = 0;
double gy = 0;
double gz = 0;  //9.81

double bias_gyro_x = 0;
double bias_gyro_y = 0;
double bias_gyro_z = 0;

/*  IMU variables set at the start of the execution of the program.
    Used for calibration.                                           */
double current_t = 0;
bool is_time_init = false;
bool is_imu_init = false;
int imu_count = 0;

/*  Initialisation of the State initialisation.                     */
void state_init(){
    x.segment<3>(0) = Vector3d(0,0,0);
}

/*  Jacobian matrix Initialisation                                  */
void jacobian_init(){
    // block use: Selects the .block(1st value location, size matrix)
    /*J.block(0,3,3,3) = MatrixXd::Identity(3,3);
    J.block(0,6,3,3) = MatrixXd::Identity(3,3);*/
}

/* Covariance matrix Q initialisation                               */
void my_Q_covariance_matrix_init(){
    /*//  Values for the Nvidia shield tablet
    Q.block(0,0,3,3) = 0.032 * 0.032 * MatrixXd::Identity(3,3);
    Q.block(3,3,3,3) = 0.1 * 0.032 * MatrixXd::Identity(3,3);
    Q.block(6,6,3,3) = 0.03 * 0.032 * MatrixXd::Identity(3,3);  //  */
    /*//  Values for the test python publisher
    //  0.02 ) 1/50 with 50hz
    /*double delta_t = 1 / double(FREQ_HZ);
    Q = delta_t * delta_t * MatrixXd::Identity(3,3);*/
    /*Q.block(3,3,3,3) = 0.1 * 0.02 * MatrixXd::Identity(3,3);
    Q.block(6,6,3,3) = 0.03 * 0.02 * MatrixXd::Identity(3,3);   //  */
}

/*  Computes the original orientation and gyroscope bias.
    These values are not used in the EKF but can be useful for
    further implementations.                                        */
void imu_init(double ax, double ay, double az, 
              double rx, double ry, double rz){
    gx += ax;
    gy += ay;
    gz += az;

    bias_gyro_x += rx;
    bias_gyro_y += ry;
    bias_gyro_z += rz;

    if(imu_count == IMU_COUNT_CALIBRATION){
        gx /= imu_count;
        gy /= imu_count;
        gz /= imu_count;

        bias_gyro_x /= imu_count;
        bias_gyro_y /= imu_count;
        bias_gyro_z /= imu_count;

        cout << "calibrated gx: " << gx << endl;
        cout << "calibrated gy: " << gy << endl;
        cout << "calibrated gz: " << gz << endl;

        cout << "bias_gyro_x: " << bias_gyro_x << endl;
        cout << "bias_gyro_y: " << bias_gyro_y << endl;
        cout << "bias_gyro_z: " << bias_gyro_z << endl;
    }
}


Vector3d get_euler_angles()
{
    return x.segment<3>(0);
}

void publishOdometryResult(double time_imu, double ax)
{
    //  Get current state variables
    Vector3d angles = get_euler_angles();
    double yaw   = angles(0);
    double pitch = angles(1);
    double roll  = angles(2);

    //  Odometry results are published
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = 0;
    odometry.pose.pose.position.y = 0;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.x = 0;
    odometry.pose.pose.orientation.y = 0;
    odometry.pose.pose.orientation.z = 0;
    odometry.pose.pose.orientation.w = 1;
    odometry.twist.twist.linear.x = 0;
    odometry.twist.twist.linear.y = 0;
    odometry.twist.twist.linear.y = 0;
    odometry.twist.twist.angular.z = angles(0);
    /*  Fix for the pitch angular being always negative */
    if(ax >= 0)
        odometry.twist.twist.angular.y = abs(angles(1));
    else
        odometry.twist.twist.angular.y = -abs(angles(1));
    odometry.twist.twist.angular.x = angles(2);

    //  Publish EKF Results
    pub_odometry.publish(odometry);

    /*  For Debugging purposes: 
        The pitch, roll and yaw values are displayed on the terminal                */
    if(EKF_DEBUG){
        cout << "--------------------------------------------" << endl;
        cout << "End of kalman filtering for this imu dataset" << endl;
        cout << "--------------------------------------------" << endl;
        ROS_INFO("Odometry results published after IMU sample time: %lf", time_imu);
        cout << "Yaw: " <<      yaw     << endl;
        cout << "Pitch: " <<    pitch   << endl;
        cout << "Roll: " <<     roll    << endl;
        cout << "                     " << endl;

        //  We limit the pitch and roll values to stop a non converging Kalman Filter
        if(pitch > PITCH_MAX || pitch < -PITCH_MAX || roll > ROLL_MAX || roll < -ROLL_MAX){
            cout << "pitch or roll value too high " << endl;
            abort();
        }
    }
}

/*  Extended Kalman Filter prediction function. */
void predict(double dt){
    if(EKF_DEBUG){
        cout << "                       " << endl;
        cout << " --------------------- " << endl;
        cout << "  NEW STATE PREDICTION " << endl;
        cout << " --------------------- " << endl;
        cout << "                       " << endl;
    }

    /*  -----------------------------
        Predict the prediction matrix
        -----------------------------
            Prediction matrix A
                    {1 0 0}
                A = {0 1 0}
                    {0 0 1}         */

    MatrixXd A(MatrixXd::Identity(3,3));

    x = A * x;

    /*  The Yaw cannot be computed with this model.
        To avoid a drift of this angle, it is set to zero.              */
    x[0] = 0;

    //  Covariance matrix prediction
    P = A * P * A.transpose() + Q;

    Vector3d angles = get_euler_angles();
    double yaw   = angles(0);
    double pitch = angles(1);
    double roll  = angles(2);

    if(EKF_DEBUG){
        cout << "predicted pitch   yaw   roll "         << endl;
        cout << pitch << ";  "  << yaw << ";  " << roll << endl;
        cout << "                 "                     << endl;

        cout << "Value of P after prediction"   << endl;
        cout << P                               << endl;
        cout << "                 "             << endl;

        cout << "Value of Q : "                 << endl;
        cout << Q                               << endl;
        cout << "                 "             << endl;
    }
}

/*  For Debugging purposes: 
    the state vector, measurements and EKF matrices
    are displayed on the terminal.                              */
void debug_ekf_cout(double pitch, double yaw, double roll,
                    VectorXd z, Vector3d H, Matrix<double, 3, 3> J,
                    VectorXd y, Matrix<double, 3, 3> S, 
                    Matrix<double, 3, 3> K, MatrixXd P){
    cout << "Old value of Pitch, yaw and Roll " << endl;
    cout << pitch << "  " << yaw << "  " << roll << endl;
    cout << "                 " << endl;
    cout << "Value of z (ax, ay, az) :" << endl;
    cout << z << endl;
    cout << "                 " << endl;
    cout << "H vector updated: " << endl;
    cout << H             << endl;
    cout << "           " << endl;
    cout << "J matix updated: " << endl;
    cout << J << endl;
    cout << "                 " << endl;
    cout << "Value of y " << endl;
    cout << y << endl;
    cout << "                 " << endl;
    /*cout << "S  updated: "<< endl;
    cout << S << endl;
    cout << "                 " << endl;
    cout << "S inverse updated: "<< endl;
    cout << S.inverse() << endl;
    cout << "                 " << endl;*/
    cout << "K updated: " << endl;
    cout << K << endl;
    cout << "                 " << endl;
    cout << "K * y updated: " << endl;
    cout << K * y << endl;
    cout << "                 " << endl;
    /*cout << "P updated: "<< endl;
    cout << P << endl;
    cout << "                 " << endl;*/
}

/*  Extended Kalman Filter update function. */
void update(double ax, double ay, double az, double rx, double ry, double rz, double dt)
{
    if(EKF_DEBUG){
        cout << "                 " << endl;
        cout << "---------------- " << endl;
        cout << " NEW UPDATE n. "   << imu_count << endl;
        cout << "---------------- " << endl;
        cout << "                 " << endl;
    }

    Vector3d angles = get_euler_angles();
    double yaw   = angles(0);
    double pitch = angles(1);
    double roll  = angles(2);

    //  Measurement Vector
    VectorXd z(3);
    z << ax, ay, az;

    /*  ---------------------------------------------------
        -                Measurement Model                -
        ---------------------------------------------------   */

    /*          --------------------------
                Measurement vector z model
                --------------------------
                               T  (0)            -1       T
                z = H(x) = Rzyx * (0),  with Rzyx   = Rzyx
                                  (1)   and Rzyx = Rz*Ry*Rx = R = ...

    [cos(yaw) -sin(yaw) 0][cos(pitch) 0 sin(pitch)][1     0          0    ]
    [sin(yaw) cos(yaw)  0][    0      1      0    ][0 cos(roll) -sin(roll)]
    [   0         0     1][-sin(pitch)0 cos(pitch)][0 sin(roll)  cos(roll)]

            --------------------------------------
            Jacobian matrix from measurement model
            -------------------------------------- 
                    J = d/dx (H(x))  
                                                                        */

    //  Theoretical Measurement from state estimate:
    Vector3d H(3);
    H <<    -sin(pitch),
            cos(pitch)*sin(roll),
            cos(pitch)*cos(roll);

            /*cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll),
            sin(yaw)*sin(pitch)*sin(roll) - cos(yaw)*sin(roll),
            cos(pitch)*cos(roll);*/

    //  Jacobian matrix J computation
    Matrix<double, 3, 3> J;
    J <<    0,  -cos(pitch),            0,
            0,  -sin(pitch)*sin(roll),  cos(pitch)*cos(roll),
            0,  -sin(pitch)*cos(roll),  -cos(pitch)*sin(roll);

    /*   
    -sin(yaw)*sin(pitch)*cos(roll)+cos(yaw)*sin(roll), cos(yaw)*cos(pitch)*cos(roll), -cos(yaw)*sin(pitch)*sin(roll)+sin(yaw)*cos(roll),
    cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll),  sin(yaw)*cos(pitch)*cos(roll), -sin(yaw)*sin(pitch)*sin(roll)-cos(yaw)*cos(roll),
    0,                                                 -sin(pitch)*cos(roll),         -cos(pitch)*sin(roll);*/

    /*  -------------------------------------
        Update step of Extended Kalman Filter
        -------------------------------------  */

    //  Innovation vector
    VectorXd y(3);
    y = z - H;

    //  Innovation covariance
    MatrixXd S(3,3);
    S = J * P * J.transpose() + R;

    //  Kalman gain
    MatrixXd K(3,3);
    K = P * J * S.inverse();

    //  State estimate update
    x = x + K * y;

    //  Covariance estimate update
    P = (MatrixXd::Identity(3, 3) - K * J) * P;

    //  If Debug Mode: Print all variables computed by this update
    if(EKF_DEBUG){
        debug_ekf_cout(pitch, yaw, roll, z, H, J, y, S, K, P);
    }
}

/*  Extended Kalman Filter global function.             */
void kalman_filtering(double ax, double ay, double az, 
                      double rx, double ry, double rz, 
                      double dt){
    predict(dt);
    update(ax, ay, az, rx, ry, rz, dt);
}


/*  IMU callback function.
    Uses the IMU measurements to compute the new state variables using an EKF. */
void imu_callback(const sensor_msgs::Imu& imu_msg)
{
    ROS_INFO("Time of IMU sample: %lf", imu_msg.header.stamp.toSec());

    /*  Accelerometer and Gyroscope measurements.
        This version doesn't use the angular velocity values */
    double ax = imu_msg.linear_acceleration.x;
    double ay = imu_msg.linear_acceleration.y;
    double az = imu_msg.linear_acceleration.z;
    double rx = imu_msg.angular_velocity.x;
    double ry = imu_msg.angular_velocity.y;
    double rz = imu_msg.angular_velocity.z;

    /*  ----------------------------------------------------------------------
        Calibrate Inertial Measurement Unit (IMU) at the start of the program.
        Can be useful in case of an initial spatial frame which is moving.
        Uses the IMU_COUNT_CALIBRATION first IMU values.
        ----------------------------------------------------------------------  */
    imu_count++;
    if(!is_imu_init){
        imu_init(ax,ay,az,rx,ry,rz);
        if(imu_count == IMU_COUNT_CALIBRATION){
            is_imu_init = true;
            imu_count = 0;
        }
        return;
    }

    //  After imu calibration, set the IMU time
    if(!is_time_init){
        current_t = imu_msg.header.stamp.toSec();
        is_time_init = true;
        return;
    }

    /*  --------------------------------
        Accelerometer data normalisation
        --------------------------------  */
    double norm_accel = sqrt(ax * ax + ay * ay + az * az);
    ax = ax / norm_accel;
    ay = ay / norm_accel;
    az = az / norm_accel;

    /*  ---------------
        IMU Time update
        ---------------  */
    double new_time = imu_msg.header.stamp.toSec();
    double dt = new_time - current_t;
    current_t = new_time;


    /*  --------------------------------------------------------------
        Extended Kalman Filtering.

        In this version, only the accelerometer measurements are used.
        --------------------------------------------------------------  */
    kalman_filtering(ax, ay, az, rx, ry, rz, dt);


    /*  -------------------------------------------------
        Results of the EKF are pusblished in a ROS topic.
        -------------------------------------------------  */
    publishOdometryResult(imu_msg.header.stamp.toSec(), ax);
}

void setupROS()
{
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    //  Define ROS topics to publish the results
    pub_odometry     = n.advertise<nav_msgs::Odometry>("odometry", 1000);

    //  Define IMU Measurements ROS topic.
    sub_imu = n.subscribe(imu_topic_name, 1000, imu_callback);
}

int main(int argc, char **argv)
{
    state_init();
    jacobian_init();
    my_Q_covariance_matrix_init();

    ros::init(argc, argv, "simpl_ekf");

    setupROS();
    ros::spin();

    return 0;
}