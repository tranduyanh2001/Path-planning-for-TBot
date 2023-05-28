#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Float64.h"
#include "common.hpp"
#include <fstream>

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

double get_pos_err(double xt,double yt,double xp,double yp) {
    double Dx = xt - xp;
    double Dy = yt - yp;
            
    return sqrt(Dx*Dx + Dy*Dy);
}

double get_ang_err(double xt,double yt,double xp,double yp, double heading) {
    double angular_error = atan2(yt-yp, xt-xp) - heading;
    
    angular_error = limit_angle(angular_error);
            
    return angular_error;
}

double motion_couple(double ang_err) {
    double x0 = 0;
    double x1 = 0.2;
    double y0 = 1;
    double y1 = 0;
    double x = abs(ang_err);
    double output = 0;
    // linear interpolate 0 to pi with 0 to 1
    // when ang_err = 0, output 1
    // when ang_err = some distance away, output 0

    output =((y1 - y0) / (x1 - x0) * (x - x0)) + y0;
    if (x > x1) {
        output = 0.0;
    }

    return abs(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;
    // for recording data
    std::ofstream data_file;
    // data_file.open("/home/steve/a237r/data.txt");

    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    ros::Publisher pub_lin_error = nh.advertise<std_msgs::Float64>("move_lin_error", 1, true);
    ros::Publisher pub_rbt_pos = nh.advertise<geometry_msgs::Pose>("pos_rbt", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.
    geometry_msgs::Pose rbt_pos;
    std_msgs::Float64 errorFloat;

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    double P_lin ,I_lin, D_lin,
    P_angle, I_angle, D_angle,
    forward_signal, angular_signal,
    prev_angular_signal,
    lin_acc, ang_acc,
    accum_error_lin, accum_error_ang,
    error_lin, ang_error,
    prev_error_lin, prev_ang_error = 0;

    double tenth_percentile = 0;
    double ninety_percentile = 0;
    double lin_max_overshoot = pos_rbt.x;
    double ang_max_overshoot = 0;

    bool captured10th = false;
    bool captured90th = false;
    bool experiment = false;
    bool linear_experiment = false;

    error_lin = get_pos_err(target.x, target.y, pos_rbt.x, pos_rbt.y);
    prev_error_lin = error_lin;

    ang_error = get_ang_err(target.x, target.y, pos_rbt.x, pos_rbt.y, ang_rbt);
    prev_ang_error = ang_error;

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            // Time step control ---------------------------------------
            // note: monitor dt for its consistency in completing one cycle of calculation
            // Question: is it better to set a constant time step than relying on dt ?
            // e.g. if (TIME_INC >= dt) 

            ////////////////// MOTION CONTROLLER HERE //////////////////
            
            //Position error -------------------------------------------
            if (experiment) {
                error_lin = target.x - pos_rbt.x;
            }
            else {
                error_lin = get_pos_err(target.x, target.y, pos_rbt.x, pos_rbt.y);
            }

            //angular error --------------------------------------------
            //ang_error = get_ang_err(target.x, target.y, pos_rbt.x, pos_rbt.y, ang_rbt);

            //omnidirection control
            ang_error = limit_angle(heading(pos_rbt,target) - ang_rbt);

            //if (abs(ang_error))

            // // //Coupling Angular Error with linear velocity

            // if (abs(ang_error)<= M_PI_2){
            //     if(abs(ang_error)<(M_PI/3)){ //(M_PI/3)?
            //         forward_signal = ( (forward_signal/2) * cos(3*ang_error) ) + (forward_signal/2);
            //     }
            //     else{
            //         forward_signal = 0;
            //     }
            // }
            // else{
            //     if(abs(ang_error)>((2*M_PI)/3)){
            //         forward_signal = -1*( ((forward_signal/2)*cos( (3*ang_error) + M_PI)) + (forward_signal/2) );
            //     }
            //     else{
            //         forward_signal = 0;
            //     }
            // }
            //error_lin*=pow(cos(ang_error),4);

            if (ang_error >= M_PI_2 || ang_error <= -M_PI_2) {
                ang_error = limit_angle(ang_error + M_PI);
                error_lin = -1 * error_lin; //forward_signal
            }
            
            //P control
            P_lin = Kp_lin * error_lin;
            //I control
            accum_error_lin += error_lin;
            I_lin = Ki_lin * accum_error_lin;
            //D control
            D_lin = Kd_lin * (error_lin - prev_error_lin) / dt;
            prev_error_lin = error_lin;
            forward_signal = P_lin + I_lin + D_lin;
            
            forward_signal = forward_signal * pow(cos(ang_error),4);

            lin_acc = (forward_signal - cmd_lin_vel) / dt;

            if (experiment) {
                cmd_lin_vel = cmd_lin_vel + lin_acc * dt;
            }
            else {
                lin_acc = sat(lin_acc, max_lin_acc); //saturated forward signal
                cmd_lin_vel = sat(cmd_lin_vel + (lin_acc * dt), max_lin_vel);
            
            // Angular error and linear velocity coupling -------------
            //cmd_lin_vel = cmd_lin_vel * motion_couple(ang_error);
            //in progress..
            }

            
            //P control
            P_angle = Kp_ang * ang_error;
            //I control
            accum_error_ang += ang_error;
            I_angle = Ki_ang * accum_error_ang;
            //D control
            D_angle = Kd_ang * (ang_error - prev_ang_error) / dt;
            prev_ang_error = ang_error;

            //PID TOTAL OF ANGULAR SIGNAL
            angular_signal = P_angle + I_angle + D_angle;

            //forward_signal = (P_lin + I_lin + D_lin)*pow(cos(ang_error),4);

            ang_acc = (angular_signal - cmd_ang_vel) / dt;

            if (experiment) {
                cmd_ang_vel = cmd_ang_vel + (ang_acc * dt);
            }
            else {
                ang_acc = sat(ang_acc, max_ang_acc); //saturated angular signal
                cmd_ang_vel = sat(cmd_ang_vel + (ang_acc * dt), max_ang_vel);
            }


            

            // publish speeds
            if (linear_experiment == true && experiment == true) {
                msg_cmd.linear.x = cmd_lin_vel;
                msg_cmd.angular.z = 0;

                errorFloat.data = pos_rbt.x;
                pub_lin_error.publish(errorFloat);
            
                rbt_pos.position.x = pos_rbt.x;
                rbt_pos.position.y = pos_rbt.y;
                pub_rbt_pos.publish(rbt_pos);

                // calculate rise time
                if (pos_rbt.x > -2.99 && pos_rbt.x <-2.96 &&  !captured10th){
                    tenth_percentile = ros::Time::now().toSec();
                    captured10th = true;
                }

                if (pos_rbt.x > -2.84 && pos_rbt.x <-2.82 && !captured90th){
                    ninety_percentile = ros::Time::now().toSec();
                    captured90th = true;
                }
                
                //max_overshoot
                if (pos_rbt.x > target.x){
                    if (pos_rbt.x - target.x > lin_max_overshoot){
                        lin_max_overshoot = pos_rbt.x - target.x;
                    }
                }
                
            }
            else if (linear_experiment != true && experiment == true) {
                msg_cmd.linear.x = 0;
                msg_cmd.angular.z = cmd_ang_vel;

                rbt_pos.orientation.w = ang_rbt;
                pub_rbt_pos.publish(rbt_pos);

                // calculate rise time
                if ((ang_rbt > 0.3 && ang_rbt < 0.4) && !captured10th){
                    tenth_percentile = ros::Time::now().toSec();
                    captured10th = true;
                }

                if ((ang_rbt > 2.7 && ang_rbt < 2.9) && !captured90th){
                    ninety_percentile = ros::Time::now().toSec();
                    captured90th = true;
                }

                //max_overshoot
                if (ang_error < 0){  //atan2(target.x - pos_rbt.x, target.y - pos_rbt.y);
                    if (ang_error < ang_max_overshoot){ //heading of target
                        ang_max_overshoot = ang_error;
                    }
                }

            }
            else {
                msg_cmd.linear.x = cmd_lin_vel;
                msg_cmd.angular.z = cmd_ang_vel;

                rbt_pos.position.x = pos_rbt.x;
                rbt_pos.position.y = pos_rbt.y;
                rbt_pos.orientation.w = ang_rbt;
                pub_rbt_pos.publish(rbt_pos);
            }
            
            pub_cmd.publish(msg_cmd);
            
            // verbose
            if (verbose)
            {
                if (linear_experiment) {
                    ROS_INFO("\t%6.3f\t%6.3f\t%7.3f\t%9.5f)\n", Kp_lin, Kd_lin, ninety_percentile-tenth_percentile, lin_max_overshoot);
                }
                else {
                    ROS_INFO("\t%6.3f\t%6.3f\t%10.5f\t%9.5f)\n", Kp_ang, Kd_ang, ninety_percentile-tenth_percentile,ang_max_overshoot);
                }
                // ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f) \n", cmd_lin_vel, cmd_ang_vel);
                // ROS_INFO(" TIME :  TIME_INC (%6.3f) \n", dt);
                // ROS_INFO(" ang_rbt: %7.3f", ang_rbt);
                // ROS_INFO(" TIME :  TIME_INC (%6.3f) \n", dt);
                // ROS_INFO(" COUPLING: %7.3f \n", motion_couple(ang_error));
                // ROS_INFO(" ang_err: %7.3f\n", ang_error);
                // ROS_INFO("\t%f\t%f\t%f\t%f\t%f",
                // ros::Time::now().toSec(), error_lin, P_lin, accum_error_lin,
                // I_lin);
            }

            // write to file
            // if (linear_experiment) {
            //     data_file << ros::Time::now().toSec() << "\t" << error_lin
            //     << "\t" << P_lin << "\t" << accum_error_lin << "\t" <<
            //     I_lin << "\t" << D_lin << "\t" << cmd_lin_vel << "\t" << pos_rbt.x << "\t" << ninety_percentile << "\t" << tenth_percentile << "\t" << lin_max_overshoot << std::endl;
            // }
            // else {
            //     data_file << ros::Time::now().toSec() << "\t" << ang_error
            //     << "\t" << P_angle << "\t" << accum_error_ang << "\t" <<
            //     I_angle << "\t" << D_angle << "\t" << cmd_ang_vel << "\t" << ang_rbt << "\t" << ninety_percentile << "\t" << tenth_percentile << "\t" << ang_max_overshoot << std::endl;
            // }
            
            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");

    // close file
    data_file.close();

    return 0;

}
