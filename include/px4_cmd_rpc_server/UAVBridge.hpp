// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef VEHICEL_HPP
#define VEHICEL_HPP
#include <ros/ros.h>
#include <string>
#include <cmath>
#include <thread>
#include <mutex>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <px4_cmd/Command.h>
#include <px4_cmd/custom_command.h>

#define PI 3.14159265358979323846

using namespace std;

namespace UAV
{
    class UAVBridge
    {
        private:
            // setting
            /// @brief ROS NodeHandle
            ros::NodeHandle nh;
            /// @brief Position Subscriber
            ros::Subscriber current_pos_sub;
            /// @brief Velocity Subscriber
            ros::Subscriber current_vel_sub;
            /// @brief State Subscriber
            ros::Subscriber current_state_sub;
            /// @brief Extend State Subscriber
            ros::Subscriber current_extend_state_sub;
            /// @brief Mavros Setpoint Publisher
            ros::Publisher setpoint_raw_pub;
            /// @brief Flying Mode Service Client
            ros::ServiceClient mode_client;
            /// @brief Arming Service Client
            ros::ServiceClient arming_client;
            /// @brief Get Model Real Properties in Gazebo Server
            ros::ServiceClient gazebo_model_state_client;
            /// @brief Quaternion for vehicle
            tf::Quaternion quat;
            /// @brief Vehicle State
            mavros_msgs::State current_state;
            /// @brief Local Position Setpoint
            mavros_msgs::PositionTarget pos_setpoint;
            /// @brief Global Position Setpoint
            mavros_msgs::GlobalPositionTarget pos_setpoint_global;
            /// @brief Attitude Setpoint
            mavros_msgs::AttitudeTarget attitude_setpoint;
            /// @brief Gazebo Model State Message
            gazebo_msgs::GetModelState model_state;
            /// @brief Flying Mode Command
            mavros_msgs::SetMode mode_cmd;
            /// @brief Arm Command
            mavros_msgs::CommandBool arm_cmd;
            /// @brief State of External Command Mode
            std_msgs::Bool ext_cmd_require_msg;
            /// @brief Mutex for multi-thread
            std::mutex cmd_mutex;
            /// @brief Mutex for Position Message Callback
            std::mutex position_mutex;
            /// @brief Mutex for Velocity Message Callback
            std::mutex velocity_mutex;
            /// @brief Mutex for Publisher
            std::mutex pub_mutex;
            /// @brief Mutex for GetModelState Service
            std::mutex gzmodel_mutex;
            /// @brief Initial Roll Angle (rad)
            double init_roll;
            /// @brief Initial Pitch Angle (rad)
            double init_pitch;
            /// @brief Initial Yaw Angle (rad)
            double init_yaw;
            /// @brief Initial x Axis Position (m) in Global Frame
            double init_x;
            /// @brief Initial y Axis Position (m) in Global Frame
            double init_y;
            /// @brief Initial z Axis Position (m) in Global Frame
            double init_z;
            /// @brief External Command Run Time 
            int desire_time = 0;
            /// @brief Hover Mode Flag
            bool hover = false;
            /// @brief Current Custom External Command Mode (More Detail in px4_cmd/custom_command.h)
            int current_custom_mode = CommandMode::TargetLocal;
            /// @brief Vehicle Mavros Topic Name Header
            std::string topic_header;

            /// @brief vehicle pose & position subscribe call back function
            /// @param msg message
            void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

            /// @brief vehicle velocity subscribe call back function
            /// @param msg message
            void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

            /// @brief vehicle state subscribe call back function
            /// @param msg message 
            void state_cb(const mavros_msgs::State::ConstPtr &msg);

            /// @brief vehicle extended state subscribe call back function
            /// @param msg message
            void extend_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg);

            /// @brief ROS node thread function 
            void ros_thread_fun();

            /// @brief Get Model State thread function
            void get_model_state_thread_func();

        public:
            /// @brief Information / Command Update Time
            double update_time = 0.02;
            /// @brief Vehicle PX4 Mode: mavros_msgs::State
            std::string state_mode;
            /// @brief Vehicle Mavros Node Name
            std::string node_name;
            /// @brief Vehicle Name
            std::string vehicle_name;
            /// @brief Vehicle Type
            int vehicle_type = px4_cmd::Command::Multicopter;
            /// @brief Set Vehicle Command
            px4_cmd::Command vehicle_command;
            /// @brief Vehicle Arm State
            bool arm_state = false;
            /// @brief Vehicle Land State
            bool land_state = true;
            /// @brief Vehicle State
            std::string state = "UnDefined";
            /// @brief Flag for Stop Main Thread
            bool thread_stop = false;
            /// @brief Flag for ROS Running State
            bool ros_stop = false;
            /// @brief Flag for achieving desire position command
            bool achieve_desire = false;
            /// @brief Param for judging if achieve desire position
            double sigma = 0.1;
            /// @brief x Axis Estimation Position (m) in Global Frame
            double x;
            /// @brief y Axis Estimation Position (m) in Global Frame
            double y;
            /// @brief z Axis Estimation Position (m) in Global Frame
            double z;
            /// @brief x Axis Estimation Velocity (m/s) in Global Frame
            double vx;
            /// @brief y Axis Estimation Velocity (m/s) in Global Frame
            double vy;
            /// @brief z Axis Estimation Velocity (m/s) in Global Frame
            double vz;
            /// @brief Estimation Quaternion [x, y, z, w]
            double quaternion[4] = {0, 0, 0, 0};
            /// @brief Estimation Pitch Angle (rad)
            double pitch;
            /// @brief Estimation Roll Angle (rad)
            double roll;
            /// @brief Estimation Yaw Angle (rad)
            double yaw;
            // real information
            /// @brief x Axis Real Position (m) in Global Frame
            double x_real;
            /// @brief y Axis Real Position (m) in Global Frame
            double y_real;
            /// @brief z Axis Real Position (m) in Global Frame
            double z_real;
            /// @brief x Axis Real Velocity (m/s) in Global Frame
            double vx_real;
            /// @brief y Axis Real Velocity (m/s) in Global Frame
            double vy_real;
            /// @brief z Axis Real Velocity (m/s) in Global Frame
            double vz_real;
            /// @brief Real Quaternion [x, y, z, w]
            double quaternion_real[4] = {0, 0, 0, 0};
            /// @brief Real Pitch Angle (rad)
            double pitch_real;
            /// @brief Real Roll Angle (rad)
            double roll_real;
            /// @brief Real Yaw Angle (rad)
            double yaw_real;

            /// @brief Home Position
            vector<double> home_position = {0, 0};
            /// @brief Hover Postion
            vector<double> hover_pos = {0, 0, 0};

            /// @brief New UAVBridge instance
            /// @param node node name for vehicle
            UAVBridge(string node);

            /// @brief start vehicle control
            void start();

            /// @brief set desire mode for vehicle and return execute state
            /// @param desire_mode desire mode for vehicle
            /// @return error string, if success, return empty string
            string set_mode(string desire_mode);

            /// @brief set desire command for vehicle and return execute state
            /// @param msg px4_cmd::Command Message Pointer
            void set_command(const px4_cmd::Command *msg);
    };

    UAVBridge::UAVBridge(string node)
    {
        node_name = node;
    };

    void UAVBridge::start()
    {
        pos_setpoint.header.frame_id = 1;
        pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        int argc = 0;
        char **argv;
        topic_header = "/" + node_name + "/mavros/";
        ros::init(argc, argv, "px4_cmd/" + node_name + "_cmd");
        ros::param::get(("/" + node_name + "/vehicle").c_str(), vehicle_name);
        ros::param::get(("/" + node_name + "/init_x").c_str(), init_x);
        ros::param::get(("/" + node_name + "/init_y").c_str(), init_y);
        ros::param::get(("/" + node_name + "/init_z").c_str(), init_z);
        ros::param::get(("/" + node_name + "/init_R").c_str(), init_roll);
        ros::param::get(("/" + node_name + "/init_P").c_str(), init_pitch);
        ros::param::get(("/" + node_name + "/init_Y").c_str(), init_yaw);
        if (vehicle_name == "plane")
        {
            vehicle_type = px4_cmd::Command::FixWing;
        }

        while (!ros::ok())
        {
            usleep(floor(1000000 * update_time));
        }
        current_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>((topic_header + "local_position/pose").c_str(), 20, &UAVBridge::pos_cb, this);
        current_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>((topic_header + "local_position/velocity_local").c_str(), 20, &UAVBridge::vel_cb, this);
        current_state_sub = nh.subscribe<mavros_msgs::State>((topic_header + "state").c_str(), 5, &UAVBridge::state_cb, this);
        current_extend_state_sub = nh.subscribe<mavros_msgs::ExtendedState>((topic_header + "extended_state").c_str(), 5, &UAVBridge::extend_state_cb, this);
        setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>((topic_header + "setpoint_raw/local").c_str(), 50);
        gazebo_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        model_state.request.model_name = node_name;
        mode_client = nh.serviceClient<mavros_msgs::SetMode>((topic_header + "set_mode").c_str());
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>((topic_header + "cmd/arming").c_str());
        std::thread ros_thread(&UAVBridge::ros_thread_fun, this);
        ros_thread.detach();
        std::thread get_model_state_thread(&UAVBridge::get_model_state_thread_func, this);
        get_model_state_thread.detach();
    }

    string UAVBridge::set_mode(string desire_mode)
    {
        int error_times = 0;
        string err_msg = "";
        // 解锁
        if (desire_mode == "Arm" || desire_mode == "DisArm")
        {
            // 如果飞行高度超过20cm则不允许DisArm
            if (!land_state && desire_mode == "DisArm")
            {
                return "Vehicle is Flying, you can not DisArm!";
            }

            bool desire_arm_cmd = (desire_mode == "Arm") ? true : false;
            while (error_times < 10)
            {
                ros::spinOnce();
                usleep(200000);
                if (current_state.armed == desire_arm_cmd)
                {
                    return "";
                }
                arm_cmd.request.value = desire_arm_cmd;
                if (arming_client.call(arm_cmd))
                {
                    // 执行回调函数
                    ros::spinOnce();
                    usleep(200000);
                    if (current_state.armed == desire_arm_cmd)
                    {
                        return "";
                    }
                    else
                    {
                        err_msg = desire_mode + " Command Sent but " + desire_mode + " Failed!";
                    }
                }
                else
                {
                    err_msg = desire_mode + " Command Sent Failed!";
                }
                error_times++;
            }
            if (error_times == 0)
            {
                err_msg = "Already " + desire_mode + "!";
            }
            return err_msg;
        }
        // 更改模式
        while (current_state.mode != desire_mode && error_times < 10)
        {
            err_msg = "";
            // 处于OFFBOARD模式时，只能改为AUTO模式
            if (current_state.mode == "OFFBOARD")
            {
                if (desire_mode != "AUTO.LAND" && desire_mode != "AUTO.RTL")
                {
                    return "In OFFBOARD Mode, you can only change to [Auto.Land] or [Auto.RTL] Mode!";
                }
            }

            // 请求更改模式服务
            mode_cmd.request.custom_mode = desire_mode;
            mode_client.call(mode_cmd);
            ros::spinOnce();
            usleep(200000);
            if (mode_cmd.response.mode_sent)
            {
                if (current_state.mode == desire_mode)
                {
                    return "";
                }
                else
                {
                    err_msg = desire_mode + " Mode Sent but Changed Failed!";
                }
            }
            else
            {
                err_msg = "Mode Sent Failed!";
            }
            error_times++;
        }
        if (current_state.mode == desire_mode)
        {
            return "Current Mode is already [" + desire_mode + "]";
        }
        return err_msg;
    }

    void UAVBridge::ros_thread_fun()
    {
        sleep(1);
        ros::spinOnce();
        bool sub_state = true;
        while (ros::ok() && !thread_stop && sub_state)
        {
            if (current_pos_sub.getNumPublishers() > 0)
            {
                sub_state = true;
            }
            else
            {
                sub_state = false;
            }
            setpoint_raw_pub.publish(pos_setpoint);
            usleep(floor(1000000 * update_time));
            ros::spinOnce();
        }
        ros::shutdown();
        ros_stop = true;
    }

    void UAVBridge::get_model_state_thread_func()
    {
        while (ros::ok() && !thread_stop)
        {
            gazebo_model_state_client.call(model_state);
            if (!model_state.response.success)
            {
                continue;
            }
            gzmodel_mutex.lock();
            quaternion_real[0] = model_state.response.pose.orientation.x;
            quaternion_real[1] = model_state.response.pose.orientation.y;
            quaternion_real[2] = model_state.response.pose.orientation.z;
            quaternion_real[3] = model_state.response.pose.orientation.w;
            tf::Quaternion q(model_state.response.pose.orientation.x, model_state.response.pose.orientation.y, model_state.response.pose.orientation.z, model_state.response.pose.orientation.w);
            tf::Matrix3x3(q).getRPY(roll_real, pitch_real, yaw_real);
            x_real = model_state.response.pose.position.x;
            y_real = model_state.response.pose.position.y;
            z_real = model_state.response.pose.position.z;
            vx_real = model_state.response.twist.linear.x;
            vy_real = model_state.response.twist.linear.y;
            vz_real = model_state.response.twist.linear.z;
            gzmodel_mutex.unlock();
            usleep(floor(1000000 * update_time));
            ros::spinOnce();
        }
    }

    void UAVBridge::set_command(const px4_cmd::Command *msg)
    {
        vehicle_command = *msg;
        std::string error;
        CustomCommand cmd;
        cmd_mutex.lock();

        // custom command
        while (state_mode == mavros_msgs::State::MODE_PX4_OFFBOARD && msg->Mode == px4_cmd::Command::Move && msg->Move_mode == px4_cmd::Command::Custom_Command)
        {
            // set hover to false
            hover = false;
            
            // convert origin message to custom command
            px4_msg_to_custom_command(vehicle_command, cmd);

            // check command
            if (!check_custom_command(cmd, vehicle_type ,error))
            {
                // if not pass check, set mode to hover
                if (vehicle_command.Vehicle == px4_cmd::Command::FixWing)
                {
                    vehicle_command.Mode = px4_cmd::Command::Loiter;
                }
                else
                {
                    vehicle_command.Mode = px4_cmd::Command::Hover;
                }
                std::cout << error << std::endl;
                break;
            }

            // get frame
            int frame;
            switch (vehicle_command.Move_frame)
            {
                case px4_cmd::Command::ENU:
                {
                    frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                    break;
                }

                case px4_cmd::Command::BODY:
                {
                    frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
                    break;
                }

                case px4_cmd::Command::GLOBAL:
                {
                    frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
                }
            }

            // generate message
            switch (cmd.mode)
            {
                case CommandMode::TargetLocal:
                {
                    // init typemask
                    pos_setpoint.type_mask = 0;

                    // frame
                    pos_setpoint.coordinate_frame = frame;
                    pos_setpoint.header.frame_id = frame;

                    // position
                    pos_setpoint.position.x = cmd.position[0] - init_x;
                    pos_setpoint.position.y = cmd.position[1] - init_y;
                    pos_setpoint.position.z = cmd.position[2] - init_z;
                    pos_setpoint.type_mask += isnan(cmd.position[0]) ? mavros_msgs::PositionTarget::IGNORE_PX : 0;
                    pos_setpoint.type_mask += isnan(cmd.position[1]) ? mavros_msgs::PositionTarget::IGNORE_PY : 0;
                    pos_setpoint.type_mask += isnan(cmd.position[2]) ? mavros_msgs::PositionTarget::IGNORE_PZ : 0;

                    // fixwing only support position setpoint
                    if (vehicle_command.Vehicle == px4_cmd::Command::FixWing)
                    {
                        pos_setpoint.type_mask = cmd.fw_mode;
                        break;
                    }
                    
                    // velocity
                    pos_setpoint.velocity.x = cmd.velocity[0];
                    pos_setpoint.velocity.y = cmd.velocity[1];
                    pos_setpoint.velocity.z = cmd.velocity[2];
                    pos_setpoint.type_mask += isnan(cmd.velocity[0]) ? mavros_msgs::PositionTarget::IGNORE_VX : 0;
                    pos_setpoint.type_mask += isnan(cmd.velocity[1]) ? mavros_msgs::PositionTarget::IGNORE_VY : 0;
                    pos_setpoint.type_mask += isnan(cmd.velocity[2]) ? mavros_msgs::PositionTarget::IGNORE_VZ : 0;

                    // accelerate
                    pos_setpoint.acceleration_or_force.x = cmd.accelerate[0];
                    pos_setpoint.acceleration_or_force.y = cmd.accelerate[1];
                    pos_setpoint.acceleration_or_force.z = cmd.accelerate[2];
                    pos_setpoint.type_mask += isnan(cmd.accelerate[0]) ? mavros_msgs::PositionTarget::IGNORE_AFX : 0;
                    pos_setpoint.type_mask += isnan(cmd.accelerate[1]) ? mavros_msgs::PositionTarget::IGNORE_AFY : 0;
                    pos_setpoint.type_mask += isnan(cmd.accelerate[2]) ? mavros_msgs::PositionTarget::IGNORE_AFZ : 0;

                    // Force flag
                    pos_setpoint.type_mask += cmd.force_flag ? mavros_msgs::PositionTarget::FORCE : 0;

                    // yaw and yaw rate
                    pos_setpoint.yaw = cmd.yaw;
                    pos_setpoint.yaw_rate = cmd.yaw_rate;
                    pos_setpoint.type_mask += isnan(cmd.yaw) ? mavros_msgs::PositionTarget::IGNORE_YAW : 0;
                    pos_setpoint.type_mask += isnan(cmd.yaw_rate) ? mavros_msgs::PositionTarget::IGNORE_YAW_RATE : 0;
                    break;
                }

                case CommandMode::TargetGlobal:
                {
                    // init typemask
                    pos_setpoint_global.type_mask = 0;

                    // frame
                    pos_setpoint_global.coordinate_frame = frame;
                    pos_setpoint_global.header.frame_id = frame;

                    // position
                    pos_setpoint_global.latitude = cmd.position[0];
                    pos_setpoint_global.longitude = cmd.position[1];
                    pos_setpoint_global.altitude = cmd.position[2];
                    pos_setpoint_global.type_mask += isnan(cmd.position[0]) ? mavros_msgs::GlobalPositionTarget::IGNORE_LATITUDE : 0;
                    pos_setpoint_global.type_mask += isnan(cmd.position[1]) ? mavros_msgs::GlobalPositionTarget::IGNORE_LONGITUDE : 0;
                    pos_setpoint_global.type_mask += isnan(cmd.position[2]) ? mavros_msgs::GlobalPositionTarget::IGNORE_ALTITUDE : 0;

                    // fixwing only support position setpoint
                    if (vehicle_command.Vehicle == px4_cmd::Command::FixWing)
                    {
                        pos_setpoint_global.type_mask = cmd.fw_mode;
                        break;
                    }

                    // velocity
                    pos_setpoint_global.velocity.x = cmd.velocity[0];
                    pos_setpoint_global.velocity.y = cmd.velocity[1];
                    pos_setpoint_global.velocity.z = cmd.velocity[2];
                    pos_setpoint_global.type_mask += isnan(cmd.velocity[0]) ? mavros_msgs::GlobalPositionTarget::IGNORE_VX : 0;
                    pos_setpoint_global.type_mask += isnan(cmd.velocity[1]) ? mavros_msgs::GlobalPositionTarget::IGNORE_VY : 0;
                    pos_setpoint_global.type_mask += isnan(cmd.velocity[2]) ? mavros_msgs::GlobalPositionTarget::IGNORE_VZ : 0;

                    // accelerate
                    pos_setpoint_global.acceleration_or_force.x = cmd.accelerate[0];
                    pos_setpoint_global.acceleration_or_force.y = cmd.accelerate[1];
                    pos_setpoint_global.acceleration_or_force.z = cmd.accelerate[2];
                    pos_setpoint_global.type_mask += isnan(cmd.accelerate[0]) ? mavros_msgs::GlobalPositionTarget::IGNORE_AFX : 0;
                    pos_setpoint_global.type_mask += isnan(cmd.accelerate[1]) ? mavros_msgs::GlobalPositionTarget::IGNORE_AFY : 0;
                    pos_setpoint_global.type_mask += isnan(cmd.accelerate[2]) ? mavros_msgs::GlobalPositionTarget::IGNORE_AFZ : 0;

                    // Force flag
                    pos_setpoint_global.type_mask += cmd.force_flag ? mavros_msgs::GlobalPositionTarget::FORCE : 0;

                    // yaw and yaw rate
                    pos_setpoint_global.yaw = cmd.yaw;
                    pos_setpoint_global.yaw_rate = cmd.yaw_rate;
                    pos_setpoint_global.type_mask += isnan(cmd.yaw) ? mavros_msgs::GlobalPositionTarget::IGNORE_YAW : 0;
                    pos_setpoint_global.type_mask += isnan(cmd.yaw_rate) ? mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE : 0;
                    break;
                }

                case CommandMode::TargetAttitude:
                {
                    // init typemask
                    attitude_setpoint.type_mask = 0;

                    // attitude
                    attitude_setpoint.orientation.x = cmd.attitude[0];
                    attitude_setpoint.orientation.y = cmd.attitude[1];
                    attitude_setpoint.orientation.z = cmd.attitude[2];
                    attitude_setpoint.orientation.w = cmd.attitude[3];
                    attitude_setpoint.type_mask += isnan(cmd.attitude[0]) ? mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE : 0;

                    // attitude rate
                    attitude_setpoint.body_rate.x = cmd.attitude_rate[0];
                    attitude_setpoint.body_rate.y = cmd.attitude_rate[1];
                    attitude_setpoint.body_rate.z = cmd.attitude_rate[2];
                    attitude_setpoint.type_mask += isnan(cmd.attitude_rate[0]) ? mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE : 0;
                    attitude_setpoint.type_mask += isnan(cmd.attitude_rate[1]) ? mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE : 0;
                    attitude_setpoint.type_mask += isnan(cmd.attitude_rate[2]) ? mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE : 0;

                    // thrust
                    attitude_setpoint.thrust = cmd.thrust;
                    attitude_setpoint.type_mask += isnan(cmd.thrust) ? mavros_msgs::AttitudeTarget::IGNORE_THRUST : 0;
                    break;
                }
            }

            // update publisher
            if (current_custom_mode != cmd.mode)
            {
                pub_mutex.lock();
                current_custom_mode = cmd.mode;
                setpoint_raw_pub.shutdown();
                switch (cmd.mode)
                {
                    case CommandMode::TargetLocal:
                    {
                        setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>((topic_header + "setpoint_raw/local").c_str(), 50);
                        break;
                    }

                    case CommandMode::TargetGlobal:
                    {
                        setpoint_raw_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>((topic_header + "setpoint_raw/global").c_str(), 50);
                        break;
                    }

                    case CommandMode::TargetAttitude:
                    {
                        setpoint_raw_pub = nh.advertise<mavros_msgs::AttitudeTarget>((topic_header + "setpoint_raw/attitude").c_str(), 50);
                        break;
                    }
                }
                pub_mutex.unlock();
            }

            // send command
            switch (cmd.mode)
            {
                case CommandMode::TargetLocal:
                {
                    setpoint_raw_pub.publish(pos_setpoint);
                    break;
                }

                case CommandMode::TargetGlobal:
                {
                    setpoint_raw_pub.publish(pos_setpoint_global);
                    break;
                }

                case CommandMode::TargetAttitude:
                {
                    setpoint_raw_pub.publish(attitude_setpoint);
                    break;
                }
            }

            // custom command end
            cmd_mutex.unlock();
            return;
        }
        
        // non-custom command mode be local mode
        if (current_custom_mode != CommandMode::TargetLocal)
        {
            pub_mutex.lock();
            setpoint_raw_pub.shutdown();
            setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>((topic_header + "setpoint_raw/local").c_str(), 50);
            pub_mutex.unlock();
        }
        current_custom_mode = CommandMode::TargetLocal;

        // judge if achieve desire cmd
        double dx = x - vehicle_command.desire_cmd[0] + init_x;
        double dy = y - vehicle_command.desire_cmd[1] + init_y;
        double dz = z - vehicle_command.desire_cmd[2] + init_z;
        if (msg->Mode == px4_cmd::Command::Move && state_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
        {
            if (abs(dx) < sigma && abs(dy) < sigma && abs(dz) < sigma)
            {
                desire_time++;
            }
            else
            {
                achieve_desire = false;
                desire_time = 0;
            }
            if (desire_time >= 5)
            {
                achieve_desire = true;
            }
        }
        else
        {
            desire_time = 0;
        }
        // reset home position when land
        if (state_mode == mavros_msgs::State::MODE_PX4_LAND && !arm_state && (msg->Move_mode == px4_cmd::Command::XYZ_POS || msg->Move_mode == px4_cmd::Command::XYZ_REL_POS))
        {
            home_position[0] = msg->desire_cmd[0] - init_x;
            home_position[1] = msg->desire_cmd[1] - init_y;
        }
        // 设定坐标系
        switch (vehicle_command.Move_frame)
        {
            case px4_cmd::Command::ENU:
            {
                pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                pos_setpoint.header.frame_id = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                break;
            }

            case px4_cmd::Command::BODY:
            {
                pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
                pos_setpoint.header.frame_id = mavros_msgs::PositionTarget::FRAME_BODY_NED;
                break;
            }
        }

        // 设定输入值
        // Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
        // Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
        // Bit 10 should set to 0, means is not force sp
        //

        int angle_mask = mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
        if (vehicle_command.yaw_rate_cmd != 0)
        {
            angle_mask = mavros_msgs::GlobalPositionTarget::IGNORE_YAW;
        }

        if (vehicle_command.Vehicle == px4_cmd::Command::Multicopter)
        {
            if (vehicle_command.Mode == px4_cmd::Command::Hover)
            {
                if (!hover)
                {
                    hover_pos[0] = x;
                    hover_pos[1] = y;
                    hover_pos[2] = z;
                    hover = true;
                }
                pos_setpoint.type_mask = 0b000111111000 + angle_mask;
                pos_setpoint.position.x = hover_pos[0];
                pos_setpoint.position.y = hover_pos[1];
                pos_setpoint.position.z = hover_pos[2];
                cmd_mutex.unlock();
                return;
            }
            hover = false;

            if (vehicle_command.Mode == px4_cmd::Command::Takeoff)
            {
                pos_setpoint.type_mask = 0b000111111000;
                pos_setpoint.position.x = home_position[0];
                pos_setpoint.position.y = home_position[1];
                pos_setpoint.position.z = vehicle_command.desire_cmd[2];
                cmd_mutex.unlock();
                return;
            }

            switch (vehicle_command.Move_mode)
            {
                case px4_cmd::Command::XYZ_POS:
                {
                    pos_setpoint.type_mask = 0b000111111000 + angle_mask;
                    pos_setpoint.position.x = vehicle_command.desire_cmd[0];
                    pos_setpoint.position.y = vehicle_command.desire_cmd[1];
                    pos_setpoint.position.z = vehicle_command.desire_cmd[2];
                    break;
                }

                case px4_cmd::Command::XY_VEL_Z_POS:
                {
                    pos_setpoint.type_mask = 0b000111100011 + angle_mask;
                    pos_setpoint.velocity.x = vehicle_command.desire_cmd[0];
                    pos_setpoint.velocity.y = vehicle_command.desire_cmd[1];
                    pos_setpoint.position.z = vehicle_command.desire_cmd[2];
                    break;
                }

                case px4_cmd::Command::XYZ_VEL:
                {
                    pos_setpoint.type_mask = 0b000111000111 + angle_mask;
                    pos_setpoint.velocity.x = vehicle_command.desire_cmd[0];
                    pos_setpoint.velocity.y = vehicle_command.desire_cmd[1];
                    pos_setpoint.velocity.z = vehicle_command.desire_cmd[2];
                    break;
                }

                case px4_cmd::Command::XYZ_REL_POS:
                {
                    pos_setpoint.type_mask = 0b000111111000 + angle_mask;
                    pos_setpoint.position.x = vehicle_command.desire_cmd[0];
                    pos_setpoint.position.y = vehicle_command.desire_cmd[1];
                    pos_setpoint.position.z = vehicle_command.desire_cmd[2];
                    break;
                }
            }
            pos_setpoint.yaw_rate = vehicle_command.yaw_rate_cmd;
            pos_setpoint.yaw = vehicle_command.yaw_cmd;
        }
        // 固定翼信息
        if (vehicle_command.Vehicle == px4_cmd::Command::FixWing)
        {
            if (vehicle_command.Mode == px4_cmd::Command::Loiter)
            {
                if (!hover)
                {
                    hover_pos[0] = x;
                    hover_pos[1] = y;
                    hover_pos[2] = z;
                    hover = true;
                }
                pos_setpoint.type_mask = 12288;
                pos_setpoint.position.x = hover_pos[0];
                pos_setpoint.position.y = hover_pos[1];
                pos_setpoint.position.z = hover_pos[2];
                cmd_mutex.unlock();
                return;
            }
            hover = false;

            if (vehicle_command.Mode == px4_cmd::Command::Takeoff)
            {
                pos_setpoint.type_mask = 4096;
                pos_setpoint.position.x = home_position[0];
                pos_setpoint.position.y = home_position[1];
                pos_setpoint.position.z = vehicle_command.desire_cmd[2];
                cmd_mutex.unlock();
                return;
            }

            if (vehicle_command.Mode == px4_cmd::Command::Gliding)
            {
                pos_setpoint.type_mask = 0b000100100100;
            }

            switch (vehicle_command.Move_mode)
            {
                case px4_cmd::Command::XYZ_POS:
                {
                    pos_setpoint.type_mask = 0b110111111000;
                    break;
                }

                case px4_cmd::Command::XYZ_REL_POS:
                {
                    pos_setpoint.type_mask = 0b110111111000;
                    break;
                }
            }
            pos_setpoint.position.x = vehicle_command.desire_cmd[0];
            pos_setpoint.position.y = vehicle_command.desire_cmd[1];
            pos_setpoint.position.z = vehicle_command.desire_cmd[2];
        }
        cmd_mutex.unlock();
    }

    void UAVBridge::state_cb(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state = *msg;
        state_mode = msg->mode;
        arm_state = msg->armed;
    }

    void UAVBridge::extend_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
    {
        // get landed state
        if (msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND || msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED)
        {
            land_state = true;
        }
        else
        {
            land_state = false;
        }
        // get vehicle state
        switch (msg->landed_state)
        {
            case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
            {
                state = "Landed";
                break;
            }

            case mavros_msgs::ExtendedState::LANDED_STATE_LANDING:
            {
                state = "Landing";
                break;
            }

            case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
            {
                state = "Flying";
                break;
            }

            case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF:
            {
                state = "TakeOff";
                break;
            }

            case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED:
            {
                state = "UnDefined";
                break;
            }
        }
    }

    void UAVBridge::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        tf::quaternionMsgToTF(msg->pose.orientation, quat);
        position_mutex.lock();
        quaternion[0] = msg->pose.orientation.x;
        quaternion[1] = msg->pose.orientation.y;
        quaternion[2] = msg->pose.orientation.z;
        quaternion[3] = msg->pose.orientation.w;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        x = init_x + msg->pose.position.x;
        y = init_y + msg->pose.position.y;
        z = init_z + msg->pose.position.z;
        pitch += init_pitch;
        roll += init_roll;
        yaw += init_yaw;
        position_mutex.unlock();
    }

    void UAVBridge::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        velocity_mutex.lock();
        vx = msg->twist.linear.x;
        vy = msg->twist.linear.y;
        vz = msg->twist.linear.z;
        velocity_mutex.unlock();
    }
};

#endif