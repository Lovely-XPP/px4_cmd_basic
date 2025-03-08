// Copyright (c) 2024 易鹏 中山大学航空航天学院
// Copyright (c) 2024 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef UAVMESSAGE_HPP
#define UAVMESSAGE_HPP

#include "NlohmannJSON.hpp"

namespace UAV
{
    /// @brief UAV Information
    struct UAVInfo
    {
        /// @brief UAV id
        unsigned int id;
        /// @brief UAV state
        std::string state;
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

        /// @brief arm state
        bool arm_state = false;
        /// @brief arm state
        bool land_state = false;

        /// @brief valid flag
        bool valid = false;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(UAVInfo, id, state, x, y, z, vx, vy, vz, quaternion, pitch, roll, yaw, x_real, y_real, z_real, vx_real, vy_real, vz_real, quaternion_real, pitch_real, roll_real, yaw_real, arm_state, land_state, valid);
    };

    /// @brief UAV Command
    enum CommandType
    {
        TakeOff = 1,    // Takeoff command
        Land = 2,       // Land command
        Return = 3,     // Return command
        TaskCommand = 4 // Task command
    };

    /// @brief UAV Frame id
    enum FrameId
    {
        ENU = 0,
        BODY = 1,
        GLOBAL = 2
    };

    /// @brief UAV custom command mode
    enum CommandMode
    {
        TargetLocal = 1,   // local frame with position / velocity / accelerate setpoint
        TargetGlobal = 2,  // global frame with position / velocity setpoint (lat / lon / alt)
        TargetAttitude = 3 // Attitude control with thrust
    };

    /// @brief UAV FixWing only support position setpoint in local & global mode, and need speicify position mode
    enum FixWingPositionMode : unsigned long
    {
        // Gliding Mode.
        // This configures TECS to prioritize airspeed over altitude in order to make the vehicle glide when there is no thrust (i.e. pitch is controlled to regulate airspeed).
        GlidingMode = 292,
        // Takeoff Mode
        TakeoffMode = 4096,
        // Land Mode
        LandMode = 8192,
        // Loiter Mode (fly a circle centred on setpoint).
        LoiterMode = 12288,
        // Idle Mode (zero throttle, zero roll / pitch).
        IdleMode = 16384
    };

    /// @brief Custom command for vehicle with px4
    struct UAVCommand
    {
        /// @brief UAV id, -1 for all uavs (only valid in TakeOff / Land / Return CommandType)
        int id;
        // command type: TakeOff / Land / TaskCommand
        CommandType type;
        // command mode: local / global / attitude
        CommandMode mode;
        // FixWing only support position setpoint in local & global mode, and need specify position mode
        FixWingPositionMode fw_mode;
        // frame id in px4_cmd message
        FrameId frame_id;
        /// @brief TakeOff Height (m)
        double takeoff_height = NAN;
        /// @brief x Axis Position (m) in Global Frame
        double x = NAN;
        /// @brief y Axis Position (m) in Global Frame
        double y = NAN;
        /// @brief z Axis Position (m) in Global Frame
        double z = NAN;
        /// @brief x Axis Velocity (m/s) in Global Frame
        double vx = NAN;
        /// @brief y Axis Velocity (m/s) in Global Frame
        double vy = NAN;
        /// @brief z Axis Velocity (m/s) in Global Frame
        double vz = NAN;
        /// @brief x Axis Acceleration (m/s^2) in Global Frame
        double ax = NAN;
        /// @brief y Axis Acceleration (m/s^2) in Global Frame
        double ay = NAN;
        /// @brief z Axis Acceleration (m/s^2) in Global Frame
        double az = NAN;
        // flag for accelerate to force
        bool force_flag = false;
        // yaw setpoint
        double yaw = NAN;
        // yaw rate setpoint
        double yaw_rate = NAN;
        // attitude setpoint [quaternion {xyzw}]
        double quaternion[4] = {
            NAN,
            NAN,
            NAN,
            NAN};
        // attitude rate setpoint
        double attitude_rate[3] = {
            NAN,
            NAN,
            NAN};
        // thrust setpoint
        double thrust = NAN;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(UAVCommand, id, type, mode, fw_mode, frame_id, takeoff_height, x, y, z, vx, vy, vz, ax, ay, az, force_flag, yaw, yaw_rate, quaternion, attitude_rate, thrust);
    };

    struct UAVInit
    {
        /// @brief UAV id
        unsigned int id;
        /// @brief initial x Axis Position (m) in Global Frame
        double init_x = NAN;
        /// @brief initial y Axis Position (m) in Global Frame
        double init_y = NAN;
        /// @brief initial z Axis Position (m) in Global Frame
        double init_z = NAN;
        /// @brief initial roll angle (rad)
        double init_roll = NAN;
        /// @brief initial pitch angle (rad)
        double init_pitch = NAN;
        /// @brief initial yaw angle (rad)
        double init_yaw = NAN;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(UAVInit, id, init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
    };

    void resetUAVCommand(UAVCommand &cmd)
    {
        cmd.x = NAN;
        cmd.y = NAN;
        cmd.z = NAN;
        cmd.vx = NAN;
        cmd.vy = NAN;
        cmd.vz = NAN;
        cmd.ax = NAN;
        cmd.ay = NAN;
        cmd.az = NAN;
        cmd.force_flag = false;
        cmd.yaw = NAN;
        cmd.yaw_rate = NAN;
        cmd.quaternion[0] = NAN;
        cmd.quaternion[1] = NAN;
        cmd.quaternion[2] = NAN;
        cmd.quaternion[3] = NAN;
        cmd.attitude_rate[0] = NAN;
        cmd.attitude_rate[1] = NAN;
        cmd.attitude_rate[2] = NAN;
        cmd.thrust = NAN;
    };

    void setUAVPosition(UAVCommand &cmd, int id, double x, double y, double z, FrameId frameId = ENU)
    {
        resetUAVCommand(cmd);
        cmd.id = id;
        cmd.type = TaskCommand;
        cmd.mode = TargetLocal;
        cmd.frame_id = frameId;
        cmd.x = x;
        cmd.y = y;
        cmd.z = z;
    };

    void setUAVVelocity(UAVCommand &cmd, int id, double vx, double vy, double vz, FrameId frameId = ENU)
    {
        resetUAVCommand(cmd);
        cmd.id = id;
        cmd.type = TaskCommand;
        cmd.mode = TargetLocal;
        cmd.frame_id = frameId;
        cmd.vx = vx;
        cmd.vy = vy;
        cmd.vz = vz;
    };

    void setUAVVelocityWithHeight(UAVCommand &cmd, int id, double vx, double vy, double z, FrameId frameId = ENU)
    {
        resetUAVCommand(cmd);
        cmd.id = id;
        cmd.type = TaskCommand;
        cmd.mode = TargetLocal;
        cmd.frame_id = frameId;
        cmd.vx = vx;
        cmd.vy = vy;
        cmd.z = z;
    };
}

#endif