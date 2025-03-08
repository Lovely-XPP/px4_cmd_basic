// Copyright (c) 2024 易鹏 中山大学航空航天学院
// Copyright (c) 2024 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef PX4_CMD_RPC_SERVER_H
#define PX4_CMD_RPC_SERVER_H
#include <string>
#include <vector>
#include <px4_cmd_rpc_server/UAVBridge.hpp>
#include <px4_cmd_rpc_server/UAVMessage.hpp>
#include <px4_cmd/Command.h>

using namespace tinyxml2;

/// @brief GetRPCIP
std::string getIP();

/// @brief create new process
/// @param path path for bin
/// @param cmd command
/// @param arg args
/// @return RPC RETURN CODE
int new_process(const char *path, const char *cmd, const char *arg);

/// @brief initial agents
/// @param init_json_string vector<UAV::UAVInit> json serial std::sting
/// @return RPC Message: std::vector<PX4RPCServerMsg>
std::string initial(std::string init_json_string);

/// @brief Stop Simulation
/// @return RPC Message
std::string stop();

/// @brief Stop RPC Server and Exit
void exit_();

/// @brief UAV Command Center
/// @param cmds_json_string vector<UAV::UAVCommand> json serial std::string
/// @return RPC Message: std::vector<PX4RPCServerMsg>
std::string UAVCmdCenter(std::string cmds_json_string);

/// @brief set uavs command function
/// @param cmds_json_string vector<UAV::UAVCommand> json serial std::string
/// @return RPC Message: std::vector<PX4RPCServerMsg>
std::string set_uavs_cmd(std::string cmds_json_string);

/// @brief takeoff command implement function
/// @param height takeoff height
/// @return RPC Message: std::vector<PX4RPCServerMsg>
std::string takeoff_uavs_cmd(double height);

/// @brief land command implement function
/// @return RPC Message: std::vector<PX4RPCServerMsg>
std::string land_uavs_cmd();

/// @brief return command implement function
/// @return RPC Message: std::vector<PX4RPCServerMsg>
std::string return_uavs_cmd();

/// @brief takeoff command implement function
/// @param id UAV id
/// @param height takeoff height
/// @return RPC Message: PX4RPCServerMsg
std::string takeoff_uav_cmd(int id, double height);

/// @brief land command implement function
/// @param id UAV id
/// @return RPC Message: PX4RPCServerMsg
std::string land_uav_cmd(int id);

/// @brief return command implement function
/// @param id UAV id
/// @return RPC Message: PX4RPCServerMsg
std::string return_uav_cmd(int id);

/// @brief takeoff command implement thread function
/// @param id UAV id
/// @param height takeoff height
void takeoff_thread_func(int id, double height);

/// @brief land command implement thread function
/// @param id UAV id
void land_thread_func(int id);

/// @brief return command implement thread function
/// @param id UAV id
void return_thread_func(int id);

/// @brief Get All UAV information
/// @return UAV information: std::vector<UAVInfo>
std::string get_uavs_info();

/// @brief Get a UAV information
/// @return UAV information: UAVInfo
std::string get_uav_info(int idx);

/// @brief Get total count of UAVs
/// @return count of UAVs
int get_uavs_count();

/// @brief UAVCommand data -> CustomCommand
/// @param cmd UAVCommand
/// @return CustomCommand
CustomCommand UAVCommand2CustomCommand(const UAV::UAVCommand &cmd);

/// @brief detect if all uavs cmd exec done
/// @return done flag
bool detect_cmd_exec_done();

/// @brief validate id input
/// @param id UAV id
/// @return RPC RETURN CODE
int validate_id(int id);

#endif