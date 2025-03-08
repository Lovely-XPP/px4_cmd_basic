// Copyright (c) 2024 易鹏 中山大学航空航天学院
// Copyright (c) 2024 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <rpc/server.h>
#include <tinyxml2.h>
#include <px4_cmd_rpc_server/px4_cmd_rpc_server.h>
#include <px4_cmd_rpc_server/rpc_return_code_definations.hpp>
#include <px4_cmd_rpc_server/NlohmannJSON.hpp>
#include <string>
#include <map>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <memory>
#include <thread>

// global varibles
// Number of agents 无人机个数
int agents_count = 0;
// roslaunch 的进程pid数组
std::vector<pid_t> ros_launch_pids = {};
// UAV 接受 / 发送指令API集合
std::vector<std::unique_ptr<UAV::UAVBridge>> UAVs = {};
// UAV px4_cmd message
std::vector<px4_cmd::Command> UAVsCmd = {};
// UAV px4_cmd message
std::vector<bool> UAVsCmdResult = {};
std::map<int, int> UAVIDMap = {};

// 初始化标志
bool init_flag = false;
// 起飞标志
bool takeoff_flag = false;

int main(int argc, char *argv[])
{
    // init ros node
    ros::init(argc, argv, "px4_cmd_rpc_server");

    // init rpc server
    rpc::server rpc_server(56789);

    // bind functions
    rpc_server.bind("init", &initial);
    rpc_server.bind("stop", &stop);
    rpc_server.bind("exit", &exit_);
    rpc_server.bind("get_uav_info", &get_uav_info);
    rpc_server.bind("get_uavs_info", &get_uavs_info);
    rpc_server.bind("get_uavs_count", &get_uavs_count);
    rpc_server.bind("land_uavs", &land_uavs_cmd);
    rpc_server.bind("takeoff_uavs", &takeoff_uavs_cmd);
    rpc_server.bind("return_uavs", &return_uavs_cmd);
    rpc_server.bind("land_uav", &land_uav_cmd);
    rpc_server.bind("takeoff_uav", &takeoff_uav_cmd);
    rpc_server.bind("return_uav", &return_uav_cmd);
    rpc_server.bind("set_uavs_cmd", &set_uavs_cmd);
    rpc_server.bind("UAVCmdRequest", &UAVCmdCenter);
    rpc_server.bind("UAVInfoRequest", &get_uavs_info);
    rpc_server.bind("UAVInitRequest", &initial);
    rpc_server.bind("UAVStopRequest", &stop);
    rpc_server.bind("UAVExitRequest", &exit_);

    // run server
    std::cout << "[Info] PX4 CMD RPC Server running in [" << getIP()  << ":56789]" << std::endl;
    rpc_server.run();

    return 0;
}

std::string UAVCmdCenter(std::string cmds_json_string)
{
    // return msgs init
    std::vector<PX4RPCServerMsg> return_msgs = {};
    nlohmann::json return_msgs_json;
    bool wait_for_exec = true;

    // if not init, reject cmds
    if (!init_flag)
    {
        return_msgs.emplace_back(generateMsg(-1, NOT_INIT));
        return_msgs_json = return_msgs;
        return return_msgs_json.dump();
    }

    // transform data
    std::vector<UAV::UAVCommand> cmds = {};
    try
    {
        nlohmann::json cmds_json = nlohmann::json::parse(cmds_json_string);
        cmds = cmds_json.get<std::vector<UAV::UAVCommand>>();
    }
    catch(const std::exception& e)
    {
        return_msgs.emplace_back(generateMsg(-1, JSON_INPUT_ERROR, e.what()));
        return_msgs_json = return_msgs;
        return return_msgs_json.dump();
    }

    // judge for exec for all uavs
    if (cmds.size() == 1 && cmds[0].id == -1 && cmds[0].type != UAV::CommandType::TaskCommand)
    {
        auto cmd = cmds[0];
        switch (cmd.type)
        {
            case UAV::CommandType::TakeOff:
            {
                return takeoff_uavs_cmd(cmd.takeoff_height);
            }

            case UAV::CommandType::Land:
            {
                return land_uavs_cmd();
            }

            case UAV::CommandType::Return:
            {
                return return_uavs_cmd();
            }
        }
    }
    
    // exec every cmd
    for (auto cmd : cmds)
    {
        // validate id
        if (validate_id(cmd.id) == ID_ERROR)
        {
            continue;
        }
        int array_id = UAVIDMap[cmd.id];

        // switch cmd type
        switch (cmd.type)
        {
            case UAV::TakeOff:
            {
                // check takeoff height input
                if (isnan(cmd.takeoff_height))
                {
                    continue;
                }
                // height at least 0.5 m
                if (cmd.takeoff_height < 0.5)
                {
                    continue;
                }
                UAVsCmdResult[array_id] = false;
                std::thread takeoff_thread(&takeoff_thread_func, cmd.id, cmd.takeoff_height);
                takeoff_thread.detach();
                break;
            }

            case UAV::Land:
            {
                UAVsCmdResult[array_id] = false;
                std::thread land_thread(&land_thread_func, cmd.id);
                land_thread.detach();
                break;
            }

            case UAV::Return:
            {
                UAVsCmdResult[array_id] = false;
                std::thread land_thread(&return_thread_func, cmd.id);
                land_thread.detach();
                break;
            }

            case UAV::TaskCommand:
            {
                CustomCommand cmd_ = UAVCommand2CustomCommand(cmd);
                std::string err;
                bool success = check_custom_command(cmd_, UAVs[array_id]->vehicle_type, err);
                if (success)
                {
                    wait_for_exec = false;
                    px4_cmd::Command cmd__;
                    custom_command_to_px4_msg(cmd_, cmd__);
                    UAVs[array_id]->set_command(&cmd__);
                    return_msgs.emplace_back(generateMsg(cmd.id, SET_COMMAND_SUCCESS));
                    continue;
                }
                return_msgs.emplace_back(generateMsg(cmd.id, COMMAND_FORMAT_ERROR, err));
                break;
            }
        }
    }
    
    // wait for thread exit
    if (wait_for_exec)
    {
        while (!detect_cmd_exec_done())
        {
            usleep(500 * 1000);
        }
        usleep(500 * 1000);
    }

    // generate msgs
    for (auto cmd : cmds)
    {
        // validate id
        if (validate_id(cmd.id) == ID_ERROR)
        {
            return_msgs.emplace_back(generateMsg(cmd.id, ID_ERROR));
            continue;
        }
        int array_id = UAVIDMap[cmd.id];

        // switch cmd type
        switch (cmd.type)
        {
            case UAV::TakeOff:
            {
                // check takeoff height input
                if (isnan(cmd.takeoff_height))
                {
                    return_msgs.emplace_back(generateMsg(cmd.id, HEIGHT_NAN_ERROR));
                    continue;
                }
                // height at least 0.5 m
                if (cmd.takeoff_height < 0.5)
                {
                    return_msgs.emplace_back(generateMsg(cmd.id, HEIGHT_ERROR));
                    continue;
                }
                // check takeoff state
                if (!UAVs[array_id]->arm_state || UAVs[array_id]->state_mode != "OFFBOARD")
                {
                    return_msgs.emplace_back(generateMsg(cmd.id, TAKEOFF_FAILURE));
                    continue;
                }
                return_msgs.emplace_back(generateMsg(cmd.id, TAKEOFF_SUCCESS));
                break;
            }

            case UAV::Land:
            {
                // validate land mode
                if (UAVs[array_id]->state_mode != "AUTO.LAND")
                {
                    return_msgs.emplace_back(generateMsg(cmd.id, LAND_FAILURE));
                    continue;
                }
                return_msgs.emplace_back(generateMsg(cmd.id, LAND_SUCCESS));
                break;
            }

            case UAV::Return:
            {
                if (UAVs[array_id]->state_mode != "AUTO.LAND")
                {
                    return_msgs.emplace_back(generateMsg(cmd.id, LAND_FAILURE));
                    continue;
                }
                return_msgs.emplace_back(generateMsg(cmd.id, LAND_SUCCESS));
                break;
            }
        }
    }

    // transform msgs to string
    return_msgs_json = return_msgs;
    return return_msgs_json.dump();
}

std::string set_uavs_cmd(std::string cmds_json_string)
{
    std::vector<PX4RPCServerMsg> msgs = {};
    std::vector<UAV::UAVCommand> cmds = {};
    nlohmann::json msgs_json;
    try
    {
        nlohmann::json cmds_json = nlohmann::json::parse(cmds_json_string);
        cmds = cmds_json.get<std::vector<UAV::UAVCommand>>();
    }
    catch (const std::exception &e)
    {
        msgs.emplace_back(generateMsg(-1, JSON_INPUT_ERROR, e.what()));
        msgs_json = msgs;
        return msgs_json.dump();
    }
    for (auto cmd : cmds)
    {
        CustomCommand cmd_ = UAVCommand2CustomCommand(cmd);
        std::string err;
        int array_id = UAVIDMap[cmd.id];
        bool success = check_custom_command(cmd_, UAVs[array_id]->vehicle_type, err);
        if (success)
        {
            px4_cmd::Command cmd__;
            custom_command_to_px4_msg(cmd_, cmd__);
            UAVs[array_id]->set_command(&cmd__);
            msgs.emplace_back(generateMsg(cmd.id, SET_COMMAND_SUCCESS));
            continue;
        }
        msgs.emplace_back(generateMsg(cmd.id, COMMAND_FORMAT_ERROR, err));
    }
    msgs_json = msgs;
    return msgs_json.dump();
}

std::string takeoff_uav_cmd(int id, double height) 
{
    // if not init, reject takeoff
    if (!init_flag)
    {
        return generateMsgString(id, NOT_INIT);
    }

    // validate id
    if (validate_id(id) == ID_ERROR)
    {
        return generateMsgString(id, ID_ERROR);
    }

    // minimum height
    if (height < 0.5)
    {
        return generateMsgString(id, HEIGHT_ERROR);
    }

    int array_id = UAVIDMap[id];
    UAVsCmdResult[array_id] = false;
    takeoff_thread_func(id, height);

    // wait for thread exit
    while (!detect_cmd_exec_done())
    {
        usleep(500 * 1000);
    }
    sleep(500 * 1000);

    if (!UAVs[array_id]->arm_state || UAVs[array_id]->state_mode != "OFFBOARD")
    {
        return generateMsgString(id, TAKEOFF_FAILURE);
    }

    return generateMsgString(id, TAKEOFF_SUCCESS);
}

std::string land_uav_cmd(int id)
{
    // if not init, reject land
    if (!init_flag)
    {
        return generateMsgString(id, NOT_INIT);
    }

    // validate id
    if (validate_id(id) == ID_ERROR)
    {
        return generateMsgString(id, ID_ERROR);
    }

    // exec land cmd
    int array_id = UAVIDMap[id];
    UAVsCmdResult[array_id] = false;
    land_thread_func(id);

    // wait for thread exit
    while (!detect_cmd_exec_done())
    {
        usleep(500 * 1000);
    }
    usleep(500 * 1000);

    // validate land mode
    if (UAVs[array_id]->state_mode != "AUTO.LAND")
    {
        return generateMsgString(id, LAND_FAILURE);
    }

    return generateMsgString(id, LAND_SUCCESS);
}

std::string return_uav_cmd(int id)
{
    // if not init, reject return
    if (!init_flag)
    {
        return generateMsgString(id, NOT_INIT);
    }

    // validate id
    if (validate_id(id) == ID_ERROR)
    {
        return generateMsgString(id, ID_ERROR);
    }

    // exec land cmd
    int array_id = UAVIDMap[id];
    UAVsCmdResult[array_id] = false;
    return_thread_func(id);

    // wait for thread exit
    while (!detect_cmd_exec_done())
    {
        usleep(500 * 1000);
    }
    usleep(500 * 1000);

    // validate land mode
    if (UAVs[array_id]->state_mode != "AUTO.RTL")
    {
        return generateMsgString(id, RETURN_FAILURE);
    }

    return generateMsgString(id, RETURN_SUCCESS);
}

std::string land_uavs_cmd()
{
    std::vector<PX4RPCServerMsg> msgs = {};
    nlohmann::json msgs_json;

    // if not init, reject land
    if (!init_flag)
    {
        msgs.emplace_back(generateMsg(-1, NOT_INIT));
        msgs_json = msgs;
        return msgs_json.dump();
    }

    // create thread for land cmd
    for (const auto &pair : UAVIDMap)
    {
        UAVsCmdResult[pair.second] = false;
        std::thread uav_thread(&land_thread_func, pair.first);
        uav_thread.detach();
    }

    // wait for thread exit
    while (!detect_cmd_exec_done())
    {
        usleep(500 * 1000);
    }
    usleep(500 * 1000);

    // validate land mode
    for (const auto &pair : UAVIDMap)
    {
        if (UAVs[pair.second]->state_mode != "AUTO.LAND")
        {
            msgs.emplace_back(generateMsg(pair.first, LAND_FAILURE));
            continue;
        }
        msgs.emplace_back(generateMsg(pair.first, LAND_SUCCESS));
    }

    // transform data
    msgs_json = msgs;

    return msgs_json.dump();
}

std::string return_uavs_cmd()
{
    std::vector<PX4RPCServerMsg> msgs = {};
    nlohmann::json msgs_json;

    // if not init, reject return
    if (!init_flag)
    {
        msgs.emplace_back(generateMsg(-1, NOT_INIT));
        msgs_json = msgs;
        return msgs_json.dump();
    }

    // create thread for return cmd
    for (const auto &pair : UAVIDMap)
    {
        UAVsCmdResult[pair.second] = false;
        std::thread uav_thread(&return_thread_func, pair.first);
        uav_thread.detach();
    }
    // wait for thread exit
    while (!detect_cmd_exec_done())
    {
        usleep(500 * 1000);
    }
    usleep(500 * 1000);

    // validate return mode
    for (const auto &pair : UAVIDMap)
    {
        if (UAVs[pair.second]->state_mode != "AUTO.RTL")
        {
            msgs.emplace_back(generateMsg(pair.first, RETURN_FAILURE));
            continue;
        }
        msgs.emplace_back(generateMsg(pair.first, RETURN_SUCCESS));
    }

    // transform data
    msgs_json = msgs;

    return msgs_json.dump();
}

std::string takeoff_uavs_cmd(double height)
{
    std::vector<PX4RPCServerMsg> msgs = {};
    nlohmann::json msgs_json;

    // if not init, reject takeoff
    if (!init_flag)
    {
        msgs.emplace_back(generateMsg(-1, NOT_INIT));
        msgs_json = msgs;
        return msgs_json.dump();
    }

    // minimum height
    if (height < 0.5)
    {
        msgs.emplace_back(generateMsg(-1, HEIGHT_ERROR));
        msgs_json = msgs;
        return msgs_json.dump();
    }

    // create thread for takeoff cmd
    for (const auto &pair: UAVIDMap)
    {
        UAVsCmdResult[pair.second] = false;
        std::thread uav_thread(&takeoff_thread_func, pair.first, height);
        uav_thread.detach();
    }

    // wait for thread exit
    while (!detect_cmd_exec_done())
    {
        usleep(500 * 1000);
    }
    usleep(500 * 1000);
    takeoff_flag = true;

    // validate takeoff mode
    for (const auto &pair : UAVIDMap)
    {
        if (!UAVs[pair.second]->arm_state || UAVs[pair.second]->state_mode != "OFFBOARD")
        {
            msgs.emplace_back(generateMsg(pair.first, TAKEOFF_FAILURE));
            continue;
        }
        msgs.emplace_back(generateMsg(pair.first, TAKEOFF_SUCCESS));
    }

    // transform data
    msgs_json = msgs;

    return msgs_json.dump();
}

void land_thread_func(int id)
{
    // transform id to array id
    id = UAVIDMap[id];

    UAVs[id]->set_mode("AUTO.LAND");
    UAVsCmdResult[id] = true;
}

void return_thread_func(int id)
{
    // transform id to array id
    id = UAVIDMap[id];

    UAVs[id]->set_mode("AUTO.RTL");
    UAVsCmdResult[id] = true;
}

void takeoff_thread_func(int id, double height)
{
    // transform id to array id
    id = UAVIDMap[id];

    // 设置命令
    UAVsCmd[id].Mode = px4_cmd::Command::Takeoff;
    UAVsCmd[id].Move_frame = px4_cmd::Command::ENU;
    UAVsCmd[id].desire_cmd[0] = 0;
    UAVsCmd[id].desire_cmd[1] = 0;
    UAVsCmd[id].desire_cmd[2] = height;
    UAVs[id]->set_command(&UAVsCmd[id]);
    UAVs[id]->set_mode("OFFBOARD");
    UAVs[id]->set_mode("Arm");
    UAVsCmdResult[id] = true;
}

int get_uavs_count()
{
    return UAVs.size();
}

std::string get_uav_info(int id)
{
    // validate id
    if (validate_id(id) == ID_ERROR)
    {
        return "";
    }

    // transform id to array id
    int i = UAVIDMap[id];
    UAV::UAVInfo single_UAV;
    single_UAV.id = i;
    single_UAV.state = UAVs[i]->state;
    single_UAV.x = UAVs[i]->x;
    single_UAV.y = UAVs[i]->y;
    single_UAV.z = UAVs[i]->z;
    single_UAV.vx = UAVs[i]->vx;
    single_UAV.vy = UAVs[i]->vy;
    single_UAV.vz = UAVs[i]->vz;
    single_UAV.quaternion[0] = UAVs[i]->quaternion[0];
    single_UAV.quaternion[1] = UAVs[i]->quaternion[1];
    single_UAV.quaternion[2] = UAVs[i]->quaternion[2];
    single_UAV.quaternion[3] = UAVs[i]->quaternion[3];
    single_UAV.pitch = UAVs[i]->pitch;
    single_UAV.roll = UAVs[i]->roll;
    single_UAV.yaw = UAVs[i]->yaw;
    single_UAV.x_real = UAVs[i]->x_real;
    single_UAV.y_real = UAVs[i]->y_real;
    single_UAV.z_real = UAVs[i]->z_real;
    single_UAV.vx_real = UAVs[i]->vx_real;
    single_UAV.vy_real = UAVs[i]->vy_real;
    single_UAV.vz_real = UAVs[i]->vz_real;
    single_UAV.quaternion_real[0] = UAVs[i]->quaternion_real[0];
    single_UAV.quaternion_real[1] = UAVs[i]->quaternion_real[1];
    single_UAV.quaternion_real[2] = UAVs[i]->quaternion_real[2];
    single_UAV.quaternion_real[3] = UAVs[i]->quaternion_real[3];
    single_UAV.pitch_real = UAVs[i]->pitch_real;
    single_UAV.roll_real = UAVs[i]->roll_real;
    single_UAV.yaw_real = UAVs[i]->yaw_real;
    single_UAV.arm_state = UAVs[i]->arm_state;
    single_UAV.land_state = UAVs[i]->land_state;
    single_UAV.valid = true;
    nlohmann::json info_json = single_UAV;
    return info_json.dump();
}

std::string get_uavs_info()
{
    std::vector<UAV::UAVInfo> all_UAVs = {};
    UAV::UAVInfo single_UAV;
    for (const auto &pair : UAVIDMap)
    {
        int i = pair.second;
        single_UAV.id = pair.first;
        single_UAV.state = UAVs[i]->state;
        single_UAV.x = UAVs[i]->x;
        single_UAV.y = UAVs[i]->y;
        single_UAV.z = UAVs[i]->z;
        single_UAV.vx = UAVs[i]->vx;
        single_UAV.vy = UAVs[i]->vy;
        single_UAV.vz = UAVs[i]->vz;
        single_UAV.quaternion[0] = UAVs[i]->quaternion[0];
        single_UAV.quaternion[1] = UAVs[i]->quaternion[1];
        single_UAV.quaternion[2] = UAVs[i]->quaternion[2];
        single_UAV.quaternion[3] = UAVs[i]->quaternion[3];
        single_UAV.pitch = UAVs[i]->pitch;
        single_UAV.roll = UAVs[i]->roll;
        single_UAV.yaw = UAVs[i]->yaw;
        single_UAV.x_real = UAVs[i]->x_real;
        single_UAV.y_real = UAVs[i]->y_real;
        single_UAV.z_real = UAVs[i]->z_real;
        single_UAV.vx_real = UAVs[i]->vx_real;
        single_UAV.vy_real = UAVs[i]->vy_real;
        single_UAV.vz_real = UAVs[i]->vz_real;
        single_UAV.quaternion_real[0] = UAVs[i]->quaternion_real[0];
        single_UAV.quaternion_real[1] = UAVs[i]->quaternion_real[1];
        single_UAV.quaternion_real[2] = UAVs[i]->quaternion_real[2];
        single_UAV.quaternion_real[3] = UAVs[i]->quaternion_real[3];
        single_UAV.pitch_real = UAVs[i]->pitch_real;
        single_UAV.roll_real = UAVs[i]->roll_real;
        single_UAV.yaw_real = UAVs[i]->yaw_real;
        single_UAV.arm_state = UAVs[i]->arm_state;
        single_UAV.land_state = UAVs[i]->land_state;
        single_UAV.valid = true;
        all_UAVs.emplace_back(single_UAV);
    }

    nlohmann::json info_json = all_UAVs;
    return info_json.dump();
}

void exit_()
{
    stop();
    exit(0);
}

std::string stop()
{
    // clear vars
    init_flag = false;
    UAVIDMap.clear();
    for (int i = 0; i < UAVs.size(); i++)
    {
        UAVs[i]->thread_stop = true;
    }
    usleep(400 * 1000 * agents_count);
    UAVsCmd.clear();
    UAVsCmdResult.clear();
    UAVs.clear();
    agents_count = 0;

    // kill roslaunch proces
    int status;
    for (auto pid : ros_launch_pids)
    {
        kill(pid, SIGTERM);
        waitpid(pid, &status, 0);
    }
    ros_launch_pids.clear();
    return generateMsgString(-1, STOP_SUCCESS);
}

std::string initial(std::string init_json_string)
{
    // init return message
    std::vector<PX4RPCServerMsg> msgs = {};
    nlohmann::json msgs_json;
    std::vector<UAV::UAVInit> init_pose = {};

    // transform data
    try
    {
        nlohmann::json init_json = nlohmann::json::parse(init_json_string);
        init_pose = init_json.get<std::vector<UAV::UAVInit>>();
    }
    catch (const std::exception &e)
    {
        msgs.emplace_back(generateMsg(-1, JSON_INPUT_ERROR, e.what()));
        msgs_json = msgs;
        return msgs_json.dump();
    }

    // tmp launch file
    std::string output_file = "/tmp/px4_cmd_rpc.launch";

    // init new xml
    XMLDocument doc;
    doc.Parse("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<!-- This launch file is generated by PX4 Cmd generator -->");
    XMLElement *root = doc.NewElement("launch");
    doc.InsertEndChild(root);
    // env config
    XMLElement *env = doc.NewElement("env");
    env->SetAttribute("name", "ROSCONSOLE_CONFIG_FILE");
    env->SetAttribute("value", "$(find px4_cmd)/config/rosconsole.conf");
    root->InsertEndChild(env);
    // arg config
    // get gui state
    string gazebo_gui = "false";
    XMLElement *arg_1 = doc.NewElement("arg");
    arg_1->SetAttribute("name", "est");
    arg_1->SetAttribute("default", "ekf2");
    root->InsertEndChild(arg_1);
    XMLElement *arg_2 = doc.NewElement("arg");
    arg_2->SetAttribute("name", "world");
    arg_2->SetAttribute("default", "$(find mavlink_sitl_gazebo)/worlds/empty.world");
    root->InsertEndChild(arg_2);
    XMLElement *arg_3 = doc.NewElement("arg");
    arg_3->SetAttribute("name", "gui");
    arg_3->SetAttribute("default", gazebo_gui.c_str());
    root->InsertEndChild(arg_3);
    XMLElement *arg_4 = doc.NewElement("arg");
    arg_4->SetAttribute("name", "debug");
    arg_4->SetAttribute("default", "false");
    root->InsertEndChild(arg_4);
    XMLElement *arg_5 = doc.NewElement("arg");
    arg_5->SetAttribute("name", "verbose");
    arg_5->SetAttribute("default", "false");
    root->InsertEndChild(arg_5);
    XMLElement *arg_6 = doc.NewElement("arg");
    arg_6->SetAttribute("name", "paused");
    arg_6->SetAttribute("default", "false");
    root->InsertEndChild(arg_6);
    XMLElement *arg_7 = doc.NewElement("arg");
    arg_7->SetAttribute("name", "topic_type");
    arg_7->SetAttribute("default", "uav_{ID}");
    root->InsertEndChild(arg_7);
    XMLElement *arg_8 = doc.NewElement("arg");
    arg_8->SetAttribute("name", "interactive");
    arg_8->SetAttribute("default", "true");
    root->InsertEndChild(arg_8);
    XMLElement *arg_9 = doc.NewElement("arg");
    arg_9->SetAttribute("name", "sim_dir");
    arg_9->SetAttribute("default", "");

    root->InsertEndChild(arg_9);

    // gazebo simulation config
    XMLComment *gazebo_comment = doc.NewComment(" gazebo simulation ");
    root->InsertEndChild(gazebo_comment);
    XMLElement *gazebo_sim_config = doc.NewElement("include");
    gazebo_sim_config->SetAttribute("file", "$(find gazebo_ros)/launch/empty_world.launch");
    root->InsertEndChild(gazebo_sim_config);
    XMLElement *gazebo_arg_1 = doc.NewElement("arg");
    gazebo_arg_1->SetAttribute("name", "gui");
    gazebo_arg_1->SetAttribute("value", "$(arg gui)");
    gazebo_sim_config->InsertEndChild(gazebo_arg_1);
    XMLElement *gazebo_arg_2 = doc.NewElement("arg");
    gazebo_arg_2->SetAttribute("name", "world_name");
    gazebo_arg_2->SetAttribute("value", "$(arg world)");
    gazebo_sim_config->InsertEndChild(gazebo_arg_2);
    XMLElement *gazebo_arg_3 = doc.NewElement("arg");
    gazebo_arg_3->SetAttribute("name", "debug");
    gazebo_arg_3->SetAttribute("value", "$(arg debug)");
    gazebo_sim_config->InsertEndChild(gazebo_arg_3);
    XMLElement *gazebo_arg_4 = doc.NewElement("arg");
    gazebo_arg_4->SetAttribute("name", "verbose");
    gazebo_arg_4->SetAttribute("value", "$(arg verbose)");
    gazebo_sim_config->InsertEndChild(gazebo_arg_4);
    XMLElement *gazebo_arg_5 = doc.NewElement("arg");
    gazebo_arg_5->SetAttribute("name", "paused");
    gazebo_arg_5->SetAttribute("value", "$(arg paused)");
    gazebo_sim_config->InsertEndChild(gazebo_arg_5);

    // config for vehicles
    string vehicle;
    string sensor;
    string sensor_folder_name;
    string model;
    string topic_type;
    string topic_name;
    string sensor_model_path;
    int ID = 0;
    int px4_instance = 0;
    int local_port = 34580;
    // not must be 14540 ~ 14549, see
    int remote_port = 14540;
    int sitl_port = 24560;
    int tcp_port = 4560;
    XMLComment *agent_comment;
    XMLElement *agent;
    XMLElement *agent_arg_1;
    XMLElement *agent_arg_2;
    XMLElement *agent_arg_3;
    XMLElement *agent_arg_4;
    XMLElement *agent_arg_5;
    XMLElement *agent_arg_6;
    XMLElement *agent_arg_7;
    XMLElement *agent_arg_x;
    XMLElement *agent_arg_y;
    XMLElement *agent_arg_z;
    XMLElement *agent_arg_R;
    XMLElement *agent_arg_P;
    XMLElement *agent_arg_Y;
    XMLElement *agent_arg_mavid;
    XMLElement *agent_arg_udp_port;
    XMLElement *agent_arg_tcp_port;
    XMLElement *agent_arg_sdk_port;
    XMLElement *agent_arg_cam_udp_port;
    XMLElement *agent_arg_gst_udp_port;
    XMLElement *agent_arg_video_uri;
    XMLComment *agent_cmd_comment;
    XMLElement *agent_arg_cmd;
    XMLElement *agent_arg_cmd_param;
    XMLElement *agent_arg_vehicle_param;
    XMLElement *agent_arg_sensor_param;
    XMLElement *agent_arg_x_param;
    XMLElement *agent_arg_y_param;
    XMLElement *agent_arg_z_param;
    XMLElement *agent_arg_R_param;
    XMLElement *agent_arg_P_param;
    XMLElement *agent_arg_Y_param;
    XMLComment *px4_comment;
    XMLElement *px4_env_1;
    XMLElement *px4_env_2;
    XMLElement *px4_arg_1;
    XMLElement *px4_arg_2;
    XMLElement *px4_node;
    XMLComment *spawn_model_comment;
    XMLElement *spawn_model;
    XMLElement *mavros;
    XMLComment *mavros_comment;
    XMLElement *mavros_arg_1;
    XMLElement *mavros_arg_2;
    XMLElement *mavros_arg_3;
    XMLElement *mavros_arg_4;
    XMLElement *mavros_arg_5;
    for (unsigned int i = agents_count; i < init_pose.size(); i++)
    {
        // get vehicle & sensor data
        topic_type = "uav_{ID}";
        vehicle = "iris";
        model = vehicle;
        ID = init_pose[i].id;

        topic_name = "uav_" + to_string(ID);

        if (UAVIDMap.find(ID) != UAVIDMap.end())
        {
            msgs.emplace_back(generateMsg(ID, INIT_ALREADY));
            continue;
        }

        // avoid 14550 port
        if (ID == 10)
        {
            px4_instance += 1;
            local_port += 1;
            remote_port += 1;
            sitl_port += 1;
            tcp_port += 1;
        }

        // init arg
        agent_comment = doc.NewComment(("uav_" + to_string(i) + ": " + vehicle).c_str());
        root->InsertEndChild(agent_comment);
        agent = doc.NewElement("group");
        agent->SetAttribute("ns", topic_name.c_str());
        root->InsertEndChild(agent);
        agent_arg_1 = doc.NewElement("arg");
        agent_arg_1->SetAttribute("name", "vehicle");
        agent_arg_1->SetAttribute("value", vehicle.c_str());
        agent->InsertEndChild(agent_arg_1);
        agent_arg_2 = doc.NewElement("arg");
        agent_arg_2->SetAttribute("name", "sensor");
        agent_arg_2->SetAttribute("value", sensor.c_str());
        agent->InsertEndChild(agent_arg_2);
        agent_arg_3 = doc.NewElement("arg");
        agent_arg_3->SetAttribute("name", "ID");
        agent_arg_3->SetAttribute("value", (to_string(ID)).c_str());
        agent->InsertEndChild(agent_arg_3);
        agent_arg_4 = doc.NewElement("arg");
        agent_arg_4->SetAttribute("name", "fcu_url");
        agent_arg_4->SetAttribute("value", ("udp://:" + to_string(remote_port) + "@localhost:" + to_string(local_port)).c_str());
        agent->InsertEndChild(agent_arg_4);
        agent_arg_5 = doc.NewElement("arg");
        agent_arg_5->SetAttribute("name", "model");
        agent_arg_5->SetAttribute("value", model.c_str());
        agent->InsertEndChild(agent_arg_5);
        agent_arg_6 = doc.NewElement("arg");
        agent_arg_6->SetAttribute("name", "sensor_model_path");
        agent_arg_6->SetAttribute("value", sensor_model_path.c_str());
        agent->InsertEndChild(agent_arg_6);
        agent_arg_7 = doc.NewElement("arg");
        agent_arg_7->SetAttribute("name", "px4_instance");
        agent_arg_7->SetAttribute("value", (to_string(px4_instance)).c_str());
        agent->InsertEndChild(agent_arg_7);
        agent_arg_x = doc.NewElement("arg");
        agent_arg_x->SetAttribute("name", "x");
        agent_arg_x->SetAttribute("value", std::to_string(init_pose[i].init_x).c_str());
        agent->InsertEndChild(agent_arg_x);
        agent_arg_y = doc.NewElement("arg");
        agent_arg_y->SetAttribute("name", "y");
        agent_arg_y->SetAttribute("value", std::to_string(init_pose[i].init_y).c_str());
        agent->InsertEndChild(agent_arg_y);
        agent_arg_z = doc.NewElement("arg");
        agent_arg_z->SetAttribute("name", "z");
        agent_arg_z->SetAttribute("value", std::to_string(init_pose[i].init_z).c_str());
        agent->InsertEndChild(agent_arg_z);
        agent_arg_R = doc.NewElement("arg");
        agent_arg_R->SetAttribute("name", "R");
        agent_arg_R->SetAttribute("value", std::to_string(init_pose[i].init_roll).c_str());
        agent->InsertEndChild(agent_arg_R);
        agent_arg_P = doc.NewElement("arg");
        agent_arg_P->SetAttribute("name", "P");
        agent_arg_P->SetAttribute("value", std::to_string(init_pose[i].init_pitch).c_str());
        agent->InsertEndChild(agent_arg_P);
        agent_arg_Y = doc.NewElement("arg");
        agent_arg_Y->SetAttribute("name", "Y");
        agent_arg_Y->SetAttribute("value", std::to_string(init_pose[i].init_yaw).c_str());
        agent->InsertEndChild(agent_arg_Y);
        agent_arg_mavid = doc.NewElement("arg");
        agent_arg_mavid->SetAttribute("name", "mavlink_id");
        agent_arg_mavid->SetAttribute("value", "$(eval 1 + arg('px4_instance'))");
        agent->InsertEndChild(agent_arg_mavid);
        agent_arg_udp_port = doc.NewElement("arg");
        agent_arg_udp_port->SetAttribute("name", "mavlink_udp_port");
        agent_arg_udp_port->SetAttribute("value", to_string(sitl_port).c_str());
        agent->InsertEndChild(agent_arg_udp_port);
        agent_arg_tcp_port = doc.NewElement("arg");
        agent_arg_tcp_port->SetAttribute("name", "mavlink_tcp_port");
        agent_arg_tcp_port->SetAttribute("value", to_string(tcp_port).c_str());
        agent->InsertEndChild(agent_arg_tcp_port);
        agent_arg_sdk_port = doc.NewElement("arg");
        agent_arg_sdk_port->SetAttribute("name", "sdk_udp_port");
        agent_arg_sdk_port->SetAttribute("value", to_string(remote_port).c_str());
        agent->InsertEndChild(agent_arg_sdk_port);
        agent_arg_cam_udp_port = doc.NewElement("arg");
        agent_arg_cam_udp_port->SetAttribute("name", "mavlink_cam_udp_port");
        agent_arg_cam_udp_port->SetAttribute("value", "14530");
        agent->InsertEndChild(agent_arg_cam_udp_port);
        agent_arg_gst_udp_port = doc.NewElement("arg");
        agent_arg_gst_udp_port->SetAttribute("name", "gst_udp_port");
        agent_arg_gst_udp_port->SetAttribute("value", "$(eval 5600 + arg('px4_instance'))");
        agent->InsertEndChild(agent_arg_gst_udp_port);
        agent_arg_video_uri = doc.NewElement("arg");
        agent_arg_video_uri->SetAttribute("name", "video_uri");
        agent_arg_video_uri->SetAttribute("value", "$(eval 5600 + arg('px4_instance'))");
        agent->InsertEndChild(agent_arg_video_uri);
        agent_cmd_comment = doc.NewComment(" generate sdf vehicle model ");
        agent->InsertEndChild(agent_cmd_comment);
        agent_arg_cmd = doc.NewElement("arg");
        agent_arg_cmd->SetAttribute("name", "cmd");
        agent_arg_cmd->SetAttribute("value", "$(find px4_cmd)/scripts/model_gen.py --stdout --sensor_model_path=$(arg sensor_model_path) --mavlink_id=$(arg mavlink_id) --mavlink_udp_port=$(arg mavlink_udp_port) --sdk_udp_port=$(arg sdk_udp_port) --mavlink_tcp_port=$(arg mavlink_tcp_port) --gst_udp_port=$(arg gst_udp_port) --video_uri=$(arg video_uri) --mavlink_cam_udp_port=$(arg mavlink_cam_udp_port) $(find px4_cmd)/models/$(arg model)/$(arg model).sdf.jinja $(find px4_cmd)");
        agent->InsertEndChild(agent_arg_cmd);
        agent_arg_cmd_param = doc.NewElement("param");
        agent_arg_cmd_param->SetAttribute("command", "$(arg cmd)");
        agent_arg_cmd_param->SetAttribute("name", "sdf_$(arg vehicle)$(arg px4_instance)");
        agent->InsertEndChild(agent_arg_cmd_param);
        // vehicle and sensor
        agent_arg_vehicle_param = doc.NewElement("param");
        agent_arg_vehicle_param->SetAttribute("name", "vehicle");
        agent_arg_vehicle_param->SetAttribute("value", vehicle.c_str());
        agent->InsertEndChild(agent_arg_vehicle_param);
        agent_arg_sensor_param = doc.NewElement("param");
        agent_arg_sensor_param->SetAttribute("name", "sensor");
        agent_arg_sensor_param->SetAttribute("value", sensor.c_str());
        agent->InsertEndChild(agent_arg_sensor_param);
        // parameter for init
        agent_arg_x_param = doc.NewElement("param");
        agent_arg_x_param->SetAttribute("name", "init_x");
        agent_arg_x_param->SetAttribute("value", std::to_string(init_pose[i].init_x).c_str());
        agent->InsertEndChild(agent_arg_x_param);
        agent_arg_y_param = doc.NewElement("param");
        agent_arg_y_param->SetAttribute("name", "init_y");
        agent_arg_y_param->SetAttribute("value", std::to_string(init_pose[i].init_y).c_str());
        agent->InsertEndChild(agent_arg_y_param);
        agent_arg_z_param = doc.NewElement("param");
        agent_arg_z_param->SetAttribute("name", "init_z");
        agent_arg_z_param->SetAttribute("value", std::to_string(init_pose[i].init_z).c_str());
        agent->InsertEndChild(agent_arg_z_param);
        agent_arg_R_param = doc.NewElement("param");
        agent_arg_R_param->SetAttribute("name", "init_R");
        agent_arg_R_param->SetAttribute("value", std::to_string(init_pose[i].init_roll).c_str());
        agent->InsertEndChild(agent_arg_R_param);
        agent_arg_P_param = doc.NewElement("param");
        agent_arg_P_param->SetAttribute("name", "init_P");
        agent_arg_P_param->SetAttribute("value", std::to_string(init_pose[i].init_pitch).c_str());
        agent->InsertEndChild(agent_arg_P_param);
        agent_arg_Y_param = doc.NewElement("param");
        agent_arg_Y_param->SetAttribute("name", "init_Y");
        agent_arg_Y_param->SetAttribute("value", std::to_string(init_pose[i].init_yaw).c_str());
        agent->InsertEndChild(agent_arg_Y_param);
        // PX4 config
        px4_comment = doc.NewComment(" px4 ");
        agent->InsertEndChild(px4_comment);
        px4_env_1 = doc.NewElement("env");
        px4_env_1->SetAttribute("name", "PX4_SIM_MODEL");
        px4_env_1->SetAttribute("value", "$(arg vehicle)");
        agent->InsertEndChild(px4_env_1);
        px4_env_2 = doc.NewElement("env");
        px4_env_2->SetAttribute("name", "PX4_ESTIMATOR");
        px4_env_2->SetAttribute("value", "$(arg est)");
        agent->InsertEndChild(px4_env_2);
        px4_arg_1 = doc.NewElement("arg");
        px4_arg_1->SetAttribute("unless", "$(arg interactive)");
        px4_arg_1->SetAttribute("name", "px4_command_arg1");
        px4_arg_1->SetAttribute("value", "");
        agent->InsertEndChild(px4_arg_1);
        px4_arg_2 = doc.NewElement("arg");
        px4_arg_2->SetAttribute("if", "$(arg interactive)");
        px4_arg_2->SetAttribute("name", "px4_command_arg1");
        px4_arg_2->SetAttribute("value", "-d");
        agent->InsertEndChild(px4_arg_2);
        px4_node = doc.NewElement("node");
        px4_node->SetAttribute("name", "sitl_$(arg px4_instance)");
        px4_node->SetAttribute("pkg", "px4");
        px4_node->SetAttribute("type", "px4");
        px4_node->SetAttribute("output", "screen");
        px4_node->SetAttribute("args", ("$(find px4_cmd)/config/px4/etc -s etc/init.d-posix/rcS -i $(arg px4_instance) -w " + topic_name + " $(arg px4_command_arg1)").c_str());
        agent->InsertEndChild(px4_node);
        // spawn model
        spawn_model_comment = doc.NewComment(" spawn model ");
        agent->InsertEndChild(spawn_model_comment);
        spawn_model = doc.NewElement("node");
        spawn_model->SetAttribute("name", "$(anon vehicle_spawn)");
        spawn_model->SetAttribute("pkg", "gazebo_ros");
        spawn_model->SetAttribute("type", "spawn_model");
        spawn_model->SetAttribute("output", "log");
        spawn_model->SetAttribute("args", ("-sdf -param sdf_$(arg vehicle)$(arg px4_instance) -model " + topic_name + " -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)").c_str());
        agent->InsertEndChild(spawn_model);
        // mavros
        mavros_comment = doc.NewComment(" mavros ");
        agent->InsertEndChild(mavros_comment);
        mavros = doc.NewElement("include");
        mavros->SetAttribute("file", "$(find px4_cmd)/launch/mavros.launch");
        agent->InsertEndChild(mavros);
        mavros_arg_1 = doc.NewElement("arg");
        mavros_arg_1->SetAttribute("name", "fcu_url");
        mavros_arg_1->SetAttribute("value", "$(arg fcu_url)");
        mavros->InsertEndChild(mavros_arg_1);
        mavros_arg_2 = doc.NewElement("arg");
        mavros_arg_2->SetAttribute("name", "gcs_url");
        mavros_arg_2->SetAttribute("value", "");
        mavros->InsertEndChild(mavros_arg_2);
        mavros_arg_3 = doc.NewElement("arg");
        mavros_arg_3->SetAttribute("name", "tgt_system");
        mavros_arg_3->SetAttribute("value", "$(eval 1 + arg('px4_instance'))");
        mavros->InsertEndChild(mavros_arg_3);
        mavros_arg_4 = doc.NewElement("arg");
        mavros_arg_4->SetAttribute("name", "tgt_component");
        mavros_arg_4->SetAttribute("value", "1");
        mavros->InsertEndChild(mavros_arg_4);
        mavros_arg_5 = doc.NewElement("arg");
        mavros_arg_5->SetAttribute("name", "log_output");
        mavros_arg_5->SetAttribute("value", "log");
        mavros->InsertEndChild(mavros_arg_5);
    
        // update UAV ID
        px4_instance += 1;
        local_port += 1;
        remote_port += 1;
        sitl_port += 1;
        tcp_port += 1;
        UAVIDMap[ID] = i;
        agents_count++;
        msgs.emplace_back(generateMsg(ID, INIT_SUCCESS));
    }
    // generate and roslaunch
    auto result = doc.SaveFile(output_file.c_str());
    if (result == 0)
    {
        system("killall px4 | >> /dev/null");
        system("killall gzserver | >> /dev/null");
        new_process("roslaunch", "roslaunch", (output_file).c_str());
        sleep(7 + 2 * UAVIDMap.size());
        for (const auto &pair : UAVIDMap)
        {
            topic_name = "uav_" + to_string(pair.first);
            UAVs.emplace_back(std::unique_ptr<UAV::UAVBridge>(new UAV::UAVBridge(topic_name)));
            UAVs[pair.second]->start();
            UAVsCmd.emplace_back(px4_cmd::Command());
            UAVsCmdResult.emplace_back(true);
        }
        init_flag = true;
    }

    // return msgs
    msgs_json = msgs;
    return msgs_json.dump();
}

int new_process(const char *path, const char *cmd, const char *arg)
{
    pid_t pid = fork();
    if (pid == -1)
    { // 创建子进程失败
        return CREATE_NEW_PROCESS_FAILURE;
    }
    else if (pid == 0)
    { // 子进程
        execlp(path, cmd, arg, NULL);
    }
    else
    {
        ros_launch_pids.emplace_back(pid);
    }
    return CREATE_NEW_PROCESS_SUCCESS;
}

CustomCommand UAVCommand2CustomCommand(const UAV::UAVCommand &cmd)
{
    // init varible
    CustomCommand cmd_;

    // mode transform
    switch (cmd.mode)
    {
        case UAV::CommandMode::TargetLocal:
            cmd_.mode = TargetLocal;
            break;

        case UAV::CommandMode::TargetGlobal:
            cmd_.mode = TargetGlobal;
            break;

        case UAV::CommandMode::TargetAttitude:
            cmd_.mode = TargetAttitude;
            break;
    }

    // fw_mode transform
    switch (cmd.fw_mode)
    {
        case UAV::FixWingPositionMode::GlidingMode:
            cmd_.fw_mode = GlidingMode;
            break;

        case UAV::FixWingPositionMode::TakeoffMode:
            cmd_.fw_mode = TakeoffMode;
            break;

        case UAV::FixWingPositionMode::LandMode:
            cmd_.fw_mode = LandMode;
            break;

        case UAV::FixWingPositionMode::LoiterMode:
            cmd_.fw_mode = LoiterMode;
            break;

        case UAV::FixWingPositionMode::IdleMode:
            cmd_.fw_mode = IdleMode;
            break;
    }

    // frame_id transform
    switch (cmd.frame_id)
    {
        case UAV::FrameId::ENU:
            cmd_.frame_id = px4_cmd::Command::ENU;
            break;

        case UAV::FrameId::BODY:
            cmd_.frame_id = px4_cmd::Command::BODY;
            break;

        case UAV::FrameId::GLOBAL:
            cmd_.frame_id = px4_cmd::Command::GLOBAL;
            break;
    }

    // data transform
    cmd_.position[0] = cmd.x;
    cmd_.position[1] = cmd.y;
    cmd_.position[2] = cmd.z;
    cmd_.velocity[0] = cmd.vx;
    cmd_.velocity[1] = cmd.vy;
    cmd_.velocity[2] = cmd.vz;
    cmd_.accelerate[0] = cmd.ax;
    cmd_.accelerate[1] = cmd.ay;
    cmd_.accelerate[2] = cmd.az;
    cmd_.force_flag = cmd.force_flag;
    cmd_.yaw = cmd.yaw;
    cmd_.yaw_rate = cmd.yaw_rate;
    cmd_.attitude[0] = cmd.quaternion[0];
    cmd_.attitude[1] = cmd.quaternion[1];
    cmd_.attitude[2] = cmd.quaternion[2];
    cmd_.attitude[3] = cmd.quaternion[3];
    cmd_.attitude_rate[0] = cmd.attitude_rate[1];
    cmd_.attitude_rate[1] = cmd.attitude_rate[1];
    cmd_.attitude_rate[2] = cmd.attitude_rate[2];
    cmd_.thrust = cmd.thrust;

    return cmd_;
}

bool detect_cmd_exec_done()
{
    for (size_t i = 0; i < UAVsCmdResult.size(); i++)
    {
        if (!UAVsCmdResult[i])
        {
            return false;
        }
    }
    return true;
}

int validate_id(int id)
{
    if (UAVIDMap.find(id) == UAVIDMap.end())
    {
        return ID_ERROR;
    }
    return 0;
}

std::string getIP()
{
    struct ifaddrs *ifAddrStruct = NULL;
    void *tmpAddrPtr = NULL;

    getifaddrs(&ifAddrStruct);

    while (ifAddrStruct != NULL)
    {
        if (ifAddrStruct->ifa_addr->sa_family == AF_INET)
        { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr = &((struct sockaddr_in *)ifAddrStruct->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            std::string IP(addressBuffer);
            if (IP.find("127.0.0.1") == std::string::npos && IP.find("10.255.255.254") == std::string::npos)
            {
                return IP;
            }
        }
        /*
        else if (ifAddrStruct->ifa_addr->sa_family == AF_INET6)
        {   // check it is IP6
            // is a valid IP6 Address
            continue;
            tmpAddrPtr = &((struct sockaddr_in *)ifAddrStruct->ifa_addr)->sin_addr;
            char addressBuffer[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
            printf("%s IP Address %s/n", ifAddrStruct->ifa_name, addressBuffer);
        }
        */
        ifAddrStruct = ifAddrStruct->ifa_next;
    }
    return "";
}