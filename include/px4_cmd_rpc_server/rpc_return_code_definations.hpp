// Copyright (c) 2024 易鹏 中山大学航空航天学院
// Copyright (c) 2024 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef RPC_RETURN_CODE_DEFINATIONS_HPP
#define RPC_RETURN_CODE_DEFINATIONS_HPP
#include <string>
#include "NlohmannJSON.hpp"

enum RPC_RETURN_CODE
{
    INIT_SUCCESS,
    INIT_FAILURE,
    INIT_ALREADY,
    CREATE_NEW_PROCESS_SUCCESS,
    CREATE_NEW_PROCESS_FAILURE,
    STOP_SUCCESS,
    TAKEOFF_SUCCESS,
    TAKEOFF_FAILURE,
    LAND_SUCCESS,
    LAND_FAILURE,
    RETURN_SUCCESS,
    RETURN_FAILURE,
    NOT_TAKEOFF,
    NOT_INIT,
    ID_ERROR,
    HEIGHT_ERROR,
    HEIGHT_NAN_ERROR,
    COMMAND_FORMAT_ERROR,
    SET_COMMAND_SUCCESS,
    JSON_INPUT_ERROR
};

struct PX4RPCServerMsg
{
    int id;
    int return_code;
    std::string basic_message;
    std::string detail_message;

    std::string to_string()
    {
        std::string output = "";
        if (id == -1)
        {
            output += "[All UAVs] " + basic_message;
            if (detail_message.size() > 0)
            {
                output += (" (Detail: " + detail_message + ".)");
            }
        }
        else
        {
            output += "[UAV " + std::to_string(id) + "] " + basic_message;
            if (detail_message.size() > 0)
            {
                output += (" (Detail: " + detail_message + ".)");
            }
        }
        return output;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PX4RPCServerMsg, id, return_code, basic_message, detail_message);
};

std::string code2msg(const int code)
{
    std::string msg = "";
    switch (code)
    {
        case RPC_RETURN_CODE::INIT_SUCCESS:
            msg = "Init Success.";
            break;

        case RPC_RETURN_CODE::INIT_FAILURE:
            msg = "Init Failure.";
            break;

        case RPC_RETURN_CODE::INIT_ALREADY:
            msg = "Already Init.";
            break;

        case RPC_RETURN_CODE::STOP_SUCCESS:
            msg = "Stop Success.";
            break;
        
        case RPC_RETURN_CODE::TAKEOFF_SUCCESS:
            msg = "TakeOff Success.";
            break;

        case RPC_RETURN_CODE::TAKEOFF_FAILURE:
            msg = "TakeOff Failure.";
            break;

        case RPC_RETURN_CODE::LAND_SUCCESS:
            msg = "Set LAND Mode Success.";
            break;

        case RPC_RETURN_CODE::LAND_FAILURE:
            msg = "Set LAND Mode Failure.";
            break;

        case RPC_RETURN_CODE::RETURN_SUCCESS:
            msg = "Set RETURN Mode Success.";
            break;

        case RPC_RETURN_CODE::RETURN_FAILURE:
            msg = "Set RETURN Mode Failure.";
            break;

        case RPC_RETURN_CODE::NOT_TAKEOFF:
            msg = "Not TakeOff.";
            break;

        case RPC_RETURN_CODE::NOT_INIT:
            msg = "Not Init.";
            break;

        case RPC_RETURN_CODE::HEIGHT_ERROR:
            msg = "Minimun Height is 0.5 m.";
            break;

        case RPC_RETURN_CODE::HEIGHT_NAN_ERROR:
            msg = "TakeOff Command Need Desire Height.";
            break;

        case RPC_RETURN_CODE::JSON_INPUT_ERROR:
            msg = "Input Json Format Error.";
            break;

        case RPC_RETURN_CODE::SET_COMMAND_SUCCESS:
            msg = "Set Task Command Success.";
            break;

        case RPC_RETURN_CODE::ID_ERROR:
            msg = "ID Error (Format Error or ID Non-Exist)";
            break;
    }
    return msg;
};

PX4RPCServerMsg generateMsg(int id, int code, std::string detail_msg = "")
{
    PX4RPCServerMsg msg;
    msg.id = id;
    msg.return_code = code;
    msg.basic_message = code2msg(code);
    msg.detail_message = detail_msg;
    return msg;
}

std::string generateMsgString(int id, int code, std::string detail_msg = "")
{
    nlohmann::json msg_json = generateMsg(id, code, detail_msg);
    return msg_json.dump();
}

#endif