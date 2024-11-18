#include "common_inc.h"
#include "usart.h"
#include "usbd_cdc_if.h"

extern DummyRobot dummy;


void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    // printf("%s\n", _cmd);
    // CDC_Transmit_FS((uint8_t*)_cmd, _len);
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    // if (_cmd[0] == '!' || !dummy.IsEnabled())
    // {
    //     std::string s(_cmd);
    //     if (s.find("STOP") != std::string::npos)
    //     {
    //         dummy.commandHandler.EmergencyStop();
    //         Respond(_responseChannel, "Stopped ok");
    //     } else if (s.find("START") != std::string::npos)
    //     {
    //         dummy.SetEnable(true);
    //         Respond(_responseChannel, "Started ok");
    //     } else if (s.find("DISABLE") != std::string::npos)
    //     {
    //         dummy.SetEnable(false);
    //         Respond(_responseChannel, "Disabled ok");
    //     }
    // } else if (_cmd[0] == '#')
    // {
    //     std::string s(_cmd);
    //     if (s.find("GETJPOS") != std::string::npos)
    //     {
    //         Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
    //                 dummy.currentJoints.a[0], dummy.currentJoints.a[1],
    //                 dummy.currentJoints.a[2], dummy.currentJoints.a[3],
    //                 dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
    //     } else if (s.find("GETLPOS") != std::string::npos)
    //     {
    //         dummy.UpdateJointPose6D();
    //         Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
    //                 dummy.currentPose6D.X, dummy.currentPose6D.Y,
    //                 dummy.currentPose6D.Z, dummy.currentPose6D.A,
    //                 dummy.currentPose6D.B, dummy.currentPose6D.C);
    //     } else if (s.find("CMDMODE") != std::string::npos)
    //     {
    //         uint32_t mode;
    //         sscanf(_cmd, "#CMDMODE %lu", &mode);
    //         dummy.SetCommandMode(mode);
    //         Respond(_responseChannel, "Set command mode to [%lu]", mode);
    //     } else
    //         Respond(_responseChannel, "ok");
    // } else if (_cmd[0] == '>' || _cmd[0] == '@')
    // {
    //     uint32_t freeSize = dummy.commandHandler.Push(_cmd);
    //     Respond(_responseChannel, "%d", freeSize);
    // }

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart6AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    char uart6_rx_buffer[128];
    sprintf(uart6_rx_buffer, "%s\n", _cmd);
    // HAL_UART_Transmit(&huart6, (uint8_t *) uart6_rx_buffer, _len+1, HAL_MAX_DELAY);
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '1')
    {
        dummy.SetEnable(true);

    }else if (_cmd[0] == '0')
    {
        dummy.SetEnable(false);
    }
    if(_cmd[0] == 'j')
    {

    }
    // if (_cmd[0] == '!' || !dummy.IsEnabled())
    // {
    //     std::string s(_cmd);
    //     if (s.find("STOP") != std::string::npos)
    //     {
    //         dummy.commandHandler.EmergencyStop();
    //         Respond(_responseChannel, "Stopped ok");
    //     } else if (s.find("START") != std::string::npos)
    //     {
    //         dummy.SetEnable(true);
    //         Respond(_responseChannel, "Started ok");
    //     } else if (s.find("DISABLE") != std::string::npos)
    //     {
    //         dummy.SetEnable(false);
    //         Respond(_responseChannel, "Disabled ok");
    //     }
    // } else if (_cmd[0] == '#')
    // {
    //     std::string s(_cmd);
    //     if (s.find("GETJPOS") != std::string::npos)
    //     {
    //         Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
    //                 dummy.currentJoints.a[0], dummy.currentJoints.a[1],
    //                 dummy.currentJoints.a[2], dummy.currentJoints.a[3],
    //                 dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
    //     } else if (s.find("GETLPOS") != std::string::npos)
    //     {
    //         dummy.UpdateJointPose6D();
    //         Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
    //                 dummy.currentPose6D.X, dummy.currentPose6D.Y,
    //                 dummy.currentPose6D.Z, dummy.currentPose6D.A,
    //                 dummy.currentPose6D.B, dummy.currentPose6D.C);
    //     } else if (s.find("CMDMODE") != std::string::npos)
    //     {
    //         uint32_t mode;
    //         sscanf(_cmd, "#CMDMODE %lu", &mode);
    //         dummy.SetCommandMode(mode);
    //         Respond(_responseChannel, "Set command mode to [%lu]", mode);
    //     } else
    //         Respond(_responseChannel, "ok");
    // } else if (_cmd[0] == '>' || _cmd[0] == '@')
    // {
    //     uint32_t freeSize = dummy.commandHandler.Push(_cmd);
    //     Respond(_responseChannel, "%d", freeSize);
    // }
/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


// void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
// {
//     /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
//
// /*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
// }