#include "common_inc.h"
#include "usart.h"
#include "usbd_cdc_if.h"

extern DummyRobot dummy;

float pi = 3.14159265358979323846f;
extern bool start_counter;
extern bool current_start_counter;
extern bool start_shijiao;
float pos_all[6][1500];

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
static float deg2rad(float deg)
{
    return deg*pi/180.0f;
}
int id[6];
int current_id[6];
float temp_pos[6];
extern float vel_rad[6];
extern float acc_rad[6];
extern float pos_rad[6];

float temp_p[10][6];
int point_num = 0;
char uart1_rx_buffer[128];
char command[64];
bool record_flag = false;
bool play_flag = false;
extern bool play_flag_first;
void OnUart6AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    int ret = 0;
    // sprintf(uart6_rx_buffer, "%s\n", _cmd);
    // HAL_UART_Transmit_DMA(&huart6, (uint8_t *) uart6_rx_buffer, 16);
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    switch(_cmd[0])
    {
    case '1':
        dummy.SetEnable(true);
        break;
    case '0':
        dummy.SetEnable(false);
        break;
    case 'r':
        if(_cmd[1] == 'e')
        {
            record_flag = true;
            // sprintf(uart1_rx_buffer, "Starting to record!\n");
            // HAL_UART_Transmit(&huart6, (uint8_t *) uart1_rx_buffer, 30, HAL_MAX_DELAY);
        }
        break;
    case 'P':
        if(_cmd[1] == 'L')
        {
            current_start_counter = false;
            play_flag = true;
            play_flag_first = true;
            // sprintf(uart1_rx_buffer, "Starting to play!\n");
            // HAL_UART_Transmit(&huart6, (uint8_t *) uart1_rx_buffer, 30, HAL_MAX_DELAY);
        }
        break;

    case 'R':
        dummy.Resting();
        break;
    case 'H':
        dummy.Homing();
        break;
    case 'S':
        dummy.Starting();
        break;
    case 'i':
        dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE);
        current_start_counter = false;
        break;
    case 'o':
        if(_cmd[1] == 'k')
        {
            dummy.SetCommandMode(DummyRobot::COMMAND_CONTINUES_TRAJECTORY);
            uint32_t freeSize;
            for(int i=0;i<point_num;i++)
            {
                sprintf((char*) command, ">%.2f,%.2f", temp_p[i][0],temp_p[i][1],temp_p[i][2],temp_p[i][3],temp_p[i][4],temp_p[i][5]);
                dummy.commandHandler.Push(command);

            }
            sprintf((char*) command, ">0,-165,90,0,0,0");
            freeSize = dummy.commandHandler.Push(command);
            point_num = 0;
            sprintf(command, "%d\n", freeSize);
            HAL_UART_Transmit(&huart6, (uint8_t *) command, 2, HAL_MAX_DELAY);
            start_shijiao = true;
        }
        break;
    case 'c':
        sscanf((char*) _cmd, "c %d %d %d %d %d %d", &current_id[0],&current_id[1],&current_id[2],&current_id[3],&current_id[4],&current_id[5]);
        current_start_counter = true;
        dummy.SetCommandMode(DummyRobot::DYNAMIC_CURRENT);
        start_shijiao = false;
        break;
    case 'p':
        sscanf((char*) _cmd, "p %d %d %d %d %d %d", &id[0],&id[1],&id[2],&id[3],&id[4],&id[5]);

        start_counter = true;
        dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL);
        break;
    case 'a':
        if(_cmd[1] == 'p')
        {
            dummy.motorJ[ALL]->ApplyPositionAsHome();
        }
        break;
    case '>':
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        break;
    }
/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}
float yuzhi1 = 1.0f;
float yuzhi2 = 0.6f;
float yuzhi3 = 0.5f;
float yuzhi4 = 0.5f;
float yuzhi5 = 1.0f;
float yuzhi6 = 1.0f;



int num_point = 0;
void OnTimer11Callback()
{
    if(current_start_counter)
    {
        if(record_flag)
        {
            for(int i=0;i<6;i++)
            {
                pos_all[i][num_point] = dummy.currentJoints.a[i];
            }
            num_point++;
            if(num_point>=1500)
            {
                num_point = 0;
                record_flag = false;
                current_start_counter = false;
                // sprintf(uart1_rx_buffer, "Recording finished!\n");
                // HAL_UART_Transmit(&huart6, (uint8_t *) uart1_rx_buffer, 24, HAL_MAX_DELAY);
            }

        }

        float p_filter[6];
        float v_filter[6];
        float a_filter[6] = {0,0,0,0,0,0};

        for(int i=0;i<6;i++)
        {
            if(i==0)
            {
                if(dummy.velocity.a[i]>0) v_filter[i] = dummy.velocity.a[i]> yuzhi1 ? deg2rad(dummy.velocity.a[i]):0.0f;
                else v_filter[i] = dummy.velocity.a[i]<-yuzhi1? deg2rad(dummy.velocity.a[i]):0.0f;
                // if (dummy.velocity.a[i]> 15.0f ) v_filter[i] = deg2rad(15.0f);
                // else if (dummy.velocity.a[i]<-15.0f ) v_filter[i] =deg2rad(-15.0f);

            }else if(i==1)
            {
                if(dummy.velocity.a[i]>0) v_filter[i] = dummy.velocity.a[i]> yuzhi2 ? deg2rad(dummy.velocity.a[i]):0.0f;
                else v_filter[i] = dummy.velocity.a[i]<-yuzhi2? deg2rad(dummy.velocity.a[i]):0.0f;
                // if (dummy.velocity.a[i]> 15.0f ) v_filter[i] = deg2rad(15.0f);
                // else if (dummy.velocity.a[i]<-15.0f ) v_filter[i] =deg2rad(-15.0f);
            }
            else if(i==2)
            {
                if(dummy.velocity.a[i]>0) v_filter[i] = dummy.velocity.a[i]> yuzhi3 ? deg2rad(dummy.velocity.a[i]):0.0f;
                else v_filter[i] = dummy.velocity.a[i]<-yuzhi3? deg2rad(dummy.velocity.a[i]):0.0f;
                // if (dummy.velocity.a[i]> 15.0f ) v_filter[i] = deg2rad(15.0f);
                // else if (dummy.velocity.a[i]<-15.0f ) v_filter[i] =deg2rad(-15.0f);
            }
            else if(i==3)
            {
                if(dummy.velocity.a[i]>0) v_filter[i] = dummy.velocity.a[i]> yuzhi4 ? deg2rad(dummy.velocity.a[i]):0.0f;
                else v_filter[i] = dummy.velocity.a[i]<-yuzhi4? deg2rad(dummy.velocity.a[i]):0.0f;
            }
            else if(i==4)
            {
                if(dummy.velocity.a[i]>0) v_filter[i] = dummy.velocity.a[i]> yuzhi5 ? deg2rad(dummy.velocity.a[i]):0.0f;
                else v_filter[i] = dummy.velocity.a[i]<-yuzhi5? deg2rad(dummy.velocity.a[i]):0.0f;
            }
            else
            {
                if(dummy.velocity.a[i]>0) v_filter[i] = dummy.velocity.a[i]> yuzhi6 ? deg2rad(dummy.velocity.a[i]):0.0f;
                else v_filter[i] = dummy.velocity.a[i] < -yuzhi6? deg2rad(dummy.velocity.a[i]):0.0f;
            }
            if (dummy.velocity.a[i]> 25.0f ) v_filter[i] = deg2rad(25.0f);
            else if (dummy.velocity.a[i]<-25.0f ) v_filter[i] =deg2rad(-25.0f);

            // if(dummy.acceleration.a[i]>0) a_filter[i] = dummy.acceleration.a[i]> 5.0f ? deg2rad(dummy.acceleration.a[i]):0.0f;
            // else a_filter[i] = dummy.acceleration.a[i]<-5.0f? deg2rad(dummy.acceleration.a[i]):0.0f;
            p_filter[i] =deg2rad(dummy.currentJoints.a[i]);
        }
        dummy.DynamicCalculation_updata(p_filter,v_filter,a_filter,dummy.current_required.a,&dummy.dynamicCalculationTime);
    }
}
// void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
// {
//     /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
//
// /*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
// }