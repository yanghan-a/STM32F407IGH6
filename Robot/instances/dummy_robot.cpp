#include "communication.hpp"
#include "dummy_robot.h"

#include "time_utils.h"
#include "usart.h"

extern DummyRobot dummy;
inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{//判断6个关节的绝对值最大值，并返回最大值和对应的关节序号
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (abs(_joints.a[i]) > max)
        {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, true, 50, -170, 170);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 50, -166, 1);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, false, 50, -56, 91);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true, 50, -180, 180);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, false, 30, -120, 120);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, false, 50, -720, 720);
    // hand = new DummyHand(_hcan, 7);

    dof6Solver = new DOF6Kinematic(0.133f, 0.035f, 0.146f, 0.117f, 0.052f, 0.0855f);
    dof6Dynamic = new DOF6Dynamic(0.133f, 0.035f, 0.146f, 0.117f, 0.052f, 0.0855f,9.8);
    planner_traj.Init();
}


DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
}


void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);//默认模式为interraptable
    SetJointSpeed(DEFAULT_JOINT_SPEED);//默认为30r/s
}


void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
}

void DummyRobot::DynamicCurrentRequired(DOF6Kinematic::Joint6D_t _currents, int i)
{
    motorJ[i]->DynamicCurrentOutput(_currents.a[i - 1]);
}

void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints, int i)
{//参数为目标位置，另一个参数为速度限制
    if(dummy.commandMode == COMMAND_TARGET_POINT_INTERRUPTABLE)
    {
        // for (int j = 1; j <= 6; j++)
        // {
        motorJ[i]->SetAngleWithVelocityLimit(_joints.a[i - 1] - initPose.a[i - 1],
                                             dynamicJointSpeeds.a[i - 1]);
        // }
    }else if(dummy.commandMode == COMMAND_TARGET_POINT_SEQUENTIAL)
    {
        // for (int j = 1; j <= 6; j++)
        // {
        motorJ[i]->SetAngle(_joints.a[i - 1] - initPose.a[i - 1]);
        // }
    }else if(dummy.commandMode == COMMAND_CONTINUES_TRAJECTORY)
    {
        // for (int j = 1; j <= 6; j++)
        // {
        motorJ[i]->SetAngleWithVelocityLimit(_joints.a[i - 1] - initPose.a[i - 1],
                                             dynamicJointSpeeds.a[i - 1]);
        // motorJ[i]->SetAngleWithVelocityAndAcceleration(_joints.a[i - 1] - initPose.a[i - 1],
        //                                                dynamicJointSpeeds_Traj.a[i - 1],
        //                                                dynamicJointAcceleration_Traj.a[i - 1]);
        // }
    }

}

bool DummyRobot::MoveJ_Traj(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,float _v1, float _v2, float _v3, float _v4, float _v5, float _v6,
    float _a1, float _a2, float _a3, float _a4, float _a5, float _a6)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {//判断6个关节运动返回是否超过了限位
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        // DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;//找到目标位置与当前位置的差值
        // uint8_t index;
        // float maxAngle = AbsMaxOf6(deltaJoints, index);//找到差值最大的
        // float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        // for (int j = 1; j <= 6; j++)
        // {
        //     dynamicJointSpeeds.a[j - 1] =
        //         abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f); //0~10r/s
        // }
        dynamicJointSpeeds_Traj = {_v1, _v2, _v3, _v4, _v5, _v6};
        dynamicJointAcceleration_Traj = {_a1, _a2, _a3, _a4, _a5, _a6};
        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}

bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {//判断6个关节运动返回是否超过了限位
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;//找到目标位置与当前位置的差值
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);//找到差值最大的
        float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        for (int j = 1; j <= 6; j++)
        {
            dynamicJointSpeeds.a[j - 1] =
                abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f); //0~10r/s
        }

        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}

bool DummyRobot::ikCalculate(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};
    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);
    bool valid[8];
    int validCnt = 0;


    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;
        // printf("solution%d:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f \r\n", i,ikSolves.config[i].a[0],ikSolves.config[i].a[1],ikSolves.config[i].a[2],
        //     ikSolves.config[i].a[3],ikSolves.config[i].a[4],ikSolves.config[i].a[5]);

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                break;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        ikResultJoint = {ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]};
        return true;
    }

    return false;
}

bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

void DummyRobot::UpdateJointAngles(int i)
{
    motorJ[i]->UpdateAngle();
}

void DummyRobot::UpdateVelAcc(int i)
{
    motorJ[i]->UpdateVelAcc();
}

void DummyRobot::UpdateJointAnglesCallback(int i)
{
    currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

    if (motorJ[i]->state == CtrlStepMotor::FINISH)
        jointsStateFlag |= (1 << i);
    else
        jointsStateFlag &= ~(1 << i);

    current.a[i-1] =  motorJ[i]->current;
    velocity.a[i-1] = motorJ[i]->velocity;
    acceleration.a[i-1] = motorJ[i]->acceleration;
}


void DummyRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)_speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}


void DummyRobot::SetJointAcceleration(float _acc)
{
    // if (_acc < 0)_acc = 0;
    // else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]);
}


void DummyRobot::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting();
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}

void DummyRobot::Starting()
{
    MoveJ(0, -100, 0, 0, 0, 0);
    for (int j = 1; j <= 6; j++)
    {
        MoveJoints(targetJoints,j);
    }
    while (IsMoving())
        osDelay(10);
}

void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(40);

    MoveJ(0, -90, 0, 0, 0, 0);
    for (int j = 1; j <= 6; j++)
    {
        MoveJoints(targetJoints,j);
    }
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(40);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    for (int j = 1; j <= 6; j++)
    {
        MoveJoints(targetJoints,j);
    }
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}


void DummyRobot::SetRGBEnable(bool _enable)
{
    isRGBEnabled = _enable;
}

bool DummyRobot::GetRGBEnabled()
{
    return isRGBEnabled;
}

void DummyRobot::SetRGBMode(uint32_t mode)
{
    rgbMode = mode;
}

uint32_t DummyRobot::GetRGBMode()
{
    return rgbMode;
}

void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}


bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > DYNAMIC_CURRENT)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(20);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(200);
            jointSpeedRatio = 1;
            break;
        case COMMAND_MOTOR_TUNING:
            break;
        default:
            break;
    }
}

char data[200];
char data1[5];
void DummyRobot::DynamicCalculation(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,float _v1, float _v2, float _v3, float _v4, float _v5, float _v6,
    float _a1, float _a2, float _a3, float _a4, float _a5, float _a6)
{
    volatile uint32_t t1 = micros();
    float j[6] = {_j1, _j2, _j3, _j4, _j5, _j6};
    float v[6] = {_v1, _v2, _v3, _v4, _v5, _v6};
    float a[6] = {_a1, _a2, _a3, _a4, _a5, _a6};

    float current[6];
    dof6Dynamic->Yr_clc(j,v,a,current);
    volatile uint32_t t2 = micros();
    volatile uint32_t t3 = t2 - t1;
    sprintf(data, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n%d", current[0], current[1], current[2], current[3], current[4], current[5],t3);
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)data, sizeof(data));
}
void DummyRobot::DynamicCalculation_updata(float* _j,float* _v,float* _a,float* current,uint32_t* time)
{
    volatile uint32_t t1 = micros();
    dof6Dynamic->Yr_clc(_j,_v,_a,current);
    volatile uint32_t t2 = micros();
    uint32_t t3 = t2 - t1;
    memcpy(time, &t3, sizeof(t3));
}

// void DummyRobot::SetPlannerMode(uint32_t _mode)
// {
//     if (_mode < NoPlanner ||
//         _mode > Square)
//         return;
//
//     plannerMode_requesting = static_cast<PlannerMode>(_mode);
// }
//
// void DummyRobot::setTargetPose(float _x, float _y, float _z, float _a, float _b, float _c)
// {
//     DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
//     set_target_pose = pose6D;
//
// }

DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint8_t
_id) :
    nodeID(_id), hcan(_hcan)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}


void DummyHand::SetAngle(float _angle)
{
    if (_angle > 30)_angle = 30;
    if (_angle < 0)_angle = 0;

    uint8_t mode = 0x02;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_angle;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetMaxCurrent(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    uint8_t mode = 0x01;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetEnable(bool _enable)
{
    if (_enable)
        SetMaxCurrent(maxCurrent);
    else
        SetMaxCurrent(0);
}


uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}


void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    for (int j = 1; j <= 6; j++)
    {
        context->MoveJoints(context->targetJoints,j);
    }
    context->isEnabled = false;
    ClearFifo();
}


std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);

    return std::string{strBuffer};
}


uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_CONTINUES_TRAJECTORY:
            if (_cmd[0] == '>')
            {
                float joints[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                for (int j = 1; j <= 6; j++)
                {
                    context->MoveJoints(context->targetJoints,j);
                }

                while (context->IsMoving() && context->IsEnabled())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart6StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                for (int j = 1; j <= 6; j++)
                {
                    context->MoveJoints(context->targetJoints,j);
                }

                while (context->IsMoving())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart6StreamOutputPtr, "ok");
            }

            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>')
            {
                float joints[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart6StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart6StreamOutputPtr, "ok");
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;
    }

    return osMessageQueueGetSpace(commandFifo);
}


void DummyRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}


void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}
