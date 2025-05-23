#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "algorithms/dynamic/dynamic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "algorithms/planner/planner.h"

#define ALL 0

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 2               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 2               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */


class DummyHand
{
public:
    uint8_t nodeID = 7;
    float maxCurrent = 0.7;


    DummyHand(CAN_HandleTypeDef* _hcan, uint8_t _id);


    void SetAngle(float _angle);
    void SetMaxCurrent(float _val);
    void SetEnable(bool _enable);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("set_angle", *this, &DummyHand::SetAngle, "angle"),
            make_protocol_function("set_enable", *this, &DummyHand::SetEnable, "enable"),
            make_protocol_function("set_current_limit", *this, &DummyHand::SetMaxCurrent, "current")
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8];
    CAN_TxHeaderTypeDef txHeader;
    float minAngle = 0;
    float maxAngle = 45;
};


class DummyRobot
{
public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();


    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING,
        DYNAMIC_CURRENT
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context)
        {
        }

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);


        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }


    private:
        DummyRobot* context;
        float time = 0;
        uint8_t tuningFlag = 0;
        float frequency = 1;
        float amplitude = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);


    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -165.3, 90, 0, 0, 0};//{0, -73, 180, 0, 0, 0};
    const DOF6Kinematic::Joint6D_t start = {0, -165.3, 90, 0, 0, 0};//{0, -100, 0, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED = 80;  // r/s 30 实际上这个值还得除以10，也就是6圈/s
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {100, 100, 100, 100, 100, 100};
    const float DEFAULT_JOINT_ACCELERATION_LOW = 50;    // 0~100 30
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;  // 0~100 100
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;

    uint32_t dynamicCalculationTime = 0;
    DOF6Kinematic::Joint6D_t current_required = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    DOF6Kinematic::Joint6D_t current_real = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    DOF6Kinematic::Joint6D_t current = {};
    DOF6Kinematic::Joint6D_t velocity = {};
    DOF6Kinematic::Joint6D_t acceleration = {};

    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = start;
    // DOF6Kinematic::Joint6D_t targetJoints_Traj = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    DOF6Kinematic::Pose6D_t targetPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor* motorJ[7] = {nullptr};
    DummyHand* hand = {nullptr};

    //添加一个IK运算结果存储，用来在oled显示
    DOF6Kinematic::Joint6D_t ikResultJoint = {};

    // MotionPlanner motionPlanner;
    bool withplanner = false;
    planner planner_traj;
    enum planner_mode
    {
        NO_PLANNER = 0,
        COMMAND_LINE = 1,
    };
    const planner_mode DEFAULT_PLANNER_MODE = NO_PLANNER;
    planner_mode plannerMode = NO_PLANNER;
    planner_mode requestPlannerMode = COMMAND_LINE;

    planner::Pose6D goal_pose = {};
    int32_t goal_index = 0;
    bool newplanner = false;


    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    bool ikCalculate(float _x, float _y, float _z, float _a, float _b, float _c);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints, int i);
    void DynamicCurrentRequired(DOF6Kinematic::Joint6D_t _currents, int i);
    bool MoveJ_Traj(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,float _v1, float _v2, float _v3, float _v4, float _v5, float _v6,
    float _a1, float _a2, float _a3, float _a4, float _a5, float _a6);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    void UpdateJointAngles(int i);
    void UpdateVelAcc(int i);
    void UpdateJointAnglesCallback(int i);
    void UpdateJointPose6D();
    void Reboot();
    void SetEnable(bool _enable);
    void SetRGBEnable(bool _enable);
    bool GetRGBEnabled();
    void SetRGBMode(uint32_t mode);
    uint32_t GetRGBMode();
    void CalibrateHomeOffset();
    void Starting();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);

    void DynamicCalculation(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,float _v1, float _v2, float _v3, float _v4, float _v5, float _v6,
float _a1, float _a2, float _a3, float _a4, float _a5, float _a6);
    void DynamicCalculation_updata(float* _j,float* _v,float* _a,float* current,uint32_t* time);

    // void SetPlannerMode(uint32_t _mode);
    // void setTargetPose(float _x, float _y, float _z, float _a, float _b, float _c);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DummyRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DummyRobot::Homing),
            make_protocol_function("resting", *this, &DummyRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot", *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable", *this, &DummyRobot::SetEnable, "enable"),
            make_protocol_function("move_j", *this, &DummyRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6"),
            make_protocol_function("move_l", *this, &DummyRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("set_joint_speed", *this, &DummyRobot::SetJointSpeed, "speed"),
            make_protocol_function("set_joint_acc", *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode, "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions()),

            //添加一个IK运算结果给用户
            make_protocol_function("ik_calculate", *this, &DummyRobot::ikCalculate, "x", "y", "z", "a", "b", "c")
        );
    }


    class CommandHandler
    {
    public:
        explicit CommandHandler(DummyRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }

        uint32_t Push(const std::string &_cmd);
        std::string Pop(uint32_t timeout);
        uint32_t ParseCommand(const std::string &_cmd);
        uint32_t GetSpace();
        void ClearFifo();
        void EmergencyStop();


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
        char strBuffer[64]{};
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds_Traj = {0, 0, 0, 0, 0, 0};
    DOF6Kinematic::Joint6D_t dynamicJointAcceleration_Traj= {0, 0, 0, 0, 0, 0};
    DOF6Kinematic* dof6Solver;
    DOF6Dynamic* dof6Dynamic;
    bool isEnabled = false;
    bool isRGBEnabled = false;
    uint32_t rgbMode = 0;
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
