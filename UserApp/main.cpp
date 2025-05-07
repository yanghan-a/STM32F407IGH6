#include "common_inc.h"
#include "usart.h"


// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
// SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
// MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 2000);
Timer timerUart6Tx(&htim10, 100);
Timer timerDynamicUpdate(&htim11, 100);


// 2x2-channel PWMs, used htim9 & htim12, each has 2-channel outputs
// PWM pwm(21000, 21000);

// RGB rgb(0);
// Robot instance
DummyRobot dummy(&hcan1);

void ThreadControlLoopUpdate(void* argument);
void ThreadControlLoopFixUpdate(void* argument);
void ThreadUartTx(void* argument);
// void ThreadOledUpdate(void* argument);

// void ThreadRGBUpdate(void* argument);

uint8_t uart_tx_buffer[256];

/* Default Entry -------------------------------------------------------*/
void Main(void)
{
    // Init all communication staff, including USB-CDC/VCP/UART/CAN etc.
    InitCommunication();

    // Init Robot.
    dummy.Init();

    // Init & Run User Threads.
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopFixUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityRealtime,
    };
    controlLoopFixUpdateHandle = osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                                             &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
        .name = "ControlLoopUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,
    };
    ControlLoopUpdateHandle = osThreadNew(ThreadControlLoopUpdate, nullptr,
                                          &ControlLoopUpdateTask_attributes);
    const osThreadAttr_t uart6TxTask_attributes = {
        .name = "uart6TxTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityHigh,
    };
    uart6TxTaskHandle = osThreadNew(ThreadUartTx, nullptr,
                                             &uart6TxTask_attributes);

    // Start Timer Callbacks.
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    timerUart6Tx.SetCallback(OnTimer10Callback);
    timerUart6Tx.Start();

    timerDynamicUpdate.SetCallback(OnTimer11Callback);
    timerDynamicUpdate.Start();

}

//考虑转子惯量的激励轨迹，测试轨迹122.8条件数
// const float a_matrix[6][5] = {  {-0.00667689144335028,0.00555279493518187,	0.215679888308004,	-0.222340260644520	,0.00778446884468378},
//                                 {0.323749409544191,	-1.91192425824356e-05,	-0.0369437114012986	,-0.0198066780245666,	-0.266979900875744},
//                                 {0.00117922116902068,	0.706470068442750,-0.352383225487775	,-0.118472741283678,	-0.236793322840318},
//                                 {0.781644860931682	,-0.763753965232784	,0.00134529062002458,	0.0192501849819349,	-0.0384863713008576},
//                                 {-0.0440609647339360,	0.419671759372827,	0.0515637059725410,	0.154239034189585,	-0.581413534801017},
//                                 {0.444704941662919,	-0.913825271634299,	0.224701354621913,	0.132344933162587,	0.112074042186880}
// };
// const float b_matrix[6][5] = {  {0.0620367356779697,	0.0299279697703382,	-0.590667821299827,	0.397526793676656	,0.0120007227948418},
//                                 {-0.0402002262450320,	5.41431153663920e-06,	0.0459638610991600,	-0.374201496461287	,0.279820760033925},
//                                 {-0.000372348957972621,	0.00331262141111746,	-0.0793349666790479,	0.0479164810920279	,0.00801721636095397},
//                                 {0.527629905710756,	-0.372040952319376,	0.0335419648760533,	0.00771092037174998,	0.0169964845625673},
//                                 {0.0779274845298214,	0.123082808700100,	-0.141238994040016,	0.180056474723353,	-0.124120403740677},
//                                 {0.407293500701078,	-0.485326517161727	,0.354625778190522,	-0.636071971007770	,0.408754016616378}
// };
// const float q[6] = { -0.0288177243549505,
//                      -1.84474133537403,
//                      -0.0184276971538652,
//                      0.569961085034361,
//                      0.179175460580366,
//                      0.327177435284013
// };

//考虑转子惯量的激励轨迹，测试轨迹57.0条件数
const float a_matrix[6][5] = {  {0.375605021648490,	-5.20334682264480e-05,	0.0393098726555316,	-0.418215645659139,	0.00335278482334372},
                                {-6.72199616370280e-05,	-0.442888213440218,	0.635919579846563,	-0.00916019330581495,	-0.183803953138893},
                                {-0.0773899504616084	,-0.397293466300001,	0.420852784494899,	0.0138807339892543,	0.0399498982774558},
                                {0.728301240292902	,-0.420456199664400	,-0.0913300417931242,	-0.0609147077258286,	-0.155600291109549},
                                {0.00198091318441475,	-0.262559396273590,	-0.342514301015317,	0.112235738897725,	0.490857045206767},
                                {0.759154673226424,	-0.351793685657654,	-0.325331729238871,	-0.0836147035788329,	0.00158544524893431}
};
const float b_matrix[6][5] = {  {-0.609543974040559	,5.11419786907279e-06,	0.0184944188577194,	0.528220216720232,	-0.311766075561854},
                                {0.000120289714255518,	-0.357329428646497,	0.259025313179261,	0.00796465124613009	,-0.0188791953887131},
                                {0.203315044531381	,-0.650570144001542,	0.280626245976777,	0.0732377508774574,	-0.00740089959369143},
                                {0.0344219758262941,	0.528114340672573,	0.0677524228869456	,-0.0870147626753532,	-0.189169775026173},
                                {-0.00221724758148770	,-0.836170689804693,	0.402850607916507,	-0.0820954845617123,	0.158877748337640},
                                {0.561491384181346	,-0.427938096454032,	-0.0197314382693417,	0.0798942749727932,	0.00680040472871405}
};
const float q[6] = { -0.849369779016732,
                     -1.89491473327570,
                     -0.0184598847709479,
                     0.416151408350225,
                     -0.437305864066428,
                     0.576584680925337
};

//激励轨迹，条件数54.3
// const float a_matrix[6][5] = {  {0.747550062217377,	-0.00937334271915357	,0.0164393726531833	,0.0738911465832424	,-0.828507238734649},
//                                 {0.00236312581342073,	-0.296151628108756	,0.741970715735629	,-0.461528974631090,	0.0133467611907969},
//                                 {-0.261821991540631,	0.0873387761563515,	0.577119700753183,	0.0951686477903112,	-0.497805133159215},
//                                 {-0.661301519528253,	-0.0863470951718834	,0.100246576383723,	0.112990229129664	,0.534411809186749},
//                                 {0.000662640335075390,	0.902199707691703,	-0.105865651164483,	-0.0503772033119528,	-0.746619493550342},
//                                 {-0.788577372616163,	0.699350806801528,	0.140129046951142	,-0.315367417060863,	0.264464935924356}
// };
// const float b_matrix[6][5] = {  {-0.224309009290802,	-0.00298592956696288	,-0.0193909927260223,	-0.0310163173946382,	0.0825038232362696},
//                                 {-0.00391130551729033,	0.0209263748964849,	-0.0979635324371864,	0.0532287198423375	,0.00860685473330597},
//                                 {0.182204931052930,	-0.0713658010839648	,-0.262325928708855,	-0.0209077585357445	,0.166227098276908},
//                                 {-0.489149603118245	,-0.0950325712249146,	-0.159673495486698,	0.0925501348619425,	0.157606938516080},
//                                 {0.0302963769631898,	0.545494108571519,0.0759601628777177,	0.0198695822029368	,-0.285728682310226},
//                                 {-0.302704745513092,	-0.103255282022617,	-0.282309398655101,	-0.137095036769267,	0.380904730520140}
// };
// const float q[6] = { -0.355741462474916,
//                      -1.76295419449212,
//                      0.138621622692846,
//                      -0.851847334815950,
//                      0.439562309764481,
//                      -0.647009936239183
// };

//负载0.5kg测试轨迹，条件数57.6
// const float a_matrix[6][5] = {  {0.322535948902418,	-0.278649368715651,	-0.0404147785444424,	-0.134569070343303,	0.131097268700979},
//                                 {0.0237919247135039,	-0.691289340484477,	0.00230081109211658,	0.000716846261186951,	0.664479758417670},
//                                 {0.104297247723081,	0.119202620064300,	-0.131803722915770,	-0.106391637138174,	0.0146954922665635},
//                                 {-0.577830583564175,	-0.0862659329417923,	0.723504563542883,	0.201132373087065	,-0.260540420123981},
//                                 {0.0824831255756590,	-0.964314790571136,	0.328291072065392	,0.296571354544711,	0.256969238385373},
//                                 {-0.908106673278423	,-0.0713155945875227,	0.642930370800042,	0.0777617423543746,	0.258730154711529}
// };
// const float b_matrix[6][5] = {  {0.716460188359933,	0.0250593698287546,	-0.0261144000591568,	-0.387765951962285,	0.172565616001833},
//                                 {-0.00726849489247392,	-0.243351073481919,	-0.00734276073802982,	0.000537494106721578,	0.102769789528703},
//                                 {0.00620641637103767,	0.333413163375635,	-0.795739732854549,	0.424229664503799,	0.00345355948522847},
//                                 {0.0555291609760570	,0.0609514322057382,	0.0419477592324197,	-0.0579781386157241	,-0.0142725497243793},
//                                 {-0.146822077268738,	-0.0870132064891155,	0.177021682550390,	-0.249391517427724,	0.157469902461339},
//                                 {0.0700511000384349,	0.214886271022423,	-0.127553263335275,	0.224472904248811,	-0.203011093814540}
// };
// const float q[6] = { 1.04701146998271,
//                      -1.92151900560098,
//                      0.0229414664043041,
//                      0.131523128557895,
//                      -0.258110088963714,
//                      0.239516234276724
// };




bool start_counter = false;
bool current_start_counter = false;
bool start_shijiao = false;
extern bool play_flag;
bool play_flag_p = false;
bool play_flag_first = true;
extern float pos_all[6][1500];
int32_t point_shijiao = 0;
int32_t point = 0;
float freq = 100.0f;
float dt = 1/freq;
float period = 10.0f;
float period_int = period*freq;
float all_time = period_int*1;
const float rad2deg = 57.295779513082320876798154814105f;
const float deg2rad = 0.01745329251994329576923690768489f;
const float wf = 0.2*3.14159265358979323846f;

float sin_value[5];
float cos_value[5];
float vel[6];
float acc[6];
float vel_rad[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float acc_rad[6]= {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
/* Thread Definitions -----------------------------------------------------*/
char arr[10];
char arr1[30];
osThreadId_t controlLoopFixUpdateHandle;
int i = 0;
void ThreadControlLoopFixUpdate(void* argument)
{
    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(start_counter&& point<=all_time&& (i%2==0) )//&& (i%2==0)
        {
            int j = i/2;
            // int j = i;
            vel[j] = a_matrix[j][0]*cos_value[0]+b_matrix[j][0]*sin_value[0]+a_matrix[j][1]*cos_value[1]+b_matrix[j][1]*sin_value[1]+
                    a_matrix[j][2]*cos_value[2]+b_matrix[j][2]*sin_value[2]+a_matrix[j][3]*cos_value[3]+b_matrix[j][3]*sin_value[3]+
                        a_matrix[j][4]*cos_value[4]+b_matrix[j][4]*sin_value[4];
            vel_rad[j] = vel[j];
            vel[j] = vel[j]*rad2deg;
            acc[j] = -a_matrix[j][0]*sin_value[0]*wf*1+b_matrix[j][0]*cos_value[0]*wf*1-a_matrix[j][1]*sin_value[1]*wf*2+b_matrix[j][1]*cos_value[1]*wf*2-
                    a_matrix[j][2]*sin_value[2]*wf*3+b_matrix[j][2]*cos_value[2]*wf*3-a_matrix[j][3]*sin_value[3]*wf*4+b_matrix[j][3]*cos_value[3]*wf*4-
                        a_matrix[j][4]*sin_value[4]*wf*5+b_matrix[j][4]*cos_value[4]*wf*5;
            acc_rad[j] = acc[j];
            acc[j] = acc[j]*rad2deg;
        }
        i++;

        if (dummy.IsEnabled())
        {
            if(i%2==1)
            {
                // Send control command to Motors & update Joint states
                switch (dummy.commandMode)
                {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints, (i+1)/2);
                    // dummy.UpdateJointPose6D();
                    break;
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    // dummy.UpdateJointPose6D();
                    break;
                case DummyRobot::DYNAMIC_CURRENT:
                    dummy.DynamicCurrentRequired(dummy.current_real, (i+1)/2);
                    break;
                }
            }else
            {
                dummy.UpdateVelAcc(i/2);
            }

        } else
        {
            if(i%2==1)
            {
                dummy.UpdateJointAngles((i+1)/2);
            }else
            {
                dummy.UpdateVelAcc(i/2);
            }

        }
        if(i ==12)
        {
            i = 0;
            dummy.UpdateJointPose6D();
        }
        // if (dummy.IsEnabled())
        // {
        //         // Send control command to Motors & update Joint states
        //         switch (dummy.commandMode)
        //         {
        //         case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
        //         case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
        //         case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
        //             dummy.MoveJoints(dummy.targetJoints, i);
        //             // dummy.UpdateJointPose6D();
        //             break;
        //         case DummyRobot::COMMAND_MOTOR_TUNING:
        //             dummy.tuningHelper.Tick(10);
        //             // dummy.UpdateJointPose6D();
        //             break;
        //         }
        //
        // } else
        // {
        //     dummy.UpdateJointAngles(i);
        // }
        // if(i ==6)
        // {
        //     i = 0;
        //     dummy.UpdateJointPose6D();
        // }
    }
}


osThreadId_t ControlLoopUpdateHandle;
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
    }
}

uint8_t joint_angles[128];
uint8_t shijiao_pos[64];
float current_pos[6];
float pos_rad[6]= {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};;
float pos[6];

float current_convert[6] = {0.125f,0.0930f,0.0930f,0.1263f,0.41667f,0.25f};
// float p1 = 45.0f,p2 = -45.0f,p3 = 50.0f,p4 = 45.0f,p5 = -40.0f,p6 = 90.0f;
extern int id[6];
extern int current_id[6];
osThreadId_t uart6TxTaskHandle;
char buffer_temp[128];
void ThreadUartTx(void* argument)
{
    for (;;)
    {

        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        float time_f = (float)dummy.dynamicCalculationTime/1000.0f;
        memcpy(joint_angles + 0 * sizeof(float), &dummy.currentJoints.a[0] ,sizeof(float));//targetPose6D
        memcpy(joint_angles + 1 * sizeof(float), &dummy.currentJoints.a[1] , sizeof(float));
        memcpy(joint_angles + 2 * sizeof(float), &dummy.currentJoints.a[2] , sizeof(float));
        memcpy(joint_angles + 3 * sizeof(float), &dummy.currentJoints.a[3] , sizeof(float));
        memcpy(joint_angles + 4 * sizeof(float), &dummy.currentJoints.a[4], sizeof(float));
        memcpy(joint_angles + 5 * sizeof(float), &dummy.currentJoints.a[5], sizeof(float));

        // memcpy(joint_angles + 6 * sizeof(float),  &pos[0], sizeof(float));
        // memcpy(joint_angles + 7 * sizeof(float),  &pos[1], sizeof(float));
        // memcpy(joint_angles + 8 * sizeof(float),  &pos[2], sizeof(float));
        // memcpy(joint_angles + 9 * sizeof(float),  &pos[3], sizeof(float));
        // memcpy(joint_angles + 10 * sizeof(float), &pos[4], sizeof(float));
        // memcpy(joint_angles + 11 * sizeof(float), &pos[5], sizeof(float));

        // memcpy(joint_angles +  12* sizeof(float),  &dummy.velocity.a[0], sizeof(float));
        // memcpy(joint_angles +  13* sizeof(float),  &dummy.velocity.a[1], sizeof(float));
        // memcpy(joint_angles +  14* sizeof(float),  &dummy.velocity.a[2], sizeof(float));
        // memcpy(joint_angles +  15* sizeof(float),  &dummy.velocity.a[3], sizeof(float));
        // memcpy(joint_angles +  16* sizeof(float),  &dummy.velocity.a[4],sizeof(float));
        // memcpy(joint_angles +  17* sizeof(float),  &dummy.velocity.a[5],sizeof(float));

        // memcpy(joint_angles + 18 * sizeof(float),  &vel[0], sizeof(float));
        // memcpy(joint_angles + 19 * sizeof(float),  &vel[1], sizeof(float));
        // memcpy(joint_angles + 20 * sizeof(float),  &vel[2], sizeof(float));
        // memcpy(joint_angles + 21 * sizeof(float),  &vel[3], sizeof(float));
        // memcpy(joint_angles + 22 * sizeof(float),  &vel[4], sizeof(float));
        // memcpy(joint_angles + 23 * sizeof(float),  &vel[5], sizeof(float));

        // memcpy(joint_angles + 0 * sizeof(float), &pos_rad[0] ,sizeof(float));//targetPose6D
        // memcpy(joint_angles + 1 * sizeof(float), &pos_rad[1] , sizeof(float));
        // memcpy(joint_angles + 2 * sizeof(float), &pos_rad[2] , sizeof(float));
        // memcpy(joint_angles + 3 * sizeof(float), &pos_rad[3] , sizeof(float));
        // memcpy(joint_angles + 4 * sizeof(float), &pos_rad[4], sizeof(float));
        // memcpy(joint_angles + 5 * sizeof(float), &pos_rad[5], sizeof(float));

        // memcpy(joint_angles + 6 * sizeof(float),  &dummy.current.a[0], sizeof(float));
        // memcpy(joint_angles + 7 * sizeof(float),  &dummy.current.a[1], sizeof(float));
        // memcpy(joint_angles + 8 * sizeof(float),  &dummy.current.a[2], sizeof(float));
        // memcpy(joint_angles + 9 * sizeof(float),  &dummy.current.a[3], sizeof(float));
        // memcpy(joint_angles + 10 * sizeof(float), &dummy.current.a[4], sizeof(float));
        // memcpy(joint_angles + 11 * sizeof(float), &dummy.current.a[5], sizeof(float));

        //将位置、速度、加速度传入动力学函数，计算需要的理论电流
        memcpy(joint_angles + 6 * sizeof(float),  &dummy.current_required.a[0], sizeof(float));
        memcpy(joint_angles + 7 * sizeof(float),  &dummy.current_required.a[1], sizeof(float));
        memcpy(joint_angles + 8 * sizeof(float),  &dummy.current_required.a[2], sizeof(float));
        memcpy(joint_angles + 9 * sizeof(float),  &dummy.current_required.a[3], sizeof(float));
        memcpy(joint_angles + 10 * sizeof(float), &dummy.current_required.a[4],  sizeof(float));
        memcpy(joint_angles + 11 * sizeof(float), &dummy.current_required.a[5],  sizeof(float));

        // memcpy(joint_angles + 12 * sizeof(float),  &vel_rad[0], sizeof(float));
        // memcpy(joint_angles + 13 * sizeof(float),  &vel_rad[1], sizeof(float));
        // memcpy(joint_angles + 14 * sizeof(float),  &vel_rad[2], sizeof(float));
        // memcpy(joint_angles + 15 * sizeof(float),  &vel_rad[3], sizeof(float));
        // memcpy(joint_angles + 16 * sizeof(float),  &vel_rad[4], sizeof(float));
        // memcpy(joint_angles + 17 * sizeof(float),  &vel_rad[5], sizeof(float));
        //
        // memcpy(joint_angles + 18 * sizeof(float),  &acc_rad[0], sizeof(float));
        // memcpy(joint_angles + 19 * sizeof(float),  &acc_rad[1], sizeof(float));
        // memcpy(joint_angles + 20 * sizeof(float),  &acc_rad[2], sizeof(float));
        // memcpy(joint_angles + 21 * sizeof(float),  &acc_rad[3], sizeof(float));
        // memcpy(joint_angles + 22 * sizeof(float),  &acc_rad[4],sizeof(float));
        // memcpy(joint_angles + 23 * sizeof(float),  &acc_rad[5],sizeof(float));

        //
        memcpy(joint_angles + 12 * sizeof(float),  &dummy.velocity.a[0], sizeof(float));
        memcpy(joint_angles + 13 * sizeof(float),  &dummy.velocity.a[1], sizeof(float));
        memcpy(joint_angles + 14 * sizeof(float),  &dummy.velocity.a[2], sizeof(float));
        memcpy(joint_angles + 15 * sizeof(float),  &dummy.velocity.a[3], sizeof(float));
        memcpy(joint_angles + 16 * sizeof(float),  &dummy.velocity.a[4],sizeof(float));
        memcpy(joint_angles + 17 * sizeof(float),  &dummy.velocity.a[5],sizeof(float));

        memcpy(joint_angles + 18 * sizeof(float),  &dummy.acceleration.a[0], sizeof(float));
        memcpy(joint_angles + 19 * sizeof(float),  &dummy.acceleration.a[1], sizeof(float));
        memcpy(joint_angles + 20 * sizeof(float),  &dummy.acceleration.a[2], sizeof(float));
        memcpy(joint_angles + 21 * sizeof(float),  &dummy.acceleration.a[3], sizeof(float));
        memcpy(joint_angles + 22 * sizeof(float),  &dummy.acceleration.a[4], sizeof(float));
        memcpy(joint_angles + 23 * sizeof(float),  &dummy.acceleration.a[5], sizeof(float));


        // memcpy(joint_angles +  12* sizeof(float),  &vel[0], sizeof(float));
        // memcpy(joint_angles +  13* sizeof(float),  &vel[1], sizeof(float));
        // memcpy(joint_angles +  14* sizeof(float),  &vel[2], sizeof(float));
        // memcpy(joint_angles +  15* sizeof(float),  &vel[3], sizeof(float));
        // memcpy(joint_angles +  16* sizeof(float),  &vel[4],sizeof(float));
        // memcpy(joint_angles +  17* sizeof(float),  &vel[5],sizeof(float));
        //
        // memcpy(joint_angles + 18 * sizeof(float),  &acc[0], sizeof(float));
        // memcpy(joint_angles + 19 * sizeof(float),  &acc[1], sizeof(float));
        // memcpy(joint_angles + 20 * sizeof(float),  &acc[2], sizeof(float));
        // memcpy(joint_angles + 21 * sizeof(float),  &acc[3], sizeof(float));
        // memcpy(joint_angles + 22 * sizeof(float),  &acc[4], sizeof(float));
        // memcpy(joint_angles + 23 * sizeof(float),  &acc[5], sizeof(float));

        // memcpy(joint_angles + 18 * sizeof(float),  &pos[0], sizeof(float));
        // memcpy(joint_angles + 19 * sizeof(float),  &pos[1], sizeof(float));
        // memcpy(joint_angles + 20 * sizeof(float),  &pos[2], sizeof(float));
        // memcpy(joint_angles + 21 * sizeof(float),  &pos[3], sizeof(float));
        // memcpy(joint_angles + 22 * sizeof(float),  &pos[4], sizeof(float));
        // memcpy(joint_angles + 23 * sizeof(float),  &pos[5], sizeof(float));

        memcpy(joint_angles + 24 * sizeof(float),  &time_f,sizeof(float));

        // memcpy(joint_angles + 0 * sizeof(float), &dummy.currentJoints.a[5] ,sizeof(float));//targetPose6D
        // memcpy(joint_angles + 1 * sizeof(float), &dummy.velocity.a[5] , sizeof(float));
        // memcpy(joint_angles + 2 * sizeof(float), &dummy.acceleration.a[5] , sizeof(float));
        // memcpy(joint_angles + 3 * sizeof(float), &pos[5] , sizeof(float));
        // memcpy(joint_angles + 4 * sizeof(float), &vel[5], sizeof(float));
        // memcpy(joint_angles + 5 * sizeof(float),  &acc[5], sizeof(float));
        // joint_angles[4*6]=0x00;
        // joint_angles[4*6+1]=0x00;
        // joint_angles[4*6+2]=0x80;
        // joint_angles[4*6+3]=0x7f;
        joint_angles[4*25]=0x00;
        joint_angles[4*25+1]=0x00;
        joint_angles[4*25+2]=0x80;
        joint_angles[4*25+3]=0x7f;
        // HAL_UART_Transmit_DMA(&huart6, (uint8_t*)joint_angles, 104);
        if(start_counter)
        {
            if(point<=all_time)
            {
                for(int i=1;i<6;i++)
                {
                    sin_value[i-1] = sinf((float)point*dt*(float)i/period*2*3.14159265358979323846f);
                    cos_value[i-1] = cosf((float)point*dt*(float)i/period*2*3.14159265358979323846f);
                }

                if(point == 0)
                {
                    for(int i=0;i<6;i++)
                    {
                        current_pos[i] = dummy.currentJoints.a[i];
                        pos_rad[i] = current_pos[i]*deg2rad;
                    }
                }

                for(int i=0;i<6;i++)
                {
                    if(id[i]==1)
                    {
                        pos[i] = a_matrix[i][0]*sin_value[0]/wf/1-b_matrix[i][0]*cos_value[0]/wf/1+a_matrix[i][1]*sin_value[1]/wf/2-b_matrix[i][1]*cos_value[1]/wf/2+
                        a_matrix[i][2]*sin_value[2]/wf/3-b_matrix[i][2]*cos_value[2]/wf/3+a_matrix[i][3]*sin_value[3]/wf/4-b_matrix[i][3]*cos_value[3]/wf/4+
                            a_matrix[i][4]*sin_value[4]/wf/5-b_matrix[i][4]*cos_value[4]/wf/5 +q[i];
                        pos_rad[i] = pos[i];
                        pos[i] = pos[i]*rad2deg;
                    }else
                    {
                        pos[i] = current_pos[i];
                    }
                }

                point ++;
                dummy.MoveJ_Traj(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],vel[0],vel[1],vel[2],vel[3],
                    vel[4],vel[5],acc[0],acc[1],acc[2],acc[3],acc[4],acc[5]);
            }
            else
            {
                point = 0;
                start_counter = false;
                dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE);
            }
        }


        if(play_flag)
        {
            if(play_flag_first)
            {
                dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE);
                dummy.MoveJ(pos_all[0][0],pos_all[1][0],pos_all[2][0],pos_all[3][0],pos_all[4][0],pos_all[5][0]);
                play_flag_first = false;
            }
            //
            // sprintf(buffer_temp, "Current State: %d\n",dummy.jointsStateFlag);
            // HAL_UART_Transmit(&huart6, (uint8_t *) buffer_temp, 30, HAL_MAX_DELAY);
            if(dummy.jointsStateFlag==0b01111110)
            {
                play_flag_p = true;
                play_flag = false;
            }
        }
        if(play_flag_p)
        {
            memcpy(shijiao_pos + 0 * sizeof(float),&dummy.currentJoints.a[0]  ,sizeof(float));//targetPose6D
            memcpy(shijiao_pos + 1 * sizeof(float),&dummy.currentJoints.a[1]  , sizeof(float));
            memcpy(shijiao_pos + 2 * sizeof(float),&dummy.currentJoints.a[2]  , sizeof(float));
            memcpy(shijiao_pos + 3 * sizeof(float),&dummy.currentJoints.a[3]  , sizeof(float));
            memcpy(shijiao_pos + 4 * sizeof(float),&dummy.currentJoints.a[4] , sizeof(float));
            memcpy(shijiao_pos + 5 * sizeof(float),&dummy.currentJoints.a[5] , sizeof(float));

            memcpy(shijiao_pos + 6 * sizeof(float), &pos_all[0][point_shijiao]  ,sizeof(float));//targetPose6D
            memcpy(shijiao_pos + 7 * sizeof(float), &pos_all[1][point_shijiao]  , sizeof(float));
            memcpy(shijiao_pos + 8* sizeof(float),  &pos_all[2][point_shijiao]  , sizeof(float));
            memcpy(shijiao_pos + 9 * sizeof(float), &pos_all[3][point_shijiao]  , sizeof(float));
            memcpy(shijiao_pos + 10 * sizeof(float),&pos_all[4][point_shijiao] , sizeof(float));
            memcpy(shijiao_pos + 11 * sizeof(float),&pos_all[5][point_shijiao] , sizeof(float));

            shijiao_pos[4*12]=0x00;
            shijiao_pos[4*12+1]=0x00;
            shijiao_pos[4*12+2]=0x80;
            shijiao_pos[4*12+3]=0x7f;
            HAL_UART_Transmit_DMA(&huart6, (uint8_t*)shijiao_pos, 52);

            dummy.SetCommandMode(DummyRobot::COMMAND_CONTINUES_TRAJECTORY);
            dummy.MoveJ(pos_all[0][point_shijiao],pos_all[1][point_shijiao],pos_all[2][point_shijiao],
                pos_all[3][point_shijiao],pos_all[4][point_shijiao],pos_all[5][point_shijiao]);
            point_shijiao++;
            if(point_shijiao>=1500)
            {
                point_shijiao = 0;
                play_flag_p = false;
                dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE);
            }

        }

        // 动力学力矩模式
        if(current_start_counter)
        {
            for(int i=0;i<6;i++)
            {
                if(current_id[i]==1)
                {
                    dummy.current_real.a[i] = dummy.current_required.a[i]*current_convert[i];
                }else
                {
                    dummy.current_real.a[i] = 0.0f;
                }
            }
        }
    }
}


/* Timer Callbacks -------------------------------------------------------*/
void OnTimer7Callback()//600Hz
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // if(dummy.withplanner)
    // {
    //     dummy.ControlLoop();
    // }

    // Wake & invoke thread IMMEDIATELY.
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}





