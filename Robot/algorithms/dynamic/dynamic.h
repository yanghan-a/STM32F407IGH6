//
// Created by 15873 on 2025-03-17.
//

#ifndef DYNAMIC_H
#define DYNAMIC_H
#include "stm32f407xx.h"
#include "arm_math.h"
#include "memory.h"

static const float AXIS_Z[3] = {0.0f, 0.0f, 1.0f};
static const float REDUCTION[6] = {2.5f, 2.5f, 2.5f, 2.5f, 1.5f, 2.5f};
class DOF6Dynamic {
public:
    explicit DOF6Dynamic( float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT,float g);
    ~DOF6Dynamic();

    void Yr_clc(const float* q,const float* dq,const float* ddq, float* current);
private:
    float m_Y_temp[468]; // 复用缓冲区（替代Y_temp/Yf6/Yn6等）
    float m_Yr_temp[312];
    float m_H[6][30];        // 复用H1~H6
    float m_A[6][30];        // 复用A1~A6
    float R[6][9];
    float R_T[6][9];
    float P[6][9];
    float Yf[6][234];
    float Yn[6][234];
    float w00[3] = {0.0f, 0.0f, 0.0f};
    float dw00[3] = {0.0f, 0.0f, 0.0f};
    float dv00[3] = {0.0f, 0.0f, dyarmConfig.gravity};

    //计算Hi的buffer
    struct {
        float S_dwi[9];    // temp1
        float S_wi[9];     // temp2
        float S_wi_sq[9];  // temp3
        float sum[9];      // temp4
    }mem_Hi;
    //计算Ai的buffer
    struct
    {
        float K_dwi[18];
        float K_wi[18];
        float S_wi[9];
        float S_dvi[9];
        float matmul[18];
        float sum[18];
    }mem_Ai;
    //计算Yf和Yn的buffer
    float Yf_temp[234];
    float Yn_temp[234];
    float n[9];

    const float RAD_TO_DEG = 57.29577951308232f;

    struct DyArmConfig_t
    {
        float L_BASE;
        float D_BASE;
        float L_ARM;
        float L_FOREARM;
        float D_ELBOW;
        float L_WRIST;
        float gravity;
    };
    DyArmConfig_t dyarmConfig;
    // const float min_parameters[52] = {
    // 0.0524003002649345,1.01263143869989,1.61771370615417,
    //-0.103614906008431,-0.0147812374908038,0.0173290485423421,0.0323646233666617,-0.0351100786354692,0.333578064212064,0.0125646463563344,2.35030735085276,3.72892662878297,
    //0.0307790690203965,0.0156915000947613,0.00238708353025085,-0.00203877681213394,0.0313570429199304,
    //                            0.0688527243430339,0.0770953998686637,-0.0133397944775803,2.95566854878672,3.38056443706488,//2.80566854878672,3.18056443706488,
    //0.0578490917089285,0.00973747001050655,-0.0141219472357664,0.0326402392496366,0.0294983623220892,0.0156488229221564,0.00717430232882985,
    //                            -0.00535369764729391,1.21153521833331,1.51748852542249,
    //-0.00521766814530114,0.0129702960422451,0.00762978674154772,0.0275386263689479,-0.0292111728659347,
    //                            -0.00570590662797709,-0.0220717737681280,0.00833635307515021,0.573619521069702,0.449441436344774,
    //-0.00909363474621605,-0.00870514910449287,-0.00136959802501082,0.0168465620049946,-0.000912580975870561,
    //                                 -0.000835732932858618,-0.00595513684013572,0.00171421848151046,1.01747747067784,1.24423813423984
    // };
    const float min_parameters[52] = {
    -0.0617996244397836, 0.907899037367811, 1.44207914648817,
    -0.0191427374567779,-0.0845085906529424 ,0.101116600487977,-0.0101888715316815,-0.0226144454611170, 0.382004894964451, 0.0224329395510088, 2.13371730519082, 3.37472930554667,
    -0.0173639517944793,0.0511617156170381,-0.0318304857695370,-0.00788459413187759,-0.107116008468665,0.0745291380720427,0.0857445390149550,0.00660358557657489,2.23656275993411,3.26371087181728,
    0.0969054999409282,-0.000364489429756455,-0.000753980414728062,0.0266399294788995,-0.00455545351967393,0.0207396827202116,0.00504931825436849,-0.00382926963901401,1.07883726101127,1.54855539340736,
    0.00135581693614293,0.0229874246191647,0.0215363906897675,0.0110676827219414,-0.0288101929746753,-0.0179132770676982,-0.0202506389865176,-0.00290386271936033,0.523930296302675,0.426060118684863,
    -0.0108386140524547,0.00235273851471932,-0.00626919452492315,0.00693435702860326,0.00375110821086271,0.0118624229206161,0.00162474143769013,0.00468228384621085,0.727890177751744,0.922277241934057
    };
    void scalarMultiply(const float* vec, float scalar, float* result);
    void crossProduct(const float* A, const float* B, float* C);
    void crossProduct3(const float* A, const float* B, const float* C,  float* output);
    void MatrixAddition2(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _n);
    void MatrixAddition3(const float* _matrix1, const float* _matrix2,const float* _matrix3, float* _matrixOut,
                        const int _m, const int _n);
    void MatrixMultiplication(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                            uint16_t _m, uint16_t _l, uint16_t _n);
    float Sign(float x);
    void GetAdditionMatrix(float reduction,float q, float dq,float* matrix);
    void MDHTrans(float alpha,float a,float d,float theta,float* R, float* R_T, float* P );
    void motion_para_clc(const float *R_inv,const float dq,const float ddq,const float* w_pre,const float* dw_pre,
        const float* dv_pre,const float* Po, const float* axis, float* w,float* dw,float* dv);
    void getK(const float* vec, float* K);
    void getS(const float* vec, float* S);
    void getHi(const float* wi,const float* dwi,const float* dvi,float* Hi);
    void getAi(const float* wi,const float* dwi,const float* dvi,float* Ai);
    void get_Yf_Yn(int id,const float* R,const float* H,const float* A,const float* Yf_next,
    const float* Yn_next,const float* Po, float dq,float ddq,float reduction,float* Yf,float* Yn);
    void Ymatrix_clc(const float* q,const float* dq,const float* ddq, float* Y);

};



#endif //DYNAMIC_H
