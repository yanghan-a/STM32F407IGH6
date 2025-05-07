//
// Created by 15873 on 2025-03-17.
//

#include "dynamic.h"

#include <cstring>

inline float cosf_(float x)
{
    return arm_cos_f32(x);
}

inline float sinf_(float x)
{
    return arm_sin_f32(x);
}
DOF6Dynamic::DOF6Dynamic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT, float g)
    : dyarmConfig(DyArmConfig_t{L_BS, D_BS, L_AM, L_FA, D_EW, L_WT, g}),
      w00{0.0f, 0.0f, 0.0f},
      dw00{0.0f, 0.0f, 0.0f},
      dv00{0.0f, 0.0f, g}  // 注意：这里使用传入的g代替dyarmConfig.gravity
{
    // 清零所有数组和结构体
    memset(m_Y_temp, 0, sizeof(m_Y_temp));
    memset(m_Yr_temp, 0, sizeof(m_Yr_temp));
    memset(m_H, 0, sizeof(m_H));
    memset(m_A, 0, sizeof(m_A));
    memset(R, 0, sizeof(R));
    memset(R_T, 0, sizeof(R_T));
    memset(P, 0, sizeof(P));
    memset(Yf, 0, sizeof(Yf));
    memset(Yn, 0, sizeof(Yn));

    // 清零结构体成员
    memset(&mem_Hi, 0, sizeof(mem_Hi));
    memset(&mem_Ai, 0, sizeof(mem_Ai));

    // 清零其他缓冲区
    memset(Yf_temp, 0, sizeof(Yf_temp));
    memset(Yn_temp, 0, sizeof(Yn_temp));
    memset(n, 0, sizeof(n));
}


inline void DOF6Dynamic::scalarMultiply(const float* vec, float scalar, float* result) {
    for (int i = 0; i < 3; i++) {
        result[i] = vec[i] * scalar;
    }
}
inline void DOF6Dynamic::crossProduct(const float* A, const float* B, float* C) {
    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];
}
inline void DOF6Dynamic::crossProduct3(const float* A, const float* B, const float* C,  float* output)
{
    float temp1[3];
    crossProduct(B, C, temp1);
    crossProduct(A, temp1, output);
}
void DOF6Dynamic::MatrixAddition2(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _n)
{
    int i, j;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            _matrixOut[i * _n + j] = _matrix1[i * _n + j] + _matrix2[i * _n + j];
        }
    }
}
void DOF6Dynamic::MatrixAddition3(const float* _matrix1, const float* _matrix2,const float* _matrix3, float* _matrixOut,
                        const int _m, const int _n)
{
    int i, j;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            _matrixOut[i * _n + j] = _matrix1[i * _n + j] + _matrix2[i * _n + j]+_matrix3[i * _n + j];
        }
    }
}
void DOF6Dynamic::MatrixMultiplication(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        uint16_t _m, uint16_t _l, uint16_t _n)
{
    // arm_matrix_instance_f32 matA = {_m, _l, (float*)_matrix1};
    // arm_matrix_instance_f32 matB = {_l, _n, (float*)_matrix2};
    // arm_matrix_instance_f32 matC = {_m, _n, _matrixOut};
    // arm_mat_mult_f32(&matA, &matB, &matC);
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            tmp = 0.0f;
            for (k = 0; k < _l; k++)
            {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}

// 使用查表法优化Sign函数
inline float DOF6Dynamic::Sign(float x) {
    const float sign_table[] = {-1.0f, 0.0f, 1.0f};
    return sign_table[(x > 0) + (x >= 0)];
}
void DOF6Dynamic::GetAdditionMatrix(float reduction, float dq, float ddq, float* matrix) {
    memset(matrix, 0, 6*sizeof(float));
    matrix[6] = reduction * reduction * ddq;
    matrix[7] = Sign(dq);
    matrix[8] = dq;
}
void DOF6Dynamic::MDHTrans(float alpha,float a,float d,float theta,float* R, float* R_T,float* P )
{
    float sa, ca, st, ct;
    sa = sinf_(alpha);
    ca = cosf_(alpha);
    st = sinf_(theta);
    ct = cosf_(theta);
    R[0] = ct;
    R[1] = -st;
    R[2] = 0;
    R[3] = st*ca;
    R[4] = ct*ca;
    R[5] = -sa;
    R[6] = st*sa;
    R[7] = ct*sa;
    R[8] = ca;

    R_T[0] = R[0];
    R_T[1] = R[3];
    R_T[2] = R[6];
    R_T[3] = R[1];
    R_T[4] = R[4];
    R_T[5] = R[7];
    R_T[6] = R[2];
    R_T[7] = R[5];
    R_T[8] = R[8];

    P[0] = a;
    P[1] = -d*sa;
    P[2] = d*ca;
}
void DOF6Dynamic::motion_para_clc(const float *R_inv,const float dq,const float ddq,const float* w_pre,const float* dw_pre,
        const float* dv_pre,const float* Po, const float* axis, float* w,float* dw,float* dv)
{
    float temp[5][3]; // 减少临时变量数量

    MatrixMultiplication(R_inv, w_pre, temp[0], 3, 3, 1);
    scalarMultiply(axis, dq, temp[1]);
    MatrixMultiplication(R_inv, dw_pre, temp[2], 3, 3, 1);
    crossProduct(temp[0], temp[1], temp[3]);
    scalarMultiply(axis, ddq, temp[4]);

    MatrixAddition2(temp[0], temp[1], w, 3, 1);
    MatrixAddition3(temp[2], temp[3],temp[4], dw, 3, 1);

    crossProduct(dw_pre, Po, temp[0]);
    crossProduct3(w_pre, w_pre, Po, temp[1]);
    MatrixAddition3(temp[0], temp[1], dv_pre,temp[2], 3, 1);
    MatrixMultiplication(R_inv, temp[2], dv, 3, 3, 1);
}
void DOF6Dynamic::getK(const float* vec, float* K)
{
    // for (int i = 0; i < 18; i++) {
    //     K[i] = 0.0f;
    // }
    memset(K, 0.0f, 18*sizeof(float));
    K[0] = vec[0];
    K[1] = vec[1];
    K[2] = vec[2];

    K[7] = vec[0];
    K[9] = vec[1];
    K[10] = vec[2];

    K[14] = vec[0];
    K[16] = vec[1];
    K[17] = vec[2];

}
void DOF6Dynamic::getS(const float* vec, float* S)
{
    S[0] =0.0f;
    S[1] = -vec[2];
    S[2] = vec[1];
    S[3] = vec[2];
    S[4] = 0.0f;
    S[5] = -vec[0];
    S[6] = -vec[1];
    S[7] = vec[0];
    S[8] = 0.0f;
}
void DOF6Dynamic::getHi(const float* wi,const float* dwi,const float* dvi,float* Hi)
{
    // 使用共用内存区减少分配

    // Step 1: 计算S矩阵
    getS(dwi, mem_Hi.S_dwi);
    getS(wi, mem_Hi.S_wi);
    MatrixMultiplication(mem_Hi.S_wi, mem_Hi.S_wi, mem_Hi.S_wi_sq, 3, 3, 3);
    // MatrixAddition2(temp1, temp3, temp4, 3, 3);
    for (int i = 0; i < 9; i++) {
        mem_Hi.sum[i] = mem_Hi.S_dwi[i] + mem_Hi.S_wi_sq[i];
    }
    // Step 4: 直接填充Hi（消除中间H数组）
    for (int i = 0; i < 3; i++) {
        // 前6列置零
        memset(&Hi[i*10], 0, 6*sizeof(float));

        // 中间3列填充sum
        memcpy(&Hi[i*10 + 6], &mem_Hi.sum[i*3], 3*sizeof(float));

        // 最后1列填充dvi
        Hi[i*10 + 9] = dvi[i];
    }
}
void DOF6Dynamic::getAi(const float* wi,const float* dwi,const float* dvi,float* Ai)
{
    getK(dwi, mem_Ai.K_dwi);
    getS(wi,mem_Ai.S_wi);
    getK(wi, mem_Ai.K_wi);
    MatrixMultiplication(mem_Ai.S_wi, mem_Ai.K_wi, mem_Ai.matmul, 3, 3, 6);
    MatrixAddition2(mem_Ai.K_dwi, mem_Ai.matmul, mem_Ai.sum, 3, 6);

    getS(dvi, mem_Ai.S_dvi);
    for (int i = 0; i < 3; i++) {
        // 前6列填充sum（使用memcpy确保对齐访问）
        memcpy(&Ai[i*10], &mem_Ai.sum[i*6], 6*sizeof(float));

        // 中间3列填充-S_dvi（手动展开循环）
        Ai[i*10 + 6] = -mem_Ai.S_dvi[i*3];
        Ai[i*10 + 7] = -mem_Ai.S_dvi[i*3 + 1];
        Ai[i*10 + 8] = -mem_Ai.S_dvi[i*3 + 2];

        // 最后1列置零
        Ai[i*10 + 9] = 0.0f;
    }
}
void DOF6Dynamic::get_Yf_Yn(int id,const float* R,const float* H,const float* A,const float* Yf_next,const float* Yn_next,const float* Po, float dq,float ddq,float reduction,float* Yf,float* Yn)
{
    const int c1 = (id-1)*13;
    const int c2 = c1 + 12; // 13-1
    const int data_cols = c2 - c1 - 2;
    // float n[9];
    GetAdditionMatrix(reduction, dq, ddq, n);
    memset(Yf_temp, 0, 234*sizeof(float));
    memset(Yn_temp, 0, 234*sizeof(float));
    // 2. 填充数据
    for (int i = 0; i < 3; i++) {
        // 填充H和A数据（ARM优化版memcpy）
        memcpy(&Yf_temp[i*78 + c1], &H[i*10], data_cols * sizeof(float));
        memcpy(&Yn_temp[i*78 + c1], &A[i*10], data_cols * sizeof(float));

        // 填充n数据（手动展开）
        Yn_temp[i*78 + c2-2] = n[i*3];
        Yn_temp[i*78 + c2-1] = n[i*3+1];
        Yn_temp[i*78 + c2]   = n[i*3+2];
    }
    float temp1[234];
    MatrixMultiplication(R, Yf_next, temp1, 3, 3, 78);
    MatrixAddition2(Yf_temp, temp1, Yf, 3, 78);
    float temp2[234];
    MatrixMultiplication(R, Yn_next, temp2, 3, 3, 78);
    float temp3[9];
    getS(Po, temp3);
    float temp4[234];
    MatrixMultiplication(temp3, temp1, temp4, 3,3, 78);
    MatrixAddition3(Yn_temp,temp2, temp4, Yn, 3, 78);

}
void DOF6Dynamic::Ymatrix_clc(const float* q,const float* dq,const float* ddq, float* Y)
{
    float (*H)[30] = m_H;        // 复用H数组
    float (*A)[30] = m_A;        // 复用A数组

    MDHTrans(0.0f             , 0.0f                  , dyarmConfig.L_BASE   , q[0], R[0], R_T[0], P[0]);
    MDHTrans(-90.0f/RAD_TO_DEG, dyarmConfig.D_BASE    , 0.0f                 , q[1], R[1], R_T[1], P[1]);
    MDHTrans(0.0f             , dyarmConfig.L_ARM     , 0.0f                 , q[2], R[2], R_T[2], P[2]);
    MDHTrans(-90.0f/RAD_TO_DEG, dyarmConfig.D_ELBOW   , dyarmConfig.L_FOREARM, q[3], R[3], R_T[3], P[3]);
    MDHTrans(90.0f/RAD_TO_DEG , 0.0f                  , 0.0f                 , q[4], R[4], R_T[4], P[4]);
    MDHTrans(-90.0f/RAD_TO_DEG, 0.0f                  , dyarmConfig.L_WRIST  , q[5], R[5], R_T[5], P[5]);

    float w11[3], w22[3], w33[3], w44[3], w55[3], w66[3];
    float dw11[3], dw22[3], dw33[3], dw44[3], dw55[3], dw66[3];
    float dv11[3], dv22[3], dv33[3], dv44[3], dv55[3], dv66[3];
    motion_para_clc(R_T[0],dq[0],ddq[0],w00,dw00,dv00,P[0],AXIS_Z,w11,dw11,dv11);
    motion_para_clc(R_T[1],dq[1],ddq[1],w11,dw11,dv11,P[1],AXIS_Z,w22,dw22,dv22);
    motion_para_clc(R_T[2],dq[2],ddq[2],w22,dw22,dv22,P[2],AXIS_Z,w33,dw33,dv33);
    motion_para_clc(R_T[3],dq[3],ddq[3],w33,dw33,dv33,P[3],AXIS_Z,w44,dw44,dv44);
    motion_para_clc(R_T[4],dq[4],ddq[4],w44,dw44,dv44,P[4],AXIS_Z,w55,dw55,dv55);
    motion_para_clc(R_T[5],dq[5],ddq[5],w55,dw55,dv55,P[5],AXIS_Z,w66,dw66,dv66);

    // float H_t1[30],H_t2[30],H_t3[30],H_t4[30],H_t5[30],H_t6[30];
    getHi(w11,dw11,dv11,H[0]);
    getHi(w22,dw22,dv22,H[1]);
    getHi(w33,dw33,dv33,H[2]);
    getHi(w44,dw44,dv44,H[3]);
    getHi(w55,dw55,dv55,H[4]);
    getHi(w66,dw66,dv66,H[5]);
    // memcpy(H_t1,H[0],30*sizeof(float));
    // memcpy(H_t2,H[1],30*sizeof(float));
    // memcpy(H_t3,H[2],30*sizeof(float));
    // memcpy(H_t4,H[3],30*sizeof(float));
    // memcpy(H_t5,H[4],30*sizeof(float));
    // memcpy(H_t6,H[5],30*sizeof(float));

    // float A_t1[30],A_t2[30],A_t3[30],A_t4[30],A_t5[30],A_t6[30];
    getAi(w11,dw11,dv11,A[0]);
    getAi(w22,dw22,dv22,A[1]);
    getAi(w33,dw33,dv33,A[2]);
    getAi(w44,dw44,dv44,A[3]);
    getAi(w55,dw55,dv55,A[4]);
    getAi(w66,dw66,dv66,A[5]);
    // memcpy(A_t1,A[0],30*sizeof(float));
    // memcpy(A_t2,A[1],30*sizeof(float));
    // memcpy(A_t3,A[2],30*sizeof(float));
    // memcpy(A_t4,A[3],30*sizeof(float));
    // memcpy(A_t5,A[4],30*sizeof(float));
    // memcpy(A_t6,A[5],30*sizeof(float));



    float n6[9];
    GetAdditionMatrix(REDUCTION[5],dq[5],ddq[5],n6);
    // float Yf6[180], Yf5[180], Yf4[180], Yf3[180], Yf2[180], Yf1[180];
    // float Yn6[180], Yn5[180], Yn4[180], Yn3[180], Yn2[180], Yn1[180];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 65; j++)
        {
            Yf[5][i*78+j] = 0.0f;
            Yn[5][i*78+j] = 0.0f;
        }
        for (int j = 0; j < 10; j++)
        {
            Yf[5][i*78+j+65] = H[5][10*i+j];
            Yn[5][i*78+j+65] = A[5][10*i+j];
        }
        for (int j = 0; j < 3; j++)
        {
            Yf[5][i*78+j+75] = 0.0f;
            Yn[5][i*78+j+75] = n6[3*i+j];
        }
    }

    get_Yf_Yn(5,R[5],H[4],A[4],Yf[5],Yn[5],P[5],dq[4],ddq[4],REDUCTION[4],Yf[4],Yn[4]);
    get_Yf_Yn(4,R[4],H[3],A[3],Yf[4],Yn[4],P[4],dq[3],ddq[3],REDUCTION[3],Yf[3],Yn[3]);
    get_Yf_Yn(3,R[3],H[2],A[2],Yf[3],Yn[3],P[3],dq[2],ddq[2],REDUCTION[2],Yf[2],Yn[2]);
    get_Yf_Yn(2,R[2],H[1],A[1],Yf[2],Yn[2],P[2],dq[1],ddq[1],REDUCTION[1],Yf[1],Yn[1]);
    get_Yf_Yn(1,R[1],H[0],A[0],Yf[1],Yn[1],P[1],dq[0],ddq[0],REDUCTION[0],Yf[0],Yn[0]);

    // memcpy(Yf6,Yf[5],180*sizeof(float));
    // memcpy(Yn6,Yn[5],180*sizeof(float));
    // memcpy(Yf5,Yf[4],180*sizeof(float));
    // memcpy(Yn5,Yn[4],180*sizeof(float));
    // memcpy(Yf4,Yf[3],180*sizeof(float));
    // memcpy(Yn4,Yn[3],180*sizeof(float));
    // memcpy(Yf3,Yf[2],180*sizeof(float));
    // memcpy(Yn3,Yn[2],180*sizeof(float));
    // memcpy(Yf2,Yf[1],180*sizeof(float));
    // memcpy(Yn2,Yn[1],180*sizeof(float));
    // memcpy(Yf1,Yf[0],180*sizeof(float));
    // memcpy(Yn1,Yn[0],180*sizeof(float));



    //只取z轴数据
    for (int i = 0; i < 6; i++) {
        const float* src = nullptr;
        switch(i) {
        case 0: src = Yn[0] + 78 * 2; break;
        case 1: src = Yn[1] + 78 * 2; break;
        case 2: src = Yn[2] + 78 * 2; break;
        case 3: src = Yn[3] + 78 * 2; break;
        case 4: src = Yn[4] + 78 * 2; break;
        case 5: src = Yn[5] + 78 * 2; break;
        }
        float* dst = Y + i * 78;
        memcpy(dst, src, 78 * sizeof(float));
    }
}
void DOF6Dynamic::Yr_clc(const float* q,const float* dq,const float* ddq,float* current)
{
    float* Y_temp = m_Y_temp;  // 直接使用成员变量
    float* Yr_temp = m_Yr_temp;
    const int SELECTED_COLS[52] ={ 6,12,13,
                                  14,15,16,18,19,20,21,25,26,
                                  27,28,29,31,32,33,34,37,38,39,
                                  40,41,42,44,45,46,47,50,51,52,
                                  53,54,55,57,58,59,60,63,64,65,
                                  66,67,68,70,71,72,73,76,77,78 };
    Ymatrix_clc(q,dq,ddq,Y_temp);
    for (int i = 0; i < 6; i++) {
        const float* src = Y_temp + i * 78;
        float* dst = Yr_temp + i * 52;
        for (int j = 0; j < 52; j++) {
            dst[j] = src[SELECTED_COLS[j] - 1];
        }
    }
    //给摩擦力相应位置置零
    const int ZERO_COLUMNS[5][10] = {
        {10,11,20,21,30,31,40,41,50,51},
        {20,21,30,31,40,41,50,51},
        {30,31,40,41,50,51},
        {40,41,50,51},
        {50,51}
    };
    const int ZERO_COUNTS[5] = {10, 8, 6, 4, 2};

    for (int row = 0; row < 5; row++) {
        float* row_ptr = Yr_temp + row * 52;
        for (int k = 0; k < ZERO_COUNTS[row]; k++) {
            row_ptr[ZERO_COLUMNS[row][k]] = 0.0f;
        }
    }
    // float Yr_t[312];
    // memcpy(Yr_t, Yr_temp, 312 * sizeof(float));
    MatrixMultiplication(Yr_temp,min_parameters,current,6,52,1);
}

