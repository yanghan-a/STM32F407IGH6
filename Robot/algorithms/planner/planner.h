//
// Created by 15873 on 2024/12/27.
//

#ifndef PLANNER_H
#define PLANNER_H
#include <cstdint>
#include "algorithms/kinematic/6dof_kinematic.h"
#include <cmath>
#include <arm_math.h>
#include <cstdio>
#include <vector>

static const float RAD_TO_DEG = 57.29577951308232f;

class planner
{
private:

public:
    struct Vector3D
    {
        float x, y, z;

        Vector3D() : x(0), y(0), z(0) {}
        Vector3D(float x, float y, float z) : x(x), y(y), z(z) {}

        static Vector3D interpolate(const Vector3D &p1, const Vector3D &p2, float t) {
            return  p1*(1 - t) +  p2*t;
        }

        // 向量加法
        Vector3D operator+(const Vector3D &other) const {
            return {x + other.x, y + other.y, z + other.z};
        }
        // 向量除法
        Vector3D operator/(float scalar) const {
            return {x / scalar, y / scalar, z / scalar};
        }
        // 向量乘法
        Vector3D operator*(float scalar) const {
            return {x * scalar, y * scalar, z * scalar};
        }
        // 向量是否相等
        bool operator!=(const Vector3D& other) const {
            const float epsilon = 1e-6f;  // 容忍误差
            return (std::abs(x - other.x) > epsilon) ||
                   (std::abs(y - other.y) > epsilon) ||
                   (std::abs(z - other.z) > epsilon);
        }
    };
    // 四元数表示
    struct Quaternion
    {
        float w, x, y, z;

        Quaternion() : w(1), x(0), y(0), z(0) {}
        Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

        // 从欧拉角转换到四元数
        static Quaternion fromEuler(float roll, float pitch, float yaw) {
            float c1 = cosf(yaw / 2);
            float s1 = sinf(yaw / 2);
            float c2 = cosf(pitch / 2);
            float s2 = sinf(pitch / 2);
            float c3 = cosf(roll / 2);
            float s3 = sinf(roll / 2);

            Quaternion q;
            q.w = c1 * c2 * c3 + s1 * s2 * s3;
            q.x = c1 * c2 * s3 - s1 * s2 * c3;
            q.y = s1 * c2 * s3 + c1 * s2 * c3;
            q.z = s1 * c2 * c3 - c1 * s2 * s3;

            return q;
        }

        // 四元数插值（SLERP）
        static Quaternion slerp( Quaternion &q1,  Quaternion &q2, float t) {
            float dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
            Quaternion result;
            const float THRESHOLD = 0.9995f;
            if (dot < -0.00001f)
            {
                dot = -dot;
                q2.w = -q2.w;
                q2.x = -q2.x;
                q2.y = -q2.y;
                q2.z = -q2.z;
            }
            if (dot > THRESHOLD) {
                result= q1*t + q2*(1 - t);
                result.normalize();
                return result;
            }
            float sintheta = sqrtf((1 - dot) * (1 + dot));
            float theta = atan2f(sintheta, dot);

            result = q1 * (sinf((1.0f - t)*theta)/sintheta) + q2 * (sinf(t*theta)/sintheta);
            return result;
        }

        // 四元数归一化
        void normalize() {
            float norm = sqrt(w * w + x * x + y * y + z * z);
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        }

        // 重载运算符
        Quaternion operator+(const Quaternion &other) const {
            return {w + other.w, x + other.x, y + other.y, z + other.z};
        }

        Quaternion operator-(const Quaternion &other) const {
            return {w - other.w, x - other.x, y - other.y, z - other.z};
        }

        Quaternion operator*(float scalar) const {
            return {w * scalar, x * scalar, y * scalar, z * scalar};
        }
        // 将四元数转换为欧拉角
         Vector3D toEuler() const {
            float roll, pitch, yaw;
            float epsilon = 0.000001f;;  // 容忍误差
            float test = w * y - x * z;

            if(fabs(fabs(test)-0.5f)<epsilon)
            {
                if(test>0)
                {
                    yaw = -2 * atan2f(x, w);
                    pitch = M_PI_2;
                }else
                {
                    yaw = 2 * atan2f(x, w);
                    pitch = -M_PI_2;
                }
                roll = 0;
                // if(fabs(temp1)<epsilon || fabs(temp2)<epsilon)
                // {
                //
                //     roll = orientation_last.x;
                //     yaw = orientation_last.z;
                // }
                // else
                // {
                //     roll = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                //     yaw = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
                // }
                // printf("111\n");
            }else
            {
                roll = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                pitch = asinf(2 * (w * y - z * x));
                yaw = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
                // printf("222\n");
            }

            return {roll, pitch, yaw};
        }
    };



    struct Pose6D
    {
        Vector3D position = {};
        Vector3D orientation = {};
        Pose6D() : position(0, 0, 0), orientation(0, 0, 0) {}
        Pose6D(Vector3D _position, Vector3D _orientation) : position(_position), orientation(_orientation) {}

    };

    struct Joint6D
    {
        Joint6D() = default;
        Joint6D(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6) :
            j1(_j1), j2(_j2), j3(_j3), j4(_j4), j5(_j5), j6(_j6)
        {}
        float j1, j2, j3, j4, j5, j6;
    };
    planner() = default;

    const int32_t CONTROL_FREQUENCY = 200; // control frequency in Hz
    const int32_t  CONTROL_PERIOD = 1000000 / CONTROL_FREQUENCY; // uS

    class Line
    {
    public:
        explicit Line(planner* _context) :
            context(_context)
        {
        }

        void Init();
        void NewTask(Pose6D _current_pose);
        void CalcSoftGoal(int32_t _numberofpoints, Pose6D _goal_pose);
        Pose6D catesian_temp_goal_pose = {};
    private:
        planner* context;
        float lamda = 0.0f;
        int32_t numberofpoints = 0;
        int32_t currentnumberofpoints = 0;

        Pose6D record_goal_pose = {};
        Pose6D catesian_final_goal_pose = {};
        Pose6D catesian_current_pose = {};

        Quaternion quaternion_final_goal_orientation = {};//最终目标方位角
        Quaternion quaternion_current_orientation = {};//任务开始方位角


        Quaternion quaternion_temp_goal_orientation = {};//插值方位角


        Joint6D temp_goal_angle = {0, 0, 0, 0, 0, 0};
    };
    Line line = Line(this);

    void Init();
};



#endif //PLANNER_H
