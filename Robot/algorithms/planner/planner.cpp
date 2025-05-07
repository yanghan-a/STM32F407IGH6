//
// Created by 15873 on 2024/12/27.
//

#include "planner.h"

#include <cstdio>

void planner::Init()
{
    line.Init();
}

void planner::Line::Init()
{

}

void planner::Line::NewTask(Pose6D _current_pose)
{

    currentnumberofpoints = 0;

    catesian_current_pose.position = _current_pose.position/1000.0f;
    catesian_current_pose.orientation = _current_pose.orientation/RAD_TO_DEG;

    quaternion_current_orientation = Quaternion::fromEuler(catesian_current_pose.orientation.x, catesian_current_pose.orientation.y, catesian_current_pose.orientation.z);

}

void planner::Line::CalcSoftGoal(int32_t _numberofpoints, Pose6D _goal_pose)
{
    //|| (catesian_final_goal_pose.position != _goal_pose.position) || (catesian_final_goal_pose.orientation != _goal_pose.orientation)
    if(numberofpoints != _numberofpoints || (record_goal_pose.position != _goal_pose.position) || (record_goal_pose.orientation != _goal_pose.orientation))
    {
        record_goal_pose.position = _goal_pose.position;
        record_goal_pose.orientation = _goal_pose.orientation;
        numberofpoints = _numberofpoints;
        currentnumberofpoints = 0;
        catesian_final_goal_pose.position = _goal_pose.position/1000.0f;//转为m
        catesian_final_goal_pose.orientation = _goal_pose.orientation/RAD_TO_DEG;//转为弧度

        quaternion_final_goal_orientation = Quaternion::fromEuler(catesian_final_goal_pose.orientation.x, catesian_final_goal_pose.orientation.y, catesian_final_goal_pose.orientation.z);

    }
    // printf("numberofpoints:%d,%d\n",numberofpoints,currentnumberofpoints);
    // printf("%.4f\n",catesian_final_goal_pose.position.x);
    if(numberofpoints != 0)
    {
        lamda = (float)currentnumberofpoints / (float)numberofpoints;
        quaternion_temp_goal_orientation = Quaternion::slerp(quaternion_current_orientation,quaternion_final_goal_orientation,lamda);


        catesian_temp_goal_pose.orientation = quaternion_temp_goal_orientation.toEuler();

        // temp_goal_position_last = catesian_temp_goal_pose.orientation;//更新temp_goal_position_last，用于下次计算

        catesian_temp_goal_pose.position = Vector3D::interpolate(catesian_current_pose.position, catesian_final_goal_pose.position, lamda);

        if(currentnumberofpoints == numberofpoints)
        {
            catesian_temp_goal_pose.position = catesian_final_goal_pose.position;
            catesian_temp_goal_pose.orientation = catesian_final_goal_pose.orientation;

            catesian_current_pose.position = catesian_temp_goal_pose.position;
            catesian_current_pose.orientation = catesian_temp_goal_pose.orientation;

            quaternion_current_orientation = quaternion_final_goal_orientation;
        }else
        {
            currentnumberofpoints++;
        }
    }else
    {
        catesian_temp_goal_pose.position = catesian_current_pose.position;
        catesian_temp_goal_pose.orientation = catesian_current_pose.orientation;
    }


}



