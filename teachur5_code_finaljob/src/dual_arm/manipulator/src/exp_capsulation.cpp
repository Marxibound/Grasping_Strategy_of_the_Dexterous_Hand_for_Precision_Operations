#include "manipulator/robot_arm.h"
namespace Robot_capsulation
{
    std::string replaceChar(const std::string& str, char oldChar, char newChar) {
        std::string result = str;
        for (char& c : result) {
            if (c == oldChar) {
                c = newChar;
            }
        }
        return result;
    }
    std::string Robot_operation::obstacle_avoidance_movement(const std::vector<std::any>& args){
        std::string target_point_name = std::any_cast<std::string>(args[0]);
        move_group_ptr->setPlanningPipelineId("ompl");
        move_group_ptr->setPlannerId("RRTConnect");
        std::vector<double>target_joints;
        if(!Record_tool::get_joint(target_point_name+"_inverse", joints_, target_joints))
            return "error";
        if(!arm_robot_ptr->move_targetJoints(target_joints,true, true))
            return "error";
        printf("成功运动至目标点%s\n",target_point_name.c_str());
        return "success";
    }
    // 类似fakecom执行带参函数
    std::string Robot_operation::move_test(const std::vector<std::any>& args){
        char id = std::any_cast<double>(args[0])+'0';

        std::string result;
        result=Analysis_Command("A S PTP J 0.2 0.2 reset",false);
        if(result!="success")return result;
        printf("id=%d\n",id);
        result=Analysis_Command(replaceChar("A S PTP J 0.2 0.2 block@_high",'@',id),false);
        if(result!="success")return result;
        return "success";
    }
    // 获取关节较并执行关节运动
    std::string Robot_operation::move_joints_test(void)
    {
        std::vector<double> joint_inverse;
        if(!Record_tool::get_joint("reset_inverse", joints_, joint_inverse))
            return "error";
        if(!arm_robot_ptr->move_targetJoints(joint_inverse,true,true))
            return "error";
        return "success";
    }

}