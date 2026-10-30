#ifndef RB2_CONTROL_ADAPTER_H
#define RB2_CONTROL_ADAPTER_H
#include "ControlCore/Config/robot_config.h"
#include "ControlCore/Kinematics/adls_ik.h"
#include "robot_ver2_kinematics.h"
void robot_config_init_robot_ver2(RobotConfig *robot);
int rb2_is_robot(const RobotConfig *robot);
bool rb2_control_ik(const RobotConfig *robot,const double target[4][4],
                    const double seed[6],double solution[6],ADLSInfo *info);
#endif
