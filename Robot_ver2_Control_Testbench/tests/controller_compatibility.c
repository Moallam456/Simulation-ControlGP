#include "robot_ver2_control_adapter.h"
#include "ControlCore/Kinematics/control_fk.h"
#include "ControlCore/Kinematics/control_jacobian.h"
#include "robot_ver2_kinematics.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
    RobotConfig robot;double q[6],T[4][4],J[6][6],direct[16],answer[6];ADLSInfo info;
    robot_config_init_robot_ver2(&robot);assert(rb2_is_robot(&robot));assert(robot.configuration.homeDefined);
    for(int i=0;i<6;i++)q[i]=robot.configuration.home[i];
    control_fk(&robot,q,T);assert(rb2_fk(q,direct));
    for(int i=0;i<16;i++)assert(fabs((&T[0][0])[i]-direct[i])<1e-12);
    control_jacobian(&robot,q,J);
    assert(adls_ik(&robot,T,q,NULL,answer,&info));assert(info.converged&&info.iterations==0);
    for(int i=0;i<6;i++)assert(fabs(answer[i]-q[i])<1e-4);
    robot_config_init_ur5(&robot);assert(!rb2_is_robot(&robot));
    puts("Robot-specific API dispatch passed; legacy UR5 configuration retained.");return 0;
}
