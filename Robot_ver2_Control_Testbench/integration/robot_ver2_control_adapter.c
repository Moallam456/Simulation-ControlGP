#include "robot_ver2_control_adapter.h"
#include "robot_ver2_kinematics.h"
#include "robot_ver2_geometry.h"
#include "robot_ver2_profile.h"
#include <math.h>
#include <string.h>
int rb2_is_robot(const RobotConfig *r) { return r&&r->identity.name&&!strcmp(r->identity.name,"robot_ver2"); }
void robot_config_init_robot_ver2(RobotConfig *r) {
    int i,j;if(!r)return;memset(r,0,sizeof *r);
    r->identity.name="robot_ver2";r->identity.manufacturer="Graduation Project";
    r->identity.role="CUSTOM ROBOT - SYNTHETIC ACTUATOR PROFILE";r->identity.dof=6;
    r->conventions.coordinateSystem="right-handed";r->conventions.lengthUnit="m";
    r->conventions.angleUnit="rad";r->conventions.angularVelocityUnit="rad/s";
    r->conventions.jointVectorShape="6x1";r->conventions.transformNotation="T_A_B";
    r->kinematics.convention="URDF - rb2 FK/Jacobian only, NOT DH arrays";
    r->frames.base="Base_Link";r->frames.flange="End_Effector";r->frames.tcp="tcp";
    r->configuration.homeDefined=true;
    for(i=0;i<6;i++){
        r->joints.names[i]="Joint";r->joints.type[i]="revolute";r->joints.order[i]=i+1;
        r->limits.qMin[i]=RB2_PROVISIONALLIMITS_RAD[2*i];r->limits.qMax[i]=RB2_PROVISIONALLIMITS_RAD[2*i+1];
        r->limits.qdMax[i]=RB2_DEMO_SPEED;r->configuration.home[i]=RB2_HOME_RAD[i];
    }
    for(i=0;i<4;i++)for(j=0;j<4;j++)r->tool.T_F_TCP[i][j]=RB2_TOOL[4*i+j];
    r->source.geometry="packaged corrected robot_ver2 URDF";
    r->source.performance="NOT EVALUATED";r->source.parameterStatus="PROVISIONAL - NOT COMMISSIONED";
}
bool rb2_control_ik(const RobotConfig *r,const double t[4][4],const double seed[6],double q[6],ADLSInfo *info) {
    double sol[8][6],fk[16],pe=0,re=0;size_t n;int i,j;
    if(info)memset(info,0,sizeof *info);
    if(!rb2_is_robot(r)||!t||!seed||!q)return false;
    n=rb2_ik(&t[0][0],seed,r->limits.qMin,r->limits.qMax,sol);
    if(!n)return false;
    memcpy(q,sol[0],6*sizeof(double));rb2_fk(q,fk);
    for(i=0;i<3;i++){double d=fk[4*i+3]-t[i][3];pe+=d*d;for(j=0;j<3;j++)re+=fk[4*i+j]*t[i][j];}
    if(info){info->converged=true;info->iterations=0;info->positionError=sqrt(pe);
        info->orientationError=acos(fmax(-1.,fmin(1.,(re-1)/2)));}
    return true;
}
