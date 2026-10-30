#include "mex.h"
#include "robot_ver2_control_adapter.h"
#include "robot_ver2_kinematics.h"
#include "HandGuiding/admittance_control.h"
#include "HandGuiding/cartesian_joint_mapper.h"
#include <string.h>
#include <math.h>
static RobotConfig robot;
static AdmittanceParams params;
static AdmittanceState admittance;
static CartesianJointMapperConfig config;
static CartesianJointMapperState mapper;
static int ready;
void control_jacobian(const RobotConfig *r,const double q[6],double J[6][6]) {
    (void)r;if(!rb2_jacobian(q,&J[0][0]))memset(J,0,36*sizeof(double));
}
static const double *vector(const mxArray *a,size_t n){
    if(!mxIsDouble(a)||mxIsComplex(a)||mxGetNumberOfElements(a)!=n)
        mexErrMsgIdAndTxt("testbench:CControllerInput","Incorrect real-double input shape.");
    const double *v=mxGetPr(a);for(size_t i=0;i<n;i++)if(!isfinite(v[i]))mexErrMsgIdAndTxt("testbench:CControllerInput","Nonfinite input.");return v;
}
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]){
    char action[16];if(nrhs<1||mxGetString(prhs[0],action,sizeof action))mexErrMsgIdAndTxt("testbench:CControllerInput","Action required.");
    if(!strcmp(action,"reset")){
        if(nrhs!=9||nlhs)mexErrMsgIdAndTxt("testbench:CControllerInput","reset(home,M,D,speed,accel,jointSpeed,sigma,dt)");
        const double *q=vector(prhs[1],6),*m=vector(prhs[2],6),*d=vector(prhs[3],6);
        const double *speed=vector(prhs[4],6),*accel=vector(prhs[5],6),*js=vector(prhs[6],6);
        double sigma=*vector(prhs[7],1),dt=*vector(prhs[8],1);
        robot_config_init_robot_ver2(&robot);memset(&params,0,sizeof params);memset(&config,0,sizeof config);
        for(int i=0;i<6;i++){
            params.virtual_mass[i]=m[i];params.damping[i]=d[i];params.speed_limit[i]=speed[i];params.acceleration_limit[i]=accel[i];
            robot.limits.qdMax[i]=js[i];config.joint_acceleration_limit[i]=2.;config.tracking_error_limit[i]=.2;
        }
        params.maximum_step_s=2*dt;config.maximum_step_s=2*dt;config.damping_lambda=.02;
        config.minimum_sigma=sigma;config.model_verified=true; /* synthetic software fixture ONLY */
        ready=admittance_enable(&admittance,&params)&&cartesian_joint_mapper_enable(&mapper,&config,&robot,q);
        if(!ready)mexErrMsgIdAndTxt("testbench:CControllerConfig","Original C controller/mapper refused setup.");return;
    }
    if(strcmp(action,"step")||nrhs!=4||nlhs!=1||!ready)mexErrMsgIdAndTxt("testbench:CControllerInput","Reset, then step(q,wrench,dt).");
    const double *q=vector(prhs[1],6),*w=vector(prhs[2],6);double dt=*vector(prhs[3],1),twist[6];
    CartesianJointMapperOutput output;
    if(!admittance_step(&params,&admittance,w,dt,twist)||!cartesian_joint_mapper_step(&mapper,&config,&robot,q,twist,dt,&output)||!output.valid){
        ready=0;mexErrMsgIdAndTxt("testbench:CControllerFault","Original C admittance/mapper rejected input, timing, limits or tracking.");
    }
    plhs[0]=mxCreateDoubleMatrix(1,6,mxREAL);memcpy(mxGetPr(plhs[0]),output.position_ref,6*sizeof(double));
}
