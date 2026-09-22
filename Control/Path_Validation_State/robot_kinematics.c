#include "robot_kinematics.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RK_PI (3.14159265358979323846F)
#define RK_EPS (1.0e-8F)

static bool finite_array(const float *x,size_t n){for(size_t i=0;i<n;++i)if(!isfinite(x[i]))return false;return true;}
static void identity4(float T[16]){memset(T,0,16U*sizeof(float));T[0]=T[5]=T[10]=T[15]=1.0F;}
static void multiply4(const float A[16],const float B[16],float C[16]){float t[16];for(uint8_t r=0;r<4;++r)for(uint8_t c=0;c<4;++c){float v=0;for(uint8_t k=0;k<4;++k)v+=A[4U*r+k]*B[4U*k+c];t[4U*r+c]=v;}memcpy(C,t,sizeof(t));}

static void dh_transform(float a,float d,float alpha,float theta,float A[16])
{
    float ct=cosf(theta),st=sinf(theta),ca=cosf(alpha),sa=sinf(alpha);
    A[0]=ct; A[1]=-st*ca; A[2]=st*sa; A[3]=a*ct;
    A[4]=st; A[5]=ct*ca; A[6]=-ct*sa; A[7]=a*st;
    A[8]=0; A[9]=sa; A[10]=ca; A[11]=d;
    A[12]=0; A[13]=0; A[14]=0; A[15]=1;
}

static bool model_valid(const RobotKinematicsModel *m)
{
    if(!m||!finite_array(m->a_m,6)||!finite_array(m->d_m,6)||!finite_array(m->alpha_rad,6)||
       !finite_array(m->theta_offset_rad,6)||!finite_array(m->joint_min_rad,6)||
       !finite_array(m->joint_max_rad,6)||!finite_array(m->T_flange_tcp,16))return false;
    for(uint8_t i=0;i<6;++i)if(m->joint_min_rad[i]>=m->joint_max_rad[i])return false;
    return true;
}

static void rotation_to_quaternion(const float R[9],float q[4])
{
    float trace=R[0]+R[4]+R[8],s;
    if(trace>0){s=2.0F*sqrtf(trace+1.0F);q[0]=0.25F*s;q[1]=(R[7]-R[5])/s;q[2]=(R[2]-R[6])/s;q[3]=(R[3]-R[1])/s;}
    else if(R[0]>R[4]&&R[0]>R[8]){s=2.0F*sqrtf(1.0F+R[0]-R[4]-R[8]);q[0]=(R[7]-R[5])/s;q[1]=0.25F*s;q[2]=(R[1]+R[3])/s;q[3]=(R[2]+R[6])/s;}
    else if(R[4]>R[8]){s=2.0F*sqrtf(1.0F+R[4]-R[0]-R[8]);q[0]=(R[2]-R[6])/s;q[1]=(R[1]+R[3])/s;q[2]=0.25F*s;q[3]=(R[5]+R[7])/s;}
    else{s=2.0F*sqrtf(1.0F+R[8]-R[0]-R[4]);q[0]=(R[3]-R[1])/s;q[1]=(R[2]+R[6])/s;q[2]=(R[5]+R[7])/s;q[3]=0.25F*s;}
    {float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);if(n>RK_EPS)for(uint8_t i=0;i<4;++i)q[i]/=n;}
}

static bool quaternion_to_rotation(const float qin[4],float R[9])
{
    float q[4],n;if(!finite_array(qin,4))return false;memcpy(q,qin,sizeof(q));
    n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);if(n<RK_EPS)return false;for(uint8_t i=0;i<4;++i)q[i]/=n;
    {float w=q[0],x=q[1],y=q[2],z=q[3];
    R[0]=1-2*(y*y+z*z);R[1]=2*(x*y-z*w);R[2]=2*(x*z+y*w);
    R[3]=2*(x*y+z*w);R[4]=1-2*(x*x+z*z);R[5]=2*(y*z-x*w);
    R[6]=2*(x*z-y*w);R[7]=2*(y*z+x*w);R[8]=1-2*(x*x+y*y);}
    return true;
}

static bool forward_matrix(const RobotKinematicsModel *m,const float q[6],float T[16])
{
    float A[16];if(!model_valid(m)||!q||!finite_array(q,6))return false;identity4(T);
    for(uint8_t i=0;i<6;++i){dh_transform(m->a_m[i],m->d_m[i],m->alpha_rad[i],q[i]+m->theta_offset_rad[i],A);multiply4(T,A,T);}
    multiply4(T,m->T_flange_tcp,T);return finite_array(T,16);
}

bool RobotKinematics_Forward(const RobotKinematicsModel *m,const float q[6],PvCartesianPose *pose)
{
    float T[16],R[9];if(!pose||!forward_matrix(m,q,T))return false;
    pose->position_m[0]=T[3];pose->position_m[1]=T[7];pose->position_m[2]=T[11];
    R[0]=T[0];R[1]=T[1];R[2]=T[2];R[3]=T[4];R[4]=T[5];R[5]=T[6];R[6]=T[8];R[7]=T[9];R[8]=T[10];
    rotation_to_quaternion(R,pose->orientation_quat);return finite_array(pose->position_m,3)&&finite_array(pose->orientation_quat,4);
}

bool RobotKinematics_Jacobian(const RobotKinematicsModel *m,const float q[6],float J[36])
{
    float T[16],A[16],Ttcp[16],origins[6][3],axes[6][3],p[3];
    if(!J||!model_valid(m)||!q||!finite_array(q,6))return false;
    identity4(T);
    for(uint8_t i=0;i<6;++i){origins[i][0]=T[3];origins[i][1]=T[7];origins[i][2]=T[11];axes[i][0]=T[2];axes[i][1]=T[6];axes[i][2]=T[10];dh_transform(m->a_m[i],m->d_m[i],m->alpha_rad[i],q[i]+m->theta_offset_rad[i],A);multiply4(T,A,T);}
    multiply4(T,m->T_flange_tcp,Ttcp);p[0]=Ttcp[3];p[1]=Ttcp[7];p[2]=Ttcp[11];
    for(uint8_t i=0;i<6;++i){float r[3]={p[0]-origins[i][0],p[1]-origins[i][1],p[2]-origins[i][2]};
        J[i]=axes[i][1]*r[2]-axes[i][2]*r[1];J[6+i]=axes[i][2]*r[0]-axes[i][0]*r[2];J[12+i]=axes[i][0]*r[1]-axes[i][1]*r[0];
        J[18+i]=axes[i][0];J[24+i]=axes[i][1];J[30+i]=axes[i][2];}
    return finite_array(J,36);
}

static float norm3(const float v[3]){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}

static void rotation_log_vector(const float R[9],float v[3])
{
    float c=0.5F*(R[0]+R[4]+R[8]-1.0F),theta;if(c>1)c=1;if(c<-1)c=-1;theta=acosf(c);
    if(theta<1e-8F){v[0]=v[1]=v[2]=0;return;}
    if(fabsf(RK_PI-theta)<1e-6F){float axis[3];
        if(1+R[8]>1e-8F){float k=1.0F/sqrtf(2*(1+R[8]));axis[0]=R[2]*k;axis[1]=R[5]*k;axis[2]=(1+R[8])*k;}
        else if(1+R[4]>1e-8F){float k=1.0F/sqrtf(2*(1+R[4]));axis[0]=R[1]*k;axis[1]=(1+R[4])*k;axis[2]=R[7]*k;}
        else{float k=1.0F/sqrtf(2*(1+R[0]));axis[0]=(1+R[0])*k;axis[1]=R[3]*k;axis[2]=R[6]*k;}
        {float n=norm3(axis);if(n>RK_EPS){v[0]=theta*axis[0]/n;v[1]=theta*axis[1]/n;v[2]=theta*axis[2]/n;}else v[0]=v[1]=v[2]=0;}return;}
    {float k=theta/(2*sinf(theta));v[0]=k*(R[7]-R[5]);v[1]=k*(R[2]-R[6]);v[2]=k*(R[3]-R[1]);}
}

static bool solve6(float A[36],float b[6],float x[6])
{
    for(uint8_t k=0;k<6;++k){uint8_t pivot=k;float best=fabsf(A[6U*k+k]);for(uint8_t r=k+1;r<6;++r)if(fabsf(A[6U*r+k])>best){best=fabsf(A[6U*r+k]);pivot=r;}if(best<1e-12F)return false;
        if(pivot!=k){for(uint8_t c=k;c<6;++c){float t=A[6U*k+c];A[6U*k+c]=A[6U*pivot+c];A[6U*pivot+c]=t;}{float t=b[k];b[k]=b[pivot];b[pivot]=t;}}
        for(uint8_t r=k+1;r<6;++r){float f=A[6U*r+k]/A[6U*k+k];A[6U*r+k]=0;for(uint8_t c=k+1;c<6;++c)A[6U*r+c]-=f*A[6U*k+c];b[r]-=f*b[k];}}
    for(int8_t r=5;r>=0;--r){float v=b[(uint8_t)r];for(uint8_t c=(uint8_t)r+1;c<6;++c)v-=A[6U*(uint8_t)r+c]*x[c];x[(uint8_t)r]=v/A[6U*(uint8_t)r+(uint8_t)r];}return finite_array(x,6);
}

static float sigma_min_6x6(const float J[36])
{
    float S[36];for(uint8_t r=0;r<6;++r)for(uint8_t c=0;c<6;++c){float v=0;for(uint8_t k=0;k<6;++k)v+=J[6U*k+r]*J[6U*k+c];S[6U*r+c]=v;}
    for(uint8_t sweep=0;sweep<32;++sweep){float largest=0;uint8_t p=0,q=1;for(uint8_t i=0;i<6;++i)for(uint8_t j=i+1;j<6;++j)if(fabsf(S[6U*i+j])>largest){largest=fabsf(S[6U*i+j]);p=i;q=j;}if(largest<1e-10F)break;
        {float app=S[6U*p+p],aqq=S[6U*q+q],apq=S[6U*p+q],phi=0.5F*atan2f(2*apq,aqq-app),c=cosf(phi),s=sinf(phi);
        for(uint8_t k=0;k<6;++k)if(k!=p&&k!=q){float skp=S[6U*k+p],skq=S[6U*k+q];S[6U*k+p]=S[6U*p+k]=c*skp-s*skq;S[6U*k+q]=S[6U*q+k]=s*skp+c*skq;}
        S[6U*p+p]=c*c*app-2*s*c*apq+s*s*aqq;S[6U*q+q]=s*s*app+2*s*c*apq+c*c*aqq;S[6U*p+q]=S[6U*q+p]=0;}}
    {float e=S[0];for(uint8_t i=1;i<6;++i)if(S[6U*i+i]<e)e=S[6U*i+i];return sqrtf(fmaxf(e,0.0F));}
}

bool RobotKinematics_AdlsIk(const RobotKinematicsModel *m,const RobotAdlsConfig *cfg,const PvCartesianPose *target,const float seed[6],float solution[6],PvIkDiagnostics *diag)
{
    float Rt[9],q[6];if(!model_valid(m)||!cfg||!target||!seed||!solution||!diag||!quaternion_to_rotation(target->orientation_quat,Rt)||!finite_array(target->position_m,3)||!finite_array(seed,6)||cfg->max_iterations==0||cfg->position_tolerance_m<=0||cfg->orientation_tolerance_rad<=0||cfg->step_size<=0||cfg->lambda_max<0||cfg->sigma_threshold<=0)return false;
    memset(diag,0,sizeof(*diag));for(uint8_t i=0;i<6;++i)q[i]=fminf(fmaxf(seed[i],m->joint_min_rad[i]),m->joint_max_rad[i]);
    for(uint16_t it=1;it<=cfg->max_iterations;++it){PvCartesianPose current;float Rc[9],Rerr[9],pe[3],oe[3],err[6],J[36],A[36],b[6],dq[6]={0},sigma,lambda;
        if(!RobotKinematics_Forward(m,q,&current)||!quaternion_to_rotation(current.orientation_quat,Rc))return false;
        for(uint8_t i=0;i<3;++i)pe[i]=target->position_m[i]-current.position_m[i];
        for(uint8_t r=0;r<3;++r)for(uint8_t c=0;c<3;++c){float v=0;for(uint8_t k=0;k<3;++k)v+=Rt[3U*r+k]*Rc[3U*c+k];Rerr[3U*r+c]=v;}
        rotation_log_vector(Rerr,oe);diag->position_error_m=norm3(pe);diag->orientation_error_rad=norm3(oe);diag->iterations=it;
        if(diag->position_error_m<=cfg->position_tolerance_m&&diag->orientation_error_rad<=cfg->orientation_tolerance_rad){diag->converged=true;memcpy(solution,q,6U*sizeof(float));return true;}
        if(!RobotKinematics_Jacobian(m,q,J))return false;
        sigma=sigma_min_6x6(J);diag->sigma_min=sigma;
        lambda=(sigma>cfg->sigma_threshold)?0.0F:cfg->lambda_max*sqrtf(fmaxf(0.0F,1.0F-(sigma/cfg->sigma_threshold)*(sigma/cfg->sigma_threshold)));
        err[0]=pe[0];err[1]=pe[1];err[2]=pe[2];err[3]=oe[0];err[4]=oe[1];err[5]=oe[2];
        for(uint8_t r=0;r<6;++r){b[r]=0;for(uint8_t k=0;k<6;++k)b[r]+=J[6U*k+r]*err[k];for(uint8_t c=0;c<6;++c){float v=0;for(uint8_t k=0;k<6;++k)v+=J[6U*k+r]*J[6U*k+c];A[6U*r+c]=v+(r==c?lambda*lambda:0);}}
        if(!solve6(A,b,dq))return false;
        for(uint8_t i=0;i<6;++i)q[i]=fminf(fmaxf(q[i]+cfg->step_size*dq[i],m->joint_min_rad[i]),m->joint_max_rad[i]);
    }
    memcpy(solution,q,6U*sizeof(float));return true;
}

bool RobotKinematics_PathIk(const PvCartesianPose *target,const float seed[PV_DOF],float solution[PV_DOF],PvIkDiagnostics *diag,void *user)
{
    RobotKinematicsContext *c=(RobotKinematicsContext *)user;if(!c)return false;
    if(c->selected_method==ROBOT_IK_ANALYTICAL)return c->analytical_solver&&c->analytical_solver(&c->model,target,seed,solution,diag,c->analytical_context);
    return RobotKinematics_AdlsIk(&c->model,&c->adls,target,seed,solution,diag);
}

bool RobotKinematics_PathFk(const float q[PV_DOF],PvCartesianPose *pose,void *user){RobotKinematicsContext *c=(RobotKinematicsContext *)user;return c&&RobotKinematics_Forward(&c->model,q,pose);}

PathValidationServices RobotKinematics_MakePathValidationServices(RobotKinematicsContext *c,PvCollisionCheckFn collision)
{
    PathValidationServices s={RobotKinematics_PathIk,RobotKinematics_PathFk,collision,c};return s;
}
