#include "mex.h"
#include "robot_ver2_kinematics.h"
#include <string.h>
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {
    char cmd[16];double *p,*o,q[6],t[16],J[36],sol[8][6],lo[6],hi[6],frames[8][16];size_t n;int i,j;
    if(nrhs<2||nlhs!=1||mxGetString(prhs[0],cmd,sizeof cmd)||!mxIsDouble(prhs[1])||mxIsComplex(prhs[1]))mexErrMsgIdAndTxt("testbench:MexInput","Use fk(q), jacobian(q), ik(T,seed).");
    p=mxGetPr(prhs[1]);
    if(!strcmp(cmd,"ik")){
        if(mxGetM(prhs[1])!=4||mxGetN(prhs[1])!=4||nrhs!=3||!mxIsDouble(prhs[2])||mxIsComplex(prhs[2])||mxGetNumberOfElements(prhs[2])!=6)mexErrMsgIdAndTxt("testbench:MexInput","T must be 4x4, seed six doubles.");
        for(i=0;i<4;i++)for(j=0;j<4;j++)t[4*i+j]=p[i+4*j];
        memcpy(q,mxGetPr(prhs[2]),sizeof q);for(i=0;i<6;i++){lo[i]=-3.141592653589793;hi[i]=-lo[i];}
        n=rb2_ik(t,q,lo,hi,sol);plhs[0]=mxCreateDoubleMatrix(n,6,mxREAL);o=mxGetPr(plhs[0]);
        for(i=0;i<(int)n;i++)for(j=0;j<6;j++)o[i+n*j]=sol[i][j];return;
    }
    if(mxGetNumberOfElements(prhs[1])!=6)mexErrMsgIdAndTxt("testbench:MexInput","Six joint angles required.");
    memcpy(q,p,sizeof q);
    if(!strcmp(cmd,"frames")){
        if(!rb2_body_transforms(q,frames))mexErrMsgIdAndTxt("testbench:InvalidQ","Invalid joints.");
        mwSize dims[3]={4,4,8};plhs[0]=mxCreateNumericArray(3,dims,mxDOUBLE_CLASS,mxREAL);o=mxGetPr(plhs[0]);
        for(int k=0;k<8;k++)for(i=0;i<4;i++)for(j=0;j<4;j++)o[k*16+i+4*j]=frames[k][4*i+j];
    }else if(!strcmp(cmd,"fk")){
        if(!rb2_fk(q,t))mexErrMsgIdAndTxt("testbench:InvalidQ","Invalid joints.");
        plhs[0]=mxCreateDoubleMatrix(4,4,mxREAL);o=mxGetPr(plhs[0]);for(i=0;i<4;i++)for(j=0;j<4;j++)o[i+4*j]=t[4*i+j];
    }else if(!strcmp(cmd,"jacobian")){
        if(!rb2_jacobian(q,J))mexErrMsgIdAndTxt("testbench:InvalidQ","Invalid joints.");
        plhs[0]=mxCreateDoubleMatrix(6,6,mxREAL);o=mxGetPr(plhs[0]);for(i=0;i<6;i++)for(j=0;j<6;j++)o[i+6*j]=J[6*i+j];
    }else mexErrMsgIdAndTxt("testbench:MexCommand","Unknown operation.");
}
