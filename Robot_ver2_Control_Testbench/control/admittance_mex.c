#include "mex.h"
#include "admittance_control.h"
#include <string.h>
/* Batch reference harness executes the ORIGINAL C admittance_step, not a port. */
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]){
    AdmittanceParams p;AdmittanceState s;double wrench[6],v[6],*out;size_t n,k;int j;
    if(nrhs!=7||nlhs!=1)mexErrMsgIdAndTxt("testbench:AdmittanceInput","W,M,D,dt,speed,acceleration,maxStep required.");
    for(j=0;j<7;j++)if(!mxIsDouble(prhs[j])||mxIsComplex(prhs[j]))mexErrMsgIdAndTxt("testbench:AdmittanceInput","Real doubles required.");
    n=mxGetM(prhs[0]);if(mxGetN(prhs[0])!=6)mexErrMsgIdAndTxt("testbench:AdmittanceInput","W must be N x 6.");
    for(j=1;j<=5;j++)if(mxGetNumberOfElements(prhs[j])!=(j==3?1:6))mexErrMsgIdAndTxt("testbench:AdmittanceInput","Parameter shape mismatch.");
    if(mxGetNumberOfElements(prhs[6])!=1)mexErrMsgIdAndTxt("testbench:AdmittanceInput","Scalar maximum step required.");
    memset(&p,0,sizeof p);memset(&s,0,sizeof s);
    for(j=0;j<6;j++){p.virtual_mass[j]=mxGetPr(prhs[1])[j];p.damping[j]=mxGetPr(prhs[2])[j];
        p.speed_limit[j]=mxGetPr(prhs[4])[j];p.acceleration_limit[j]=mxGetPr(prhs[5])[j];}
    p.maximum_step_s=mxGetScalar(prhs[6]);
    if(!admittance_enable(&s,&p))mexErrMsgIdAndTxt("testbench:AdmittanceConfig","Original C rejected configuration.");
    plhs[0]=mxCreateDoubleMatrix(n,6,mxREAL);out=mxGetPr(plhs[0]);
    for(k=0;k<n;k++){
        for(j=0;j<6;j++)wrench[j]=mxGetPr(prhs[0])[k+n*j];
        if(!admittance_step(&p,&s,wrench,mxGetScalar(prhs[3]),v))mexErrMsgIdAndTxt("testbench:AdmittanceFault","Original C controller rejected step.");
        for(j=0;j<6;j++)out[k+n*j]=v[j];
    }
}
