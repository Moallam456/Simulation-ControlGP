#include "robot_ver2_kinematics.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static unsigned random_state=17;
static double rnd(void){random_state=random_state*1664525U+1013904223U;return (random_state/4294967296.0)*6.0-3.0;}
int main(void){
    double q[6],t[16],J[36],sol[8][6],fk[16],lo[6],hi[6],shifted[6],perturbed[16];
    unsigned successes=0;int sample,i,j;size_t n,k;
    for(i=0;i<6;i++){lo[i]=-3.141592653589793;hi[i]=-lo[i];}
    for(sample=0;sample<1000;sample++){
        for(i=0;i<6;i++)q[i]=rnd();assert(rb2_fk(q,t));assert(rb2_jacobian(q,J));
        for(i=0;i<6;i++){
            for(j=0;j<6;j++)shifted[j]=q[j];shifted[i]+=1e-7;assert(rb2_fk(shifted,perturbed));
            for(j=0;j<3;j++)assert(fabs((perturbed[4*j+3]-t[4*j+3])/1e-7-J[6*j+i])<1e-6);
        }
        n=rb2_ik(t,q,lo,hi,sol);assert(n<=8);
        if(n)successes++;
        for(k=0;k<n;k++){
            assert(rb2_fk(sol[k],fk));for(i=0;i<3;i++)assert(fabs(fk[4*i+3]-t[4*i+3])<1e-5);
        }
    }
    printf("%u/1000 random FK-to-IK round trips; exact singular branches documented separately.\n",successes);
    assert(successes>=995);t[3]=100;assert(rb2_ik(t,q,lo,hi,sol)==0);
    q[0]=NAN;assert(!rb2_fk(q,t));puts("Kinematics portable tests passed.");return 0;
}
