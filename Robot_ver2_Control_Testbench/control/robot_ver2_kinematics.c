#include "robot_ver2_kinematics.h"
#include "robot_ver2_geometry.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846
static double clamp(double x) { return fmax(-1.,fmin(1.,x)); }
static double wrap(double x) { return atan2(sin(x),cos(x)); }
static void ident(double *t) { int i; memset(t,0,16*sizeof(double)); for(i=0;i<4;i++)t[5*i]=1; }
static void mul(const double *a,const double *b,double *c) {
    double d[16]; int i,j,k;
    for(i=0;i<4;i++)for(j=0;j<4;j++){d[4*i+j]=0;for(k=0;k<4;k++)d[4*i+j]+=a[4*i+k]*b[4*k+j];}
    memcpy(c,d,sizeof d);
}
static int validq(const double *q) { int i;if(!q)return 0;for(i=0;i<6;i++)if(!isfinite(q[i]))return 0;return 1; }
static void axisrot(const double *u,double q,double *t) {
    double c=cos(q),s=sin(q),v=1-c; int i,j;ident(t);
    for(i=0;i<3;i++)for(j=0;j<3;j++)t[4*i+j]=u[i]*u[j]*v+(i==j?c:0);
    t[1]-=u[2]*s;t[2]+=u[1]*s;t[4]+=u[2]*s;t[6]-=u[0]*s;t[8]-=u[1]*s;t[9]+=u[0]*s;
}
static int chain(const double *q,double *t,double origins[6][3],double axes[6][3]) {
    int i,j,k;double rot[16];if(!validq(q)||!t)return 0;ident(t);
    for(i=0;i<6;i++){
        mul(t,RB2_ORIGINS[i],t);
        if(origins)for(j=0;j<3;j++){
            origins[i][j]=t[4*j+3];axes[i][j]=0;
            for(k=0;k<3;k++)axes[i][j]+=t[4*j+k]*RB2_AXES[i][k];
        }
        axisrot(RB2_AXES[i],q[i],rot);mul(t,rot,t);
    }
    mul(t,RB2_TOOL,t);return 1;
}
int rb2_fk(const double q[6],double t[16]) { return chain(q,t,NULL,NULL); }
int rb2_body_transforms(const double q[6],double out[8][16]) {
    double t[16],rot[16];int i;
    if(!validq(q)||!out)return 0;ident(t);memcpy(out[0],t,sizeof t);
    for(i=0;i<6;i++){
        mul(t,RB2_ORIGINS[i],t);axisrot(RB2_AXES[i],q[i],rot);mul(t,rot,t);memcpy(out[i+1],t,sizeof t);
    }
    mul(t,RB2_TOOL,out[7]);return 1;
}
int rb2_jacobian(const double q[6],double J[36]) {
    double t[16],p[6][3],a[6][3],v[3];int i,j;if(!J||!chain(q,t,p,a))return 0;
    for(i=0;i<6;i++){
        for(j=0;j<3;j++){v[j]=t[4*j+3]-p[i][j];J[(j+3)*6+i]=a[i][j];}
        J[i]=a[i][1]*v[2]-a[i][2]*v[1];
        J[6+i]=a[i][2]*v[0]-a[i][0]*v[2];J[12+i]=a[i][0]*v[1]-a[i][1]*v[0];
    }return 1;
}
static int validpose(const double *t) {
    int i,j,k;double d;if(!t)return 0;for(i=0;i<16;i++)if(!isfinite(t[i]))return 0;
    if(fabs(t[12])+fabs(t[13])+fabs(t[14])+fabs(t[15]-1)>1e-10)return 0;
    for(i=0;i<3;i++)for(j=0;j<3;j++){
        d=0;for(k=0;k<3;k++)d+=t[4*k+i]*t[4*k+j];if(fabs(d-(i==j))>1e-8)return 0;
    }
    d=t[0]*(t[5]*t[10]-t[6]*t[9])-t[1]*(t[4]*t[10]-t[6]*t[8])+t[2]*(t[4]*t[9]-t[5]*t[8]);
    return d>0;
}
size_t rb2_ik(const double target[16],const double seed[6],const double lo[6],const double hi[6],double out[8][6]) {
    const double d1=.2,yP=-.2425,zA=.132,a2=.589954679438418,a3x=.404499960921119,a3y=-4.53236012523117e-5;
    const double a3=hypot(a3x,a3y),offset=atan2(-a3y,a3x),d5=.0555,yW=-.0554999969603261-.1195;
    double inv[16],T[16],A,B,rn,phi,delta,q1,s1,c1,s5,c5m,c5,P,Q,U,X,Y,s3,c3m,c3,q[6],r2,c2,s2,th,c4,s4;
    double rzth[16],rx[16],rz1[16],rot[16],R36[16],fk[16],trace,pe,diff,cost[8],tmp[6],dc;
    int b,w,e,i,j,k,duplicate,ok;size_t count=0,n;
    if(!validpose(target)||!validq(seed)||!lo||!hi||!out)return 0;
    for(i=0;i<6;i++)if(!isfinite(lo[i])||!isfinite(hi[i])||lo[i]>=hi[i])return 0;
    ident(inv);for(i=0;i<3;i++)for(j=0;j<3;j++)inv[4*i+j]=RB2_TOOL[4*j+i];
    for(i=0;i<3;i++){inv[4*i+3]=0;for(j=0;j<3;j++)inv[4*i+3]-=inv[4*i+j]*RB2_TOOL[4*j+3];}
    mul(target,inv,T);A=T[3]-d5*T[2];B=T[7]-d5*T[6];rn=hypot(A,B);
    if(rn<fabs(yP))return 0;
    phi=atan2(B,A);delta=asin(clamp(yP/rn));
    for(b=0;b<2;b++){
        q1=b?PI-delta-phi:delta-phi;s1=sin(q1);c1=cos(q1);s5=-(s1*T[2]+c1*T[6]);
        if(fabs(s5)>1+1e-10)continue;
        s5=clamp(s5);c5m=sqrt(fmax(0,1-s5*s5));if(c5m<1e-10)continue;
        for(w=0;w<2;w++){
            c5=(w?-1:1)*c5m;P=c1*T[2]-s1*T[6];Q=T[10];U=c1*T[3]-s1*T[7];
            X=U-d5*P+yW*Q/c5;Y=T[11]-d1-d5*Q-yW*P/c5-zA;
            s3=(X*X+Y*Y-a2*a2-a3*a3)/(2*a2*a3);if(fabs(s3)>1+1e-10)continue;
            s3=clamp(s3);c3m=sqrt(fmax(0,1-s3*s3));
            for(e=0;e<2;e++){
                c3=(e?-1:1)*c3m;q[2]=atan2(s3,c3)-offset;r2=X*X+Y*Y;if(r2<1e-12)continue;
                c2=(-a3*c3*X+(a2+a3*s3)*Y)/r2;s2=(-a3*c3*Y-(a2+a3*s3)*X)/r2;
                q[0]=q1;q[1]=atan2(s2,c2);th=q[2]-q[1];c4=(-P*cos(th)+Q*sin(th))/c5;s4=(-P*sin(th)-Q*cos(th))/c5;
                q[3]=atan2(s4,c4);q[4]=atan2(s5,c5);
                ident(rzth);rzth[0]=cos(th);rzth[1]=sin(th);rzth[4]=-sin(th);rzth[5]=cos(th);
                ident(rx);rx[5]=0;rx[6]=-1;rx[9]=1;rx[10]=0;
                ident(rz1);rz1[0]=c1;rz1[1]=-s1;rz1[4]=s1;rz1[5]=c1;
                memcpy(rot,T,sizeof rot);rot[3]=rot[7]=rot[11]=0;
                mul(rzth,rx,R36);mul(R36,rz1,R36);mul(R36,rot,R36);
                q[5]=atan2(-R36[9]/c5,R36[8]/c5);ok=1;
                for(i=0;i<6;i++){q[i]=wrap(q[i]);if(q[i]<lo[i]-1e-9||q[i]>hi[i]+1e-9)ok=0;}if(!ok)continue;
                rb2_fk(q,fk);pe=0;trace=0;for(i=0;i<3;i++){
                    diff=fk[4*i+3]-target[4*i+3];pe+=diff*diff;
                    for(j=0;j<3;j++)trace+=target[4*i+j]*fk[4*i+j];
                }if(sqrt(pe)>1e-5||acos(clamp((trace-1)/2))>1e-4)continue;
                duplicate=0;for(n=0;n<count;n++){diff=0;for(i=0;i<6;i++){dc=wrap(q[i]-out[n][i]);diff+=dc*dc;}if(diff<1e-12)duplicate=1;}
                if(duplicate)continue;
                memcpy(out[count],q,sizeof q);cost[count]=0;
                for(i=0;i<6;i++){diff=wrap(q[i]-seed[i]);cost[count]+=diff*diff;}count++;
            }
        }
    }
    for(i=0;i<(int)count;i++)for(k=i+1;k<(int)count;k++)if(cost[k]<cost[i]){
        dc=cost[i];cost[i]=cost[k];cost[k]=dc;memcpy(tmp,out[i],sizeof tmp);memcpy(out[i],out[k],sizeof tmp);memcpy(out[k],tmp,sizeof tmp);
    }return count;
}
