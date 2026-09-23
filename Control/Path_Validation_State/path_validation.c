#include "path_validation.h"
#include "path_validation_debug.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PV_EPS (1.0e-7F)
#define PV_PI  (3.14159265358979323846F)
#define PV_CRC32_POLYNOMIAL (0xEDB88320UL)

typedef struct {
    float duration[7];
    float jerk[7];
    float start_time[7];
    float start_s[7];
    float start_v[7];
    float start_a[7];
    float total_time;
} PvSCurve;

static uint32_t crc32_update(uint32_t crc,const void *input,size_t length)
{
    const uint8_t *data=(const uint8_t *)input;
    for(size_t i=0;i<length;++i){crc^=data[i];for(uint8_t bit=0;bit<8;++bit){uint32_t mask=(uint32_t)-(int32_t)(crc&1U);crc=(crc>>1U)^(PV_CRC32_POLYNOMIAL&mask);}}
    return crc;
}

uint32_t PathValidation_CalculateDraftCrc(const TaughtProgram *p)
{
    uint32_t crc=0xFFFFFFFFUL;
    if(!p)return 0U;
#define DCRC(x) do{crc=crc32_update(crc,&(x),sizeof(x));}while(0)
    DCRC(p->program_id);DCRC(p->draft_revision);DCRC(p->segment_count);DCRC(p->global_speed_scale);
    for(uint16_t s=0;s<p->segment_count&&s<TEACHING_MAX_SEGMENTS;++s){const TaughtSegment *g=&p->segments[s];
        DCRC(g->segment_id);DCRC(g->type);DCRC(g->point_count);DCRC(g->speed_mps);DCRC(g->circle_direction);DCRC(g->orientation_mode);
        for(uint8_t k=0;k<g->point_count&&k<TEACHING_MAX_POINTS_PER_SEG;++k){const TaughtPoint *q=&g->points[k];
            crc=crc32_update(crc,q->position_m,sizeof(q->position_m));crc=crc32_update(crc,q->orientation_quat,sizeof(q->orientation_quat));
            crc=crc32_update(crc,q->joint_position_rad,sizeof(q->joint_position_rad));DCRC(q->record_timestamp_ms);DCRC(q->calibration_version);DCRC(q->frame_id);DCRC(q->tool_id);}}
#undef DCRC
    return ~crc;
}

static uint32_t artifact_crc(const ValidatedTrajectory *a)
{
    uint32_t crc=0xFFFFFFFFUL;
#define ACRC(x) do{crc=crc32_update(crc,&(x),sizeof(x));}while(0)
    ACRC(a->program_id);ACRC(a->source_revision);ACRC(a->source_crc);ACRC(a->sample_data_crc);ACRC(a->sample_count);ACRC(a->segment_count);
    ACRC(a->sample_period_us);ACRC(a->duration_s);ACRC(a->path_length_m);
    for(uint16_t i=0;i<a->segment_count;++i){const PvSegmentIndex *g=&a->segments[i];
        ACRC(g->first_sample);ACRC(g->sample_count);ACRC(g->segment_type);}
#undef ACRC
    return ~crc;
}

static void fail(PathValidationState *s, PathValidationError e)
{
    s->error = e;
    s->result = PV_RESULT_INVALID;
    s->phase = PV_PHASE_INVALID;
    s->report.error = e;
    s->report.result = PV_RESULT_INVALID;
    s->report.phase = PV_PHASE_INVALID;
    s->report.failed_segment = s->segment_index;
    s->report.failed_sample = s->sample_index;
    if(s->storage.abort)s->storage.abort(s->storage.context);
    PV_LOG_ERROR("%s segment=%u sample=%u\r\n", PathValidation_ErrorName(e),
                 (unsigned)s->segment_index, (unsigned)s->sample_index);
}

static bool finite_config(const PathValidationConfig *c)
{
    uint8_t j;
    if (!(isfinite(c->default_tcp_speed_mps) && c->default_tcp_speed_mps > 0.0F &&
          isfinite(c->max_tcp_speed_mps) && c->max_tcp_speed_mps > 0.0F &&
          isfinite(c->max_tcp_acceleration_mps2) && c->max_tcp_acceleration_mps2 > 0.0F &&
          isfinite(c->max_tcp_jerk_mps3) && c->max_tcp_jerk_mps3 > 0.0F &&
          isfinite(c->minimum_segment_length_m) && c->minimum_segment_length_m > 0.0F &&
          isfinite(c->maximum_fk_position_error_m) && c->maximum_fk_position_error_m > 0.0F &&
          isfinite(c->maximum_fk_orientation_error_rad) && c->maximum_fk_orientation_error_rad > 0.0F &&
          isfinite(c->minimum_singularity_sigma) && c->minimum_singularity_sigma >= 0.0F &&
          isfinite(c->maximum_joint_step_rad) && c->maximum_joint_step_rad > 0.0F &&
          isfinite(c->maximum_position_quantization_error_rad) &&
          c->maximum_position_quantization_error_rad >= 0.0F)) return false;
    for (j = 0; j < PV_DOF; ++j) {
        if (!isfinite(c->joint_min_rad[j]) || !isfinite(c->joint_max_rad[j]) ||
            c->joint_min_rad[j] >= c->joint_max_rad[j] ||
            !isfinite(c->joint_velocity_max_rad_s[j]) || c->joint_velocity_max_rad_s[j] <= 0.0F ||
            !isfinite(c->joint_acceleration_max_rad_s2[j]) || c->joint_acceleration_max_rad_s2[j] <= 0.0F ||
            !isfinite(c->drive_units_per_joint_rad[j]) || c->drive_units_per_joint_rad[j] <= 0.0F ||
            (c->joint_direction_sign[j] != 1 && c->joint_direction_sign[j] != -1)) return false;
    }
    return true;
}

static bool radians_to_drive_units(const PathValidationConfig *c,
                                   uint8_t joint,
                                   float radians,
                                   int32_t *units,
                                   float *represented_radians)
{
    double signed_scale;
    double command;
    long long rounded;
    if(!c || !units || !represented_radians || joint>=PV_DOF || !isfinite(radians))return false;
    signed_scale=(double)c->drive_units_per_joint_rad[joint]*(double)c->joint_direction_sign[joint];
    command=(double)c->drive_zero_offset_units[joint]+(double)radians*signed_scale;
    if(!isfinite(command)||command<(double)INT32_MIN||command>(double)INT32_MAX)return false;
    rounded=llround(command);
    if(rounded<INT32_MIN||rounded>INT32_MAX)return false;
    *units=(int32_t)rounded;
    *represented_radians=(float)(((double)*units-(double)c->drive_zero_offset_units[joint])/signed_scale);
    return isfinite(*represented_radians);
}

static float norm3_delta(const float a[3], const float b[3])
{
    float x=a[0]-b[0], y=a[1]-b[1], z=a[2]-b[2];
    return sqrtf(x*x+y*y+z*z);
}

static float dot3(const float a[3],const float b[3]){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static void cross3(const float a[3],const float b[3],float c[3]){c[0]=a[1]*b[2]-a[2]*b[1];c[1]=a[2]*b[0]-a[0]*b[2];c[2]=a[0]*b[1]-a[1]*b[0];}
static float norm3(const float a[3]){return sqrtf(dot3(a,a));}
static bool normalize3(float a[3]){float n=norm3(a);if(!isfinite(n)||n<PV_EPS)return false;for(uint8_t i=0;i<3;++i)a[i]/=n;return true;}
static float positive_mod_2pi(float x){const float two_pi=2.0F*PV_PI;float r=fmodf(x,two_pi);return r<0.0F?r+two_pi:r;}

static bool normalize_quat(float q[4])
{
    float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    uint8_t i;
    if (!isfinite(n) || n < PV_EPS) return false;
    for (i=0;i<4;++i) q[i]/=n;
    return true;
}

static bool point_numeric_valid(const TaughtPoint *p)
{
    float qnorm=0.0F;
    for(uint8_t i=0;i<3;++i)if(!isfinite(p->position_m[i]))return false;
    for(uint8_t i=0;i<4;++i){if(!isfinite(p->orientation_quat[i]))return false;qnorm+=p->orientation_quat[i]*p->orientation_quat[i];}
    for(uint8_t i=0;i<PV_DOF;++i)if(!isfinite(p->joint_position_rad[i]))return false;
    return qnorm>PV_EPS;
}

static void slerp(const float qa[4], const float qb_in[4], float u, float out[4])
{
    float q0[4],qb[4], dot, theta, st, a, b;
    uint8_t i;
    memcpy(q0,qa,sizeof(q0));memcpy(qb,qb_in,sizeof(qb));(void)normalize_quat(q0);(void)normalize_quat(qb);
    dot=q0[0]*qb[0]+q0[1]*qb[1]+q0[2]*qb[2]+q0[3]*qb[3];
    if (dot<0.0F) { for(i=0;i<4;++i) qb[i]=-qb[i]; dot=-dot; }
    if (dot>0.9995F) { for(i=0;i<4;++i) out[i]=q0[i]+u*(qb[i]-q0[i]); (void)normalize_quat(out); return; }
    if (dot>1.0F) dot=1.0F;
    theta=acosf(dot); st=sinf(theta);
    a=sinf((1.0F-u)*theta)/st; b=sinf(u*theta)/st;
    for(i=0;i<4;++i) out[i]=a*q0[i]+b*qb[i];
    (void)normalize_quat(out);
}

static void quat_multiply(const float a[4],const float b[4],float q[4])
{
    q[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    q[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    q[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    q[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}

static void orientation_from_arc(const float q_start_in[4],const float q_end_in[4],const float axis[3],float theta_total,float u,float out[4])
{
    float qs[4],qe[4],qpath[4],qpred[4],qpath_full[4],qpred_end[4],qpred_inv[4],left[4],left_u[4],identity[4]={1,0,0,0};
    memcpy(qs,q_start_in,sizeof(qs));memcpy(qe,q_end_in,sizeof(qe));(void)normalize_quat(qs);(void)normalize_quat(qe);
    qpath[0]=cosf(0.5F*u*theta_total);for(uint8_t i=0;i<3;++i)qpath[i+1]=sinf(0.5F*u*theta_total)*axis[i];
    quat_multiply(qpath,qs,qpred);
    qpath_full[0]=cosf(0.5F*theta_total);for(uint8_t i=0;i<3;++i)qpath_full[i+1]=sinf(0.5F*theta_total)*axis[i];
    quat_multiply(qpath_full,qs,qpred_end);qpred_inv[0]=qpred_end[0];for(uint8_t i=1;i<4;++i)qpred_inv[i]=-qpred_end[i];
    quat_multiply(qe,qpred_inv,left);(void)normalize_quat(left);if(left[0]<0.0F)for(uint8_t i=0;i<4;++i)left[i]=-left[i];
    slerp(identity,left,u,left_u);quat_multiply(left_u,qpred,out);(void)normalize_quat(out);
}

static bool prepare_circle_geometry(PathValidationState *s,const TaughtSegment *seg,bool full_circle)
{
    const float *p1=seg->points[0].position_m,*p2=seg->points[1].position_m,*p3=seg->points[2].position_m;
    float a[3],b[3],n[3],bxn[3],nxa[3],n2,a2,b2,theta_mid,theta_end,ccw,cw,mid_ccw,mid_cw;
    for(uint8_t i=0;i<3;++i){a[i]=p2[i]-p1[i];b[i]=p3[i]-p1[i];}
    cross3(a,b,n);n2=dot3(n,n);if(n2<PV_EPS*PV_EPS)return false;
    cross3(b,n,bxn);cross3(n,a,nxa);a2=dot3(a,a);b2=dot3(b,b);
    for(uint8_t i=0;i<3;++i)s->geometry_center_m[i]=p1[i]+(a2*bxn[i]+b2*nxa[i])/(2.0F*n2);
    memcpy(s->geometry_axis,n,sizeof(n));if(!normalize3(s->geometry_axis))return false;
    s->geometry_radius_m=norm3_delta(p1,s->geometry_center_m);if(s->geometry_radius_m<s->config.minimum_segment_length_m)return false;
    for(uint8_t i=0;i<3;++i)s->geometry_e1[i]=(p1[i]-s->geometry_center_m[i])/s->geometry_radius_m;
    cross3(s->geometry_axis,s->geometry_e1,s->geometry_e2);if(!normalize3(s->geometry_e2))return false;
    if(full_circle){
        if(seg->circle_direction!=TEACH_CIRCLE_CCW&&seg->circle_direction!=TEACH_CIRCLE_CW)return false;
        s->geometry_theta_total_rad=(float)seg->circle_direction*2.0F*PV_PI;
    }else{
        float pm[3],pe[3];for(uint8_t i=0;i<3;++i){pm[i]=p2[i]-s->geometry_center_m[i];pe[i]=p3[i]-s->geometry_center_m[i];}
        theta_mid=atan2f(dot3(pm,s->geometry_e2),dot3(pm,s->geometry_e1));theta_end=atan2f(dot3(pe,s->geometry_e2),dot3(pe,s->geometry_e1));
        ccw=positive_mod_2pi(theta_end);if(ccw<PV_EPS)ccw=2.0F*PV_PI;mid_ccw=positive_mod_2pi(theta_mid);
        cw=-positive_mod_2pi(-theta_end);if(cw>-PV_EPS)cw=-2.0F*PV_PI;mid_cw=-positive_mod_2pi(-theta_mid);
        bool ccw_ok=(mid_ccw<=ccw+PV_EPS)&&(mid_ccw>PV_EPS),cw_ok=(mid_cw>=cw-PV_EPS)&&(mid_cw<-PV_EPS);
        if(ccw_ok==cw_ok) {
            return false;
        }
        s->geometry_theta_total_rad=ccw_ok?ccw:cw;
    }
    s->segment_length_m=s->geometry_radius_m*fabsf(s->geometry_theta_total_rad);
    memcpy(s->geometry_q_start,seg->points[0].orientation_quat,sizeof(s->geometry_q_start));
    if(full_circle)memcpy(s->geometry_q_end,seg->points[0].orientation_quat,sizeof(s->geometry_q_end));
    else memcpy(s->geometry_q_end,seg->points[2].orientation_quat,sizeof(s->geometry_q_end));
    return true;
}

static float orientation_error(const float a[4], const float b[4])
{
    float d=fabsf(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]);
    if (d>1.0F) d=1.0F;
    return 2.0F*acosf(d);
}

/* Exact seven-phase S-curve case selection used by the MATLAB function. */
static bool build_scurve(float vmax, float amax, float jmax, PvSCurve *p)
{
    float v_at_amax, d_reach_amax, vpeak, apeak, tj, ta, tv, d_vmax;
    float cs=0.0F, cv=0.0F, ca=0.0F, ct=0.0F;
    uint8_t k;
    if (vmax<=0.0F || amax<=0.0F || jmax<=0.0F) return false;
    v_at_amax=amax*amax/jmax;
    d_reach_amax=2.0F*amax*amax*amax/(jmax*jmax);
    if (vmax<v_at_amax) {
        float tj_v=sqrtf(vmax/jmax); d_vmax=2.0F*vmax*tj_v;
        if (1.0F>=d_vmax) { vpeak=vmax; tj=tj_v; ta=0.0F; apeak=jmax*tj; tv=(1.0F-d_vmax)/vpeak; }
        else { vpeak=powf(sqrtf(jmax)/2.0F,2.0F/3.0F); tj=sqrtf(vpeak/jmax); ta=tv=0.0F; apeak=jmax*tj; }
    } else {
        tj=amax/jmax; d_vmax=vmax*(tj+vmax/amax);
        if (1.0F>=d_vmax) { vpeak=vmax; apeak=amax; ta=vpeak/amax-tj; tv=(1.0F-d_vmax)/vpeak; }
        else if (1.0F>=d_reach_amax) { apeak=amax; tv=0.0F; vpeak=(-amax*amax/jmax+sqrtf(powf(amax*amax/jmax,2.0F)+4.0F*amax))/2.0F; ta=vpeak/amax-tj; }
        else { vpeak=powf(sqrtf(jmax)/2.0F,2.0F/3.0F); tj=sqrtf(vpeak/jmax); ta=tv=0.0F; apeak=jmax*tj; }
    }
    (void)apeak;
    p->duration[0]=tj; p->duration[1]=ta; p->duration[2]=tj; p->duration[3]=tv;
    p->duration[4]=tj; p->duration[5]=ta; p->duration[6]=tj;
    p->jerk[0]=jmax; p->jerk[1]=0; p->jerk[2]=-jmax; p->jerk[3]=0;
    p->jerk[4]=-jmax; p->jerk[5]=0; p->jerk[6]=jmax;
    for(k=0;k<7;++k){ float h=p->duration[k],j=p->jerk[k];
        p->start_time[k]=ct; p->start_s[k]=cs; p->start_v[k]=cv; p->start_a[k]=ca;
        cs=cs+cv*h+0.5F*ca*h*h+(j*h*h*h)/6.0F;
        cv=cv+ca*h+0.5F*j*h*h; ca=ca+j*h; ct+=h;
    }
    p->total_time=ct; return isfinite(ct) && ct>0.0F;
}

static void eval_scurve(const PvSCurve *p, float t, float *s, float *v)
{
    uint8_t k=6;
    float tau,j;
    for(uint8_t i=0;i<7;++i) if(t<=p->start_time[i]+p->duration[i]+1e-6F){k=i;break;}
    tau=t-p->start_time[k]; if(tau<0)tau=0; if(tau>p->duration[k])tau=p->duration[k]; j=p->jerk[k];
    *s=p->start_s[k]+p->start_v[k]*tau+0.5F*p->start_a[k]*tau*tau+j*tau*tau*tau/6.0F;
    *v=p->start_v[k]+p->start_a[k]*tau+0.5F*j*tau*tau;
    if(t>=p->total_time){*s=1.0F;*v=0.0F;}
}

static bool prepare_segment(PathValidationState *s)
{
    const TaughtSegment *seg=&s->source_program->segments[s->segment_index];
    float speed, vn,an,jn;
    PvSCurve p;
    if(seg->type==TEACH_SEGMENT_LINE){
        s->segment_length_m=norm3_delta(seg->points[1].position_m,seg->points[0].position_m);
        memcpy(s->geometry_q_start,seg->points[0].orientation_quat,sizeof(s->geometry_q_start));
        memcpy(s->geometry_q_end,seg->points[1].orientation_quat,sizeof(s->geometry_q_end));
    }else if(seg->type==TEACH_SEGMENT_ARC){
        if(!prepare_circle_geometry(s,seg,false)){fail(s,PV_ERR_DEGENERATE_GEOMETRY);return false;}
    }else if(seg->type==TEACH_SEGMENT_CIRCLE){
        if(seg->circle_direction!=TEACH_CIRCLE_CCW&&seg->circle_direction!=TEACH_CIRCLE_CW){fail(s,PV_ERR_INVALID_CIRCLE_DIRECTION);return false;}
        if(!prepare_circle_geometry(s,seg,true)){fail(s,PV_ERR_DEGENERATE_GEOMETRY);return false;}
    }else{fail(s,PV_ERR_UNSUPPORTED_SEGMENT);return false;}
    if(s->segment_length_m<s->config.minimum_segment_length_m){fail(s,PV_ERR_ZERO_LENGTH_SEGMENT);return false;}
    speed=(seg->speed_mps>0.0F)?seg->speed_mps:s->config.default_tcp_speed_mps;
    speed*=s->source_program->global_speed_scale;
    if(speed>s->config.max_tcp_speed_mps || speed<=0.0F){fail(s,PV_ERR_INVALID_PARAMETER);return false;}
    vn=speed/s->segment_length_m; an=s->config.max_tcp_acceleration_mps2/s->segment_length_m;
    jn=s->config.max_tcp_jerk_mps3/s->segment_length_m;
    if(!build_scurve(vn,an,jn,&p)){fail(s,PV_ERR_INVALID_PARAMETER);return false;}
    s->segment_duration_s=p.total_time; s->s_curve_tj_s=p.duration[0];
    s->s_curve_ta_s=p.duration[1]; s->s_curve_tv_s=p.duration[3]; s->s_curve_jerk_norm=jn;
    s->segment_sample_count=(uint32_t)ceilf(p.total_time/PV_SAMPLE_PERIOD_S)+1UL;
    /* For later segments skip local sample zero: it duplicates the previous
       segment endpoint and would create a zero-dt pair in the global artifact. */
    s->segment_local_sample=(s->segment_index==0U)?0U:1U;
    s->artifact->segments[s->segment_index].first_sample=s->sample_index;
    s->artifact->segments[s->segment_index].sample_count=0U;
    s->artifact->segments[s->segment_index].segment_type=(uint8_t)seg->type;
    PV_LOG_INFO("segment=%u L=%.5f m speed=%.5f m/s T=%.5f s samples=%u\r\n",
      (unsigned)s->segment_index,(double)s->segment_length_m,(double)speed,(double)p.total_time,(unsigned)s->segment_sample_count);
    return true;
}

static bool generate_and_validate_sample(PathValidationState *s)
{
    const TaughtSegment *seg=&s->source_program->segments[s->segment_index];
    PvSCurve p; PvExecutionSample sample,*out=&sample; PvCartesianPose target,achieved;
    PvIkDiagnostics ik; float q_ik[PV_DOF],q[PV_DOF],qd[PV_DOF],qdd[PV_DOF];
    float t,u,v,current_time_s,dt,poserr,orierr; uint8_t j;
    float speed=(seg->speed_mps>0.0F?seg->speed_mps:s->config.default_tcp_speed_mps)*s->source_program->global_speed_scale;
    if(!build_scurve(speed/s->segment_length_m,
       s->config.max_tcp_acceleration_mps2/s->segment_length_m,
       s->config.max_tcp_jerk_mps3/s->segment_length_m,&p)) return false;
    if(s->sample_index>=s->storage.capacity_samples){fail(s,PV_ERR_SAMPLE_CAPACITY);return false;}
    t=s->segment_local_sample*PV_SAMPLE_PERIOD_S; if(t>p.total_time)t=p.total_time;
    current_time_s=(float)s->sample_index*PV_SAMPLE_PERIOD_S;
    eval_scurve(&p,t,&u,&v);
    (void)v; memset(&target,0,sizeof(target));memset(q_ik,0,sizeof(q_ik));memset(q,0,sizeof(q));memset(qd,0,sizeof(qd));memset(qdd,0,sizeof(qdd));
    if(seg->type==TEACH_SEGMENT_LINE){
        for(j=0;j<3;++j)target.position_m[j]=seg->points[0].position_m[j]+u*(seg->points[1].position_m[j]-seg->points[0].position_m[j]);
        slerp(s->geometry_q_start,s->geometry_q_end,u,target.orientation_quat);
    }else{
        float theta=u*s->geometry_theta_total_rad,c=cosf(theta),sn=sinf(theta);
        for(j=0;j<3;++j)target.position_m[j]=s->geometry_center_m[j]+s->geometry_radius_m*(c*s->geometry_e1[j]+sn*s->geometry_e2[j]);
        orientation_from_arc(s->geometry_q_start,s->geometry_q_end,s->geometry_axis,s->geometry_theta_total_rad,u,target.orientation_quat);
    }
    memset(&ik,0,sizeof(ik));
    if(!s->services.inverse_kinematics(&target,s->seed_rad,q_ik,&ik,s->services.user_context)||!ik.converged){fail(s,PV_ERR_IK_FAILED);return false;}
    if(ik.sigma_min<s->config.minimum_singularity_sigma){fail(s,PV_ERR_SINGULARITY_MARGIN);return false;}
    if(ik.sigma_min<s->report.minimum_sigma_seen)s->report.minimum_sigma_seen=ik.sigma_min;
    for(j=0;j<PV_DOF;++j){
        if(!radians_to_drive_units(&s->config,j,q_ik[j],&out->target_position_units[j],&q[j])){s->report.failed_joint=j;fail(s,PV_ERR_POSITION_CONVERSION);return false;}
        if(fabsf(q[j]-q_ik[j])>s->config.maximum_position_quantization_error_rad){s->report.failed_joint=j;fail(s,PV_ERR_POSITION_CONVERSION);return false;}
    }
    if(!s->services.forward_kinematics(q,&achieved,s->services.user_context)){fail(s,PV_ERR_CALLBACK_MISSING);return false;}
    poserr=norm3_delta(target.position_m,achieved.position_m);
    orierr=orientation_error(target.orientation_quat,achieved.orientation_quat);
    if(poserr>s->report.max_fk_position_error_m)s->report.max_fk_position_error_m=poserr;
    if(orierr>s->report.max_fk_orientation_error_rad)s->report.max_fk_orientation_error_rad=orierr;
    if(poserr>s->config.maximum_fk_position_error_m){fail(s,PV_ERR_FK_POSITION);return false;}
    if(orierr>s->config.maximum_fk_orientation_error_rad){fail(s,PV_ERR_FK_ORIENTATION);return false;}
    for(j=0;j<PV_DOF;++j){
        if(q[j]<s->config.joint_min_rad[j]||q[j]>s->config.joint_max_rad[j]){s->report.failed_joint=j;fail(s,PV_ERR_JOINT_POSITION);return false;}
        if(s->previous_q_valid){float dq=q[j]-s->previous_q_rad[j];dt=current_time_s-s->previous_time_s;
            if(dt<=PV_EPS){fail(s,PV_ERR_INVALID_PARAMETER);return false;}
            if(fabsf(dq)>s->config.maximum_joint_step_rad){s->report.failed_joint=j;fail(s,PV_ERR_JOINT_DISCONTINUITY);return false;}
            qd[j]=dq/dt;
            if(fabsf(qd[j])>s->report.peak_joint_velocity_rad_s[j])s->report.peak_joint_velocity_rad_s[j]=fabsf(qd[j]);
            if(fabsf(qd[j])>s->config.joint_velocity_max_rad_s[j]){s->report.failed_joint=j;fail(s,PV_ERR_JOINT_VELOCITY);return false;}
            if(s->previous_qd_valid){qdd[j]=(qd[j]-s->previous_qd_rad_s[j])/dt;
                if(fabsf(qdd[j])>s->report.peak_joint_acceleration_rad_s2[j])s->report.peak_joint_acceleration_rad_s2[j]=fabsf(qdd[j]);
                if(s->config.check_joint_acceleration&&fabsf(qdd[j])>s->config.joint_acceleration_max_rad_s2[j]){s->report.failed_joint=j;fail(s,PV_ERR_JOINT_ACCELERATION);return false;}
            }
        }
    }
    if(s->services.collision_free && !s->services.collision_free(q,s->segment_index,s->sample_index,s->services.user_context)){fail(s,PV_ERR_COLLISION);return false;}
    if(!s->storage.write_sample(s->sample_index,out,s->storage.context)){fail(s,PV_ERR_STORAGE);return false;}
    s->sample_crc_state=crc32_update(s->sample_crc_state,out,sizeof(*out));
    memcpy(s->seed_rad,q,sizeof(s->seed_rad));memcpy(s->previous_q_rad,q,sizeof(s->previous_q_rad));memcpy(s->previous_qd_rad_s,qd,sizeof(s->previous_qd_rad_s));
    s->previous_time_s=current_time_s;
    s->previous_qd_valid=s->previous_q_valid;s->previous_q_valid=true;
    ++s->artifact->segments[s->segment_index].sample_count;
    ++s->sample_index; ++s->segment_local_sample; s->artifact->sample_count=s->sample_index;
    return true;
}

bool PathValidation_Init(PathValidationState *s,const PathValidationConfig *c,const PathValidationServices *v,const PathValidationStorage *storage,ValidatedTrajectory *a)
{
    if(!s||!c||!v||!storage||!a||!finite_config(c)||!v->inverse_kinematics||!v->forward_kinematics||
       !storage->begin||!storage->write_sample||!storage->commit||storage->capacity_samples==0U||
       (c->require_collision_callback&&!v->collision_free))return false;
    memset(s,0,sizeof(*s)); s->config=*c;s->services=*v;s->storage=*storage;s->artifact=a;s->phase=PV_PHASE_IDLE;s->report.minimum_sigma_seen=FLT_MAX;return true;
}

bool PathValidation_Start(PathValidationState *s,const TaughtProgram *p,uint32_t rev,uint32_t crc)
{
    if(!s||!p||!s->artifact)return false;
    if(s->result==PV_RESULT_RUNNING)return false;
    memset(s->artifact,0,sizeof(*s->artifact)); memset(&s->report,0,sizeof(s->report)); s->report.minimum_sigma_seen=FLT_MAX;
    if(!s->storage.begin(s->storage.context)){s->error=PV_ERR_STORAGE;s->result=PV_RESULT_INVALID;s->phase=PV_PHASE_INVALID;s->report.error=s->error;s->report.result=s->result;s->report.phase=s->phase;return false;}
    s->source_program=p;s->expected_revision=rev;s->expected_crc=crc;s->segment_index=0;s->sample_index=0;s->segment_start_time_s=0;
    s->previous_q_valid=false;s->previous_qd_valid=false;s->sample_crc_state=0xFFFFFFFFUL;
    s->error=PV_ERR_NONE;s->result=PV_RESULT_RUNNING;s->phase=PV_PHASE_SNAPSHOT_CHECK;s->report.result=PV_RESULT_RUNNING;s->report.phase=s->phase;
    PV_LOG_INFO("start program=%lu revision=%lu crc=0x%08lX\r\n",(unsigned long)p->program_id,(unsigned long)rev,(unsigned long)crc);return true;
}

PathValidationResult PathValidation_Step(PathValidationState *s,uint16_t budget)
{
    uint16_t used=0; if(!s||s->result!=PV_RESULT_RUNNING)return s?s->result:PV_RESULT_INVALID; if(budget==0)budget=1;
    while(s->result==PV_RESULT_RUNNING&&used<budget){
        switch(s->phase){
        case PV_PHASE_SNAPSHOT_CHECK:
            if(s->source_program->draft_revision!=s->expected_revision){fail(s,PV_ERR_REVISION_MISMATCH);break;}
            if(s->source_program->draft_crc!=s->expected_crc||PathValidation_CalculateDraftCrc(s->source_program)!=s->expected_crc){fail(s,PV_ERR_DRAFT_CRC_MISMATCH);break;}
            if(s->source_program->segment_count==0){fail(s,PV_ERR_EMPTY_PROGRAM);break;}
            s->phase=PV_PHASE_STRUCTURAL_CHECK;break;
        case PV_PHASE_STRUCTURAL_CHECK:{const TaughtPoint *r=&s->source_program->segments[0].points[0];
            for(uint16_t i=0;i<s->source_program->segment_count;++i){const TaughtSegment *g=&s->source_program->segments[i];
                uint8_t required=(g->type==TEACH_SEGMENT_LINE)?2U:((g->type==TEACH_SEGMENT_ARC||g->type==TEACH_SEGMENT_CIRCLE)?3U:0U);
                if(required==0U){fail(s,PV_ERR_UNSUPPORTED_SEGMENT);break;}
                if(!g->segment_valid||g->point_count!=required){fail(s,PV_ERR_INCOMPLETE_SEGMENT);break;}
                for(uint8_t k=0;k<required;++k){if(!g->points[k].point_valid||!point_numeric_valid(&g->points[k])){fail(s,PV_ERR_INVALID_POINT);break;} if(g->points[k].frame_id!=r->frame_id){fail(s,PV_ERR_FRAME_MISMATCH);break;} if(g->points[k].tool_id!=r->tool_id){fail(s,PV_ERR_TOOL_MISMATCH);break;} if(g->points[k].calibration_version!=r->calibration_version){fail(s,PV_ERR_CALIBRATION_MISMATCH);break;}}
                if(s->result!=PV_RESULT_RUNNING)break;}
            if(s->result==PV_RESULT_RUNNING){memcpy(s->seed_rad,r->joint_position_rad,sizeof(s->seed_rad));s->phase=PV_PHASE_PREPARE_SEGMENT;}break;}
        case PV_PHASE_PREPARE_SEGMENT:
            if(!prepare_segment(s)) {
                break;
            }
            s->phase=PV_PHASE_GENERATE_AND_VALIDATE_SAMPLE;
            break;
        case PV_PHASE_GENERATE_AND_VALIDATE_SAMPLE:
            if(!generate_and_validate_sample(s)) {
                break;
            }
            ++used;
            if(s->segment_local_sample>=s->segment_sample_count){s->artifact->path_length_m+=s->segment_length_m;s->segment_start_time_s=(float)s->sample_index*PV_SAMPLE_PERIOD_S;++s->segment_index;
                if(s->segment_index>=s->source_program->segment_count)s->phase=PV_PHASE_FINALIZE;else{s->phase=PV_PHASE_PREPARE_SEGMENT;}}
            break;
        case PV_PHASE_FINALIZE:
            s->artifact->program_id=s->source_program->program_id;s->artifact->source_revision=s->expected_revision;s->artifact->source_crc=s->expected_crc;
            s->artifact->segment_count=s->source_program->segment_count;s->artifact->sample_period_us=PV_SAMPLE_PERIOD_US;
            s->artifact->duration_s=(s->artifact->sample_count>0U)?((float)(s->artifact->sample_count-1U)*PV_SAMPLE_PERIOD_S):0.0F;
            s->artifact->sample_data_crc=~s->sample_crc_state;s->artifact->artifact_crc=artifact_crc(s->artifact);
            if(!s->storage.commit(s->artifact,s->storage.context)){fail(s,PV_ERR_STORAGE);break;}
            s->result=PV_RESULT_VALID;s->phase=PV_PHASE_VALID;s->report.result=s->result;s->report.phase=s->phase;s->report.progress_0_to_1=1.0F;
            PV_LOG_INFO("VALID samples=%u duration=%.4f s length=%.4f m artifact_crc=0x%08lX\r\n",(unsigned)s->artifact->sample_count,(double)s->artifact->duration_s,(double)s->artifact->path_length_m,(unsigned long)s->artifact->artifact_crc);break;
        default: fail(s,PV_ERR_INVALID_PARAMETER);break;}
        s->report.phase=s->phase;
        if(s->source_program->segment_count>0)s->report.progress_0_to_1=(float)s->segment_index/(float)s->source_program->segment_count;
    }
    return s->result;
}

void PathValidation_Cancel(PathValidationState *s){if(s&&s->result==PV_RESULT_RUNNING){if(s->storage.abort)s->storage.abort(s->storage.context);s->error=PV_ERR_CANCELLED;s->result=PV_RESULT_CANCELLED;s->phase=PV_PHASE_INVALID;s->report.error=s->error;s->report.result=s->result;PV_LOG_WARN("cancelled\r\n");}}
const PathValidationReport *PathValidation_GetReport(const PathValidationState *s){return s?&s->report:NULL;}
const char *PathValidation_PhaseName(PathValidationPhase p){switch(p){case PV_PHASE_IDLE:return"IDLE";case PV_PHASE_SNAPSHOT_CHECK:return"SNAPSHOT_CHECK";case PV_PHASE_STRUCTURAL_CHECK:return"STRUCTURAL_CHECK";case PV_PHASE_PREPARE_SEGMENT:return"PREPARE_SEGMENT";case PV_PHASE_GENERATE_AND_VALIDATE_SAMPLE:return"GENERATE_AND_VALIDATE_SAMPLE";case PV_PHASE_FINALIZE:return"FINALIZE";case PV_PHASE_VALID:return"VALID";case PV_PHASE_INVALID:return"INVALID";default:return"UNKNOWN";}}
const char *PathValidation_ErrorName(PathValidationError e){switch(e){case PV_ERR_NONE:return"NONE";case PV_ERR_NULL_ARGUMENT:return"NULL_ARGUMENT";case PV_ERR_BUSY:return"BUSY";case PV_ERR_EMPTY_PROGRAM:return"EMPTY_PROGRAM";case PV_ERR_REVISION_MISMATCH:return"REVISION_MISMATCH";case PV_ERR_DRAFT_CRC_MISMATCH:return"DRAFT_CRC_MISMATCH";case PV_ERR_UNSUPPORTED_SEGMENT:return"UNSUPPORTED_SEGMENT";case PV_ERR_INCOMPLETE_SEGMENT:return"INCOMPLETE_SEGMENT";case PV_ERR_INVALID_POINT:return"INVALID_POINT";case PV_ERR_DEGENERATE_GEOMETRY:return"DEGENERATE_GEOMETRY";case PV_ERR_ARC_DIRECTION:return"ARC_DIRECTION";case PV_ERR_INVALID_CIRCLE_DIRECTION:return"INVALID_CIRCLE_DIRECTION";case PV_ERR_FRAME_MISMATCH:return"FRAME_MISMATCH";case PV_ERR_TOOL_MISMATCH:return"TOOL_MISMATCH";case PV_ERR_CALIBRATION_MISMATCH:return"CALIBRATION_MISMATCH";case PV_ERR_INVALID_PARAMETER:return"INVALID_PARAMETER";case PV_ERR_ZERO_LENGTH_SEGMENT:return"ZERO_LENGTH_SEGMENT";case PV_ERR_SAMPLE_CAPACITY:return"SAMPLE_CAPACITY";case PV_ERR_IK_FAILED:return"IK_FAILED";case PV_ERR_FK_POSITION:return"FK_POSITION";case PV_ERR_FK_ORIENTATION:return"FK_ORIENTATION";case PV_ERR_JOINT_POSITION:return"JOINT_POSITION";case PV_ERR_JOINT_VELOCITY:return"JOINT_VELOCITY";case PV_ERR_JOINT_ACCELERATION:return"JOINT_ACCELERATION";case PV_ERR_JOINT_DISCONTINUITY:return"JOINT_DISCONTINUITY";case PV_ERR_POSITION_CONVERSION:return"POSITION_CONVERSION";case PV_ERR_SINGULARITY_MARGIN:return"SINGULARITY_MARGIN";case PV_ERR_COLLISION:return"COLLISION";case PV_ERR_STORAGE:return"STORAGE";case PV_ERR_CALLBACK_MISSING:return"CALLBACK_MISSING";case PV_ERR_CANCELLED:return"CANCELLED";default:return"UNKNOWN";}}
