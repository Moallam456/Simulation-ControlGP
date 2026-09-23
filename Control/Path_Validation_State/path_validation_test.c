#include "path_validation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct { PvCartesianPose last_target; } MockContext;

#define TEST_STORAGE_CAPACITY (30000UL)
typedef struct {
    PvExecutionSample samples[TEST_STORAGE_CAPACITY];
    uint32_t count;
    bool committed;
    bool aborted;
} RamStorage;

static bool storage_begin(void *ctx)
{
    RamStorage *s=(RamStorage *)ctx;s->count=0U;s->committed=false;s->aborted=false;return true;
}
static bool storage_write(uint32_t index,const PvExecutionSample *sample,void *ctx)
{
    RamStorage *s=(RamStorage *)ctx;if(index>=TEST_STORAGE_CAPACITY||index!=s->count)return false;
    s->samples[index]=*sample;++s->count;return true;
}
static bool storage_commit(const ValidatedTrajectory *metadata,void *ctx)
{
    RamStorage *s=(RamStorage *)ctx;if(metadata->sample_count!=s->count)return false;s->committed=true;return true;
}
static void storage_abort(void *ctx){((RamStorage *)ctx)->aborted=true;}

static bool mock_ik(const PvCartesianPose *target,const float seed[PV_DOF],float q[PV_DOF],PvIkDiagnostics *d,void *ctx)
{
    MockContext *m=(MockContext *)ctx;(void)seed;memset(q,0,sizeof(float)*PV_DOF);m->last_target=*target;
    q[0]=target->position_m[0];q[1]=target->position_m[1];q[2]=target->position_m[2];
    d->converged=true;d->iterations=4;d->sigma_min=0.5F;return true;
}
static bool mock_fk(const float q[PV_DOF],PvCartesianPose *p,void *ctx)
{
    MockContext *m=(MockContext *)ctx;*p=m->last_target;p->position_m[0]=q[0];p->position_m[1]=q[1];p->position_m[2]=q[2];return true;
}
static bool no_collision(const float q[PV_DOF],uint16_t seg,uint32_t sample,void *ctx)
{(void)q;(void)seg;(void)sample;(void)ctx;return true;}

static void make_point(TaughtPoint *p,float x,float y,float z)
{
    memset(p,0,sizeof(*p));p->position_m[0]=x;p->position_m[1]=y;p->position_m[2]=z;p->orientation_quat[0]=1.0F;
    p->frame_id=1;p->tool_id=2;p->calibration_version=3;p->point_valid=true;
}

int main(void)
{
    PathValidationState state;ValidatedTrajectory artifact;TaughtProgram program;PathValidationResult result;MockContext mock;
    static RamStorage ram;
    PathValidationConfig c={.default_tcp_speed_mps=0.05F,.max_tcp_speed_mps=0.10F,
      .max_tcp_acceleration_mps2=0.25F,.max_tcp_jerk_mps3=1.0F,.minimum_segment_length_m=0.001F,
      .maximum_fk_position_error_m=1e-4F,.maximum_fk_orientation_error_rad=1e-4F,
      .minimum_singularity_sigma=0.01F,.maximum_joint_step_rad=0.1F,
      .maximum_position_quantization_error_rad=1.0e-6F,.require_collision_callback=true};
    PathValidationServices services={mock_ik,mock_fk,no_collision,&mock};
    PathValidationStorage storage={storage_begin,storage_write,storage_commit,storage_abort,TEST_STORAGE_CAPACITY,&ram};
    memset(&program,0,sizeof(program));memset(&mock,0,sizeof(mock));
    for(uint8_t j=0;j<PV_DOF;++j){c.joint_min_rad[j]=-6.3F;c.joint_max_rad[j]=6.3F;c.joint_velocity_max_rad_s[j]=10.0F;c.joint_acceleration_max_rad_s2[j]=100.0F;c.drive_units_per_joint_rad[j]=1000000.0F;c.drive_zero_offset_units[j]=0;c.joint_direction_sign[j]=1;}
    program.program_id=9;program.draft_revision=4;program.segment_count=3;program.global_speed_scale=1.0F;
    program.segments[0].segment_id=1;program.segments[0].type=TEACH_SEGMENT_LINE;program.segments[0].point_count=2;
    program.segments[0].speed_mps=0.05F;program.segments[0].segment_valid=true;
    make_point(&program.segments[0].points[0],0.10F,0.00F,0.00F);make_point(&program.segments[0].points[1],0.20F,0.00F,0.00F);

    program.segments[1].segment_id=2;program.segments[1].type=TEACH_SEGMENT_ARC;program.segments[1].point_count=3;
    program.segments[1].speed_mps=0.05F;program.segments[1].segment_valid=true;
    make_point(&program.segments[1].points[0],0.20F,0.00F,0.00F);make_point(&program.segments[1].points[1],0.25F,0.05F,0.00F);make_point(&program.segments[1].points[2],0.30F,0.00F,0.00F);

    program.segments[2].segment_id=3;program.segments[2].type=TEACH_SEGMENT_CIRCLE;program.segments[2].point_count=3;
    program.segments[2].speed_mps=0.05F;program.segments[2].segment_valid=true;program.segments[2].circle_direction=TEACH_CIRCLE_CCW;
    make_point(&program.segments[2].points[0],0.30F,0.00F,0.00F);make_point(&program.segments[2].points[1],0.35F,0.05F,0.00F);make_point(&program.segments[2].points[2],0.40F,0.00F,0.00F);
    program.draft_crc=PathValidation_CalculateDraftCrc(&program);
    assert(PathValidation_Init(&state,&c,&services,&storage,&artifact));
    assert(PathValidation_Start(&state,&program,4,program.draft_crc));
    do{result=PathValidation_Step(&state,8);}while(result==PV_RESULT_RUNNING);
    assert(result==PV_RESULT_VALID);assert(artifact.sample_count>10);assert(artifact.duration_s>0);assert(artifact.path_length_m>0.56F);
    assert(artifact.segment_count==3U);assert(artifact.sample_period_us==1000U);assert(ram.committed);
    assert(artifact.segments[0].first_sample==0U);assert(artifact.segments[0].segment_type==TEACH_SEGMENT_LINE);
    assert(artifact.segments[1].first_sample==artifact.segments[0].sample_count);
    assert(artifact.segments[2].first_sample==(uint16_t)(artifact.segments[0].sample_count+artifact.segments[1].sample_count));
    assert((uint16_t)(artifact.segments[2].first_sample+artifact.segments[2].sample_count)==artifact.sample_count);
    assert(ram.samples[artifact.sample_count-1U].target_position_units[0]>290000);
    assert(fabsf(artifact.duration_s-(float)(artifact.sample_count-1U)*0.001F)<1e-5F);

    /* A changed payload with an old CRC must be rejected. */
    program.segments[0].points[1].position_m[0]=0.21F;
    assert(PathValidation_Start(&state,&program,4,program.draft_crc));
    assert(PathValidation_Step(&state,1)==PV_RESULT_INVALID);
    assert(PathValidation_GetReport(&state)->error==PV_ERR_DRAFT_CRC_MISMATCH);

    /* Restore the program, then verify degenerate ARC geometry is rejected. */
    program.segments[0].points[1].position_m[0]=0.20F;
    make_point(&program.segments[1].points[1],0.25F,0.00F,0.00F);
    program.draft_crc=PathValidation_CalculateDraftCrc(&program);
    assert(PathValidation_Start(&state,&program,4,program.draft_crc));
    do{result=PathValidation_Step(&state,8);}while(result==PV_RESULT_RUNNING);
    assert(result==PV_RESULT_INVALID);
    assert(PathValidation_GetReport(&state)->error==PV_ERR_DEGENERATE_GEOMETRY);

    printf("PvExecutionSample storage: %zu bytes\n",sizeof(PvExecutionSample));
    printf("ValidatedTrajectory storage: %zu bytes\n",sizeof(ValidatedTrajectory));
    assert(sizeof(PvExecutionSample)==PV_DOF*sizeof(int32_t));
    assert(sizeof(ValidatedTrajectory)<1024U);
    puts("Path validation tests passed.");return 0;
}
