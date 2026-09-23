#include "robot_kinematics.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static RobotKinematicsModel make_ur5_scale_model(void)
{
    RobotKinematicsModel m;
    const float a[6]={0.0F,-0.425F,-0.39225F,0.0F,0.0F,0.0F};
    const float d[6]={0.089159F,0.0F,0.0F,0.10915F,0.09465F,0.0823F};
    const float alpha[6]={1.57079632679F,0.0F,0.0F,1.57079632679F,-1.57079632679F,0.0F};
    memset(&m,0,sizeof(m));memcpy(m.a_m,a,sizeof(a));memcpy(m.d_m,d,sizeof(d));memcpy(m.alpha_rad,alpha,sizeof(alpha));
    for(uint8_t i=0;i<6;++i){m.joint_min_rad[i]=-6.2831853F;m.joint_max_rad[i]=6.2831853F;}
    m.T_flange_tcp[0]=m.T_flange_tcp[5]=m.T_flange_tcp[10]=m.T_flange_tcp[15]=1.0F;
    return m;
}

static bool mock_analytical(const RobotKinematicsModel *model,
                            const PvCartesianPose *target,
                            const float seed[6],float solution[6],
                            PvIkDiagnostics *diag,void *context)
{
    (void)model;(void)target;(void)context;memcpy(solution,seed,6U*sizeof(float));
    memset(diag,0,sizeof(*diag));diag->converged=true;return true;
}

int main(void)
{
    RobotKinematicsModel model=make_ur5_scale_model();
    float q[6]={0.20F,-1.00F,1.10F,-0.60F,0.80F,0.30F};
    float J[36],h=1.0e-4F;
    PvCartesianPose pose;
    assert(RobotKinematics_Forward(&model,q,&pose));
    assert(RobotKinematics_Jacobian(&model,q,J));

    /* Translation rows of the analytic geometric Jacobian must match a
       central finite difference of controlFK. */
    for(uint8_t joint=0;joint<6;++joint){float qp[6],qm[6];PvCartesianPose pp,pm;memcpy(qp,q,sizeof(q));memcpy(qm,q,sizeof(q));qp[joint]+=h;qm[joint]-=h;
        assert(RobotKinematics_Forward(&model,qp,&pp));assert(RobotKinematics_Forward(&model,qm,&pm));
        for(uint8_t row=0;row<3;++row){float numeric=(pp.position_m[row]-pm.position_m[row])/(2*h);assert(fabsf(numeric-J[6U*row+joint])<2.0e-3F);}}

    /* ADLS must recover a nearby pose using the translated FK/Jacobian. */
    {
        const float qtarget[6]={0.24F,-0.96F,1.05F,-0.56F,0.76F,0.34F};
        float solved[6];PvCartesianPose target;PvIkDiagnostics diag;
        RobotAdlsConfig cfg={.max_iterations=1000U,.position_tolerance_m=1.0e-4F,.orientation_tolerance_rad=1.0e-4F,.step_size=0.5F,.lambda_max=0.1F,.sigma_threshold=1.0e-4F};
        assert(RobotKinematics_Forward(&model,qtarget,&target));
        assert(RobotKinematics_AdlsIk(&model,&cfg,&target,q,solved,&diag));
        assert(diag.converged);assert(diag.position_error_m<=cfg.position_tolerance_m);assert(diag.orientation_error_rad<=cfg.orientation_tolerance_rad);

        /* The same state callback can later dispatch to a true analytical
           implementation without changing Path Validation. */
        {RobotKinematicsContext context;PathValidationServices services;float routed[6];PvIkDiagnostics routed_diag;
            memset(&context,0,sizeof(context));context.model=model;context.adls=cfg;context.selected_method=ROBOT_IK_ANALYTICAL;context.analytical_solver=mock_analytical;
            services=RobotKinematics_MakePathValidationServices(&context,NULL);
            assert(services.inverse_kinematics(&target,q,routed,&routed_diag,services.user_context));
            for(uint8_t i=0;i<6;++i)assert(routed[i]==q[i]);}
    }

    puts("Robot kinematics tests passed.");
    return 0;
}
