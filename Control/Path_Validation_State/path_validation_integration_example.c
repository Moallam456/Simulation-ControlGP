/* Integration sketch: adapt to the selected STM32 core/RTOS architecture. */
#if 0
#include "path_validation.h"
#include "robot_kinematics.h"

static PathValidationState validation;
/* Metadata only. The CSP samples are written through PathValidationStorage. */
static ValidatedTrajectory trajectory;
static RobotKinematicsContext kinematics;

/* These functions are intentionally hardware-independent placeholders. When
 * the external-memory IC is selected, implement them with a page buffer and
 * the chosen QSPI/OCTOSPI driver. Do not program or erase flash from the 1 ms
 * CSP execution interrupt. */
static bool TrajectoryStore_Begin(void *context);
static bool TrajectoryStore_Write(uint32_t sample_index,
                                  const PvExecutionSample *sample,
                                  void *context);
static bool TrajectoryStore_Commit(const ValidatedTrajectory *metadata,
                                   void *context);
static void TrajectoryStore_Abort(void *context);

static PathValidationStorage trajectory_storage = {
    .begin = TrajectoryStore_Begin,
    .write_sample = TrajectoryStore_Write,
    .commit = TrajectoryStore_Commit,
    .abort = TrajectoryStore_Abort,
    .capacity_samples = 0U, /* Set after the external-memory partition is chosen. */
    .context = NULL
};

static void ConfigureKinematics(void)
{
    /* Copy the commissioned values from the single robot configuration
       source. These arrays are placeholders for integration, not constants
       to copy blindly from a UR5 or the MATLAB test robot. */
    LoadRobotDhParameters(kinematics.model.a_m,
                          kinematics.model.d_m,
                          kinematics.model.alpha_rad,
                          kinematics.model.theta_offset_rad);
    LoadRobotJointLimits(kinematics.model.joint_min_rad,
                         kinematics.model.joint_max_rad);
    LoadFlangeToTcpTransform(kinematics.model.T_flange_tcp);

    kinematics.adls.max_iterations = 1000U;
    kinematics.adls.position_tolerance_m = 1.0e-4F;
    kinematics.adls.orientation_tolerance_rad = 1.0e-4F;
    kinematics.adls.step_size = 0.5F;
    kinematics.adls.lambda_max = 0.1F;
    kinematics.adls.sigma_threshold = 1.0e-4F;

    /* ADLS_IK is numerical. Set ROBOT_IK_ANALYTICAL only after assigning a
       true closed-form solver to analytical_solver. */
    kinematics.selected_method = ROBOT_IK_NUMERICAL_ADLS;
    kinematics.analytical_solver = NULL;
    kinematics.analytical_context = NULL;
}

static void InitializePathValidation(void)
{
    PathValidationServices services;
    ConfigureKinematics();
    services = RobotKinematics_MakePathValidationServices(
        &kinematics, Collision_CheckConfiguration);
    (void)PathValidation_Init(&validation, &validation_config, &services,
                              &trajectory_storage, &trajectory);
}

void PathValidationTask(void)
{
    PathValidationResult r=PathValidation_Step(&validation,4U);
    Hmi_SetValidationProgress(PathValidation_GetReport(&validation)->progress_0_to_1);
    if(r==PV_RESULT_VALID){MainState_RequestTransition(STATE_PREVIEW_READY);}
    else if(r==PV_RESULT_INVALID){Hmi_ShowValidationError(PathValidation_GetReport(&validation));MainState_RequestTransition(STATE_TEACHING);}
}
#endif
