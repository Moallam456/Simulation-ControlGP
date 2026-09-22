#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include "path_validation_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_KINEMATICS_DOF (6U)

typedef struct {
    float a_m[ROBOT_KINEMATICS_DOF];
    float d_m[ROBOT_KINEMATICS_DOF];
    float alpha_rad[ROBOT_KINEMATICS_DOF];
    float theta_offset_rad[ROBOT_KINEMATICS_DOF];
    float joint_min_rad[ROBOT_KINEMATICS_DOF];
    float joint_max_rad[ROBOT_KINEMATICS_DOF];
    /* Row-major flange-to-TCP homogeneous transform. */
    float T_flange_tcp[16];
} RobotKinematicsModel;

typedef struct {
    uint16_t max_iterations;
    float position_tolerance_m;
    float orientation_tolerance_rad;
    float step_size;
    float lambda_max;
    float sigma_threshold;
} RobotAdlsConfig;

typedef enum {
    ROBOT_IK_NUMERICAL_ADLS = 0,
    ROBOT_IK_ANALYTICAL
} RobotIkMethod;

typedef bool (*RobotAnalyticalIkFn)(const RobotKinematicsModel *model,
                                    const PvCartesianPose *target,
                                    const float seed_rad[ROBOT_KINEMATICS_DOF],
                                    float solution_rad[ROBOT_KINEMATICS_DOF],
                                    PvIkDiagnostics *diagnostics,
                                    void *context);

typedef struct {
    RobotKinematicsModel model;
    RobotAdlsConfig adls;
    RobotIkMethod selected_method;
    RobotAnalyticalIkFn analytical_solver;
    void *analytical_context;
} RobotKinematicsContext;

/* C equivalents of controlFK.m and controlJacobian.m. */
bool RobotKinematics_Forward(const RobotKinematicsModel *model,
                             const float q_rad[ROBOT_KINEMATICS_DOF],
                             PvCartesianPose *tcp_pose);

bool RobotKinematics_Jacobian(const RobotKinematicsModel *model,
                              const float q_rad[ROBOT_KINEMATICS_DOF],
                              float jacobian_6x6[36]);

/* C equivalent of the repository's numerical ADLS_IK.m. */
bool RobotKinematics_AdlsIk(const RobotKinematicsModel *model,
                            const RobotAdlsConfig *config,
                            const PvCartesianPose *target,
                            const float seed_rad[ROBOT_KINEMATICS_DOF],
                            float solution_rad[ROBOT_KINEMATICS_DOF],
                            PvIkDiagnostics *diagnostics);

/* Callback adapters used directly by PathValidationServices. */
bool RobotKinematics_PathIk(const PvCartesianPose *target,
                            const float seed_rad[PV_DOF],
                            float solution_rad[PV_DOF],
                            PvIkDiagnostics *diagnostics,
                            void *user_context);

bool RobotKinematics_PathFk(const float joints_rad[PV_DOF],
                            PvCartesianPose *achieved,
                            void *user_context);

PathValidationServices RobotKinematics_MakePathValidationServices(
    RobotKinematicsContext *context,
    PvCollisionCheckFn collision_callback);

#ifdef __cplusplus
}
#endif
#endif
