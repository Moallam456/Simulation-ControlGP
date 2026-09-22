#ifndef APPROACH_STATE_TYPES_H
#define APPROACH_STATE_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APPROACH_DOF                 6U
#define APPROACH_SAMPLE_PERIOD_US    1000U
#define APPROACH_SAMPLE_PERIOD_S     0.001f
#define APPROACH_MAX_CLEARANCE_POSES 3U
#define APPROACH_MAX_LEGS             (APPROACH_MAX_CLEARANCE_POSES + 1U)

typedef enum {
    APPROACH_OPERATION_NONE = 0,
    APPROACH_OPERATION_PREVIEW,
    APPROACH_OPERATION_WELD
} ApproachOperation;

typedef enum {
    APPROACH_PHASE_IDLE = 0,
    APPROACH_PHASE_CHECK_REQUEST,
    APPROACH_PHASE_LOAD_TARGET,
    APPROACH_PHASE_READ_START,
    APPROACH_PHASE_PREPARE_LEG,
    APPROACH_PHASE_VALIDATE_LEG,
    APPROACH_PHASE_WAIT_PERMISSION,
    APPROACH_PHASE_EXECUTE_LEG,
    APPROACH_PHASE_PAUSED,
    APPROACH_PHASE_VERIFY_TARGET,
    APPROACH_PHASE_COMPLETE,
    APPROACH_PHASE_ABORTED,
    APPROACH_PHASE_FAILED
} ApproachPhase;

typedef enum {
    APPROACH_ERR_NONE = 0,
    APPROACH_ERR_NULL_ARGUMENT,
    APPROACH_ERR_INVALID_CONFIG,
    APPROACH_ERR_INVALID_REQUEST,
    APPROACH_ERR_TRAJECTORY_NOT_VALID,
    APPROACH_ERR_TRAJECTORY_MISMATCH,
    APPROACH_ERR_EMPTY_TRAJECTORY,
    APPROACH_ERR_STORAGE_READ,
    APPROACH_ERR_POSITION_CONVERSION,
    APPROACH_ERR_TARGET_LIMIT,
    APPROACH_ERR_FEEDBACK,
    APPROACH_ERR_DRIVE_NOT_READY,
    APPROACH_ERR_PLAN_LIMIT,
    APPROACH_ERR_COLLISION,
    APPROACH_ERR_COLLISION_CHECK_MISSING,
    APPROACH_ERR_MOTION_TIMEOUT,
    APPROACH_ERR_VERIFY_TIMEOUT,
    APPROACH_ERR_FOLLOWING_ERROR,
    APPROACH_ERR_BUS,
    APPROACH_ERR_EXTERNAL_FAULT,
    APPROACH_ERR_ESTOP,
    APPROACH_ERR_RESET_REQUESTED,
    APPROACH_ERR_HOME_REQUESTED
} ApproachError;

typedef enum {
    APPROACH_RESULT_NONE = 0,
    APPROACH_RESULT_RUNNING,
    APPROACH_RESULT_COMPLETE,
    APPROACH_RESULT_ABORTED,
    APPROACH_RESULT_FAILED
} ApproachResult;

/* Minimum trajectory metadata needed by Approach. It can be populated from
 * ValidatedTrajectory without coupling this state to the validation module. */
typedef struct {
    bool valid;
    uint32_t program_id;
    uint32_t source_revision;
    uint32_t artifact_crc;
    uint32_t sample_data_crc;
    uint32_t sample_count;
    uint32_t sample_period_us;
} ApproachTrajectoryInfo;

typedef struct {
    int32_t target_position_units[APPROACH_DOF];
} ApproachExecutionSample;

typedef struct {
    ApproachOperation operation;
    const ApproachTrajectoryInfo *trajectory;
    uint32_t expected_program_id;
    uint32_t expected_source_revision;
    uint32_t expected_artifact_crc;

    /* Optional intermediate joint configurations. Use only poses that have
     * been checked against the installed cell model. The final destination is
     * always sample zero of the validated trajectory. */
    const float (*clearance_pose_rad)[APPROACH_DOF];
    uint8_t clearance_pose_count;
} ApproachRequest;

typedef struct {
    bool motion_permission;
    bool pause_requested;
    bool protective_stop_active;
    bool reset_requested;
    bool home_requested;
    bool estop_active;
    bool external_fault_active;
} ApproachControlInputs;

typedef struct {
    float joint_min_rad[APPROACH_DOF];
    float joint_max_rad[APPROACH_DOF];
    float velocity_max_rad_s[APPROACH_DOF];
    float acceleration_max_rad_s2[APPROACH_DOF];
    float jerk_max_rad_s3[APPROACH_DOF];
    bool use_acceleration_limits;
    bool use_jerk_limits;

    float duration_safety_factor;
    float minimum_leg_duration_s;
    float maximum_leg_duration_s;
    float final_position_tolerance_rad;
    float following_error_limit_rad;
    uint32_t required_stable_cycles;
    uint32_t maximum_verification_cycles;
    uint16_t validation_samples_per_step;
    bool require_collision_check;
} ApproachConfig;

typedef bool (*ApproachReadFirstSampleFn)(ApproachExecutionSample *sample,
                                          void *context);
typedef bool (*ApproachUnitsToRadFn)(const int32_t units[APPROACH_DOF],
                                     float joints_rad[APPROACH_DOF],
                                     void *context);
typedef bool (*ApproachRadToUnitsFn)(const float joints_rad[APPROACH_DOF],
                                     int32_t units[APPROACH_DOF],
                                     void *context);
typedef bool (*ApproachReadFeedbackFn)(float actual_rad[APPROACH_DOF],
                                       bool drive_ready[APPROACH_DOF],
                                       void *context);
typedef bool (*ApproachWriteTargetsFn)(const int32_t units[APPROACH_DOF],
                                       void *context);
typedef bool (*ApproachBusExchangeFn)(void *context);
typedef bool (*ApproachCollisionFreeFn)(const float joints_rad[APPROACH_DOF],
                                        void *context);

typedef struct {
    ApproachReadFirstSampleFn read_first_sample;
    ApproachUnitsToRadFn units_to_rad;
    ApproachRadToUnitsFn rad_to_units;
    ApproachReadFeedbackFn read_feedback;
    ApproachWriteTargetsFn write_targets;
    ApproachBusExchangeFn exchange_bus;
    ApproachCollisionFreeFn collision_free;
    void *context;
} ApproachServices;

typedef struct {
    ApproachPhase phase;
    ApproachResult result;
    ApproachError error;
    uint8_t failed_joint; /* 0 = none/global, 1..6 = joint number */
    uint8_t active_leg;
    uint8_t total_legs;
    uint32_t sample_index;
    uint32_t leg_intervals;
    uint32_t samples_sent;
    uint32_t stable_cycles;
    uint32_t verification_cycles;
    uint32_t replan_count;
    float leg_duration_s;
    float progress_0_to_1;
    float q_start_rad[APPROACH_DOF];
    float q_goal_rad[APPROACH_DOF];
    float final_target_rad[APPROACH_DOF];
    int32_t final_target_units[APPROACH_DOF];
} ApproachReport;

typedef struct {
    ApproachPhase phase;
    ApproachResult result;
    ApproachError error;
    uint8_t failed_joint;

    ApproachRequest request;
    ApproachConfig config;
    ApproachServices services;

    uint8_t active_leg;
    uint8_t total_legs;
    uint32_t sample_index;
    uint32_t leg_intervals;
    uint32_t validation_index;
    uint32_t samples_sent;
    uint32_t stable_cycles;
    uint32_t verification_cycles;
    uint32_t replan_count;
    float leg_duration_s;
    float route_pose_rad[APPROACH_MAX_LEGS + 1U][APPROACH_DOF];
    float route_duration_s[APPROACH_MAX_LEGS];
    uint32_t route_intervals[APPROACH_MAX_LEGS];
    float q_start_rad[APPROACH_DOF];
    float q_goal_rad[APPROACH_DOF];
    float final_target_rad[APPROACH_DOF];
    float last_command_rad[APPROACH_DOF];
    int32_t final_target_units[APPROACH_DOF];
    bool last_command_valid;
    bool paused_during_verification;
} ApproachState;

#ifdef __cplusplus
}
#endif
#endif
