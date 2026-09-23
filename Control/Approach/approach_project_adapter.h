#ifndef APPROACH_PROJECT_ADAPTER_H
#define APPROACH_PROJECT_ADAPTER_H

#include "approach_state.h"
#include "path_validation_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Read one committed Path Validation sample. The production implementation
 * may read external NOR flash; a RAM backend is sufficient for host tests. */
typedef bool (*ApproachProjectReadPvSampleFn)(uint32_t sample_index,
                                              PvExecutionSample *sample,
                                              void *context);

/* Hardware-facing callbacks deliberately use native CiA 402 position units.
 * The adapter applies the same axis scale, direction and zero offset used by
 * Path Validation, preventing mismatched conversions between states. */
typedef bool (*ApproachProjectReadDriveFn)(
    int32_t actual_position_units[APPROACH_DOF],
    bool csp_operation_enabled[APPROACH_DOF],
    void *context);

typedef bool (*ApproachProjectWriteDriveFn)(
    const int32_t target_position_units[APPROACH_DOF],
    void *context);

typedef bool (*ApproachProjectExchangeFn)(void *context);

typedef bool (*ApproachProjectCollisionFn)(
    const float joints_rad[APPROACH_DOF],
    void *context);

typedef struct {
    const ValidatedTrajectory *trajectory;
    const PathValidationConfig *path_config;
    bool committed_artifact_valid;

    ApproachProjectReadPvSampleFn read_pv_sample;
    void *storage_context;

    ApproachProjectReadDriveFn read_drive;
    ApproachProjectWriteDriveFn write_drive;
    ApproachProjectExchangeFn exchange;
    void *drive_context;

    ApproachProjectCollisionFn collision_free;
    void *collision_context;

    ApproachTrajectoryInfo approach_info;
} ApproachProjectAdapter;

bool ApproachProjectAdapter_Init(
    ApproachProjectAdapter *adapter,
    const ValidatedTrajectory *trajectory,
    const PathValidationConfig *path_config,
    bool committed_artifact_valid,
    ApproachProjectReadPvSampleFn read_pv_sample,
    void *storage_context,
    ApproachProjectReadDriveFn read_drive,
    ApproachProjectWriteDriveFn write_drive,
    ApproachProjectExchangeFn exchange,
    void *drive_context,
    ApproachProjectCollisionFn collision_free,
    void *collision_context);

/* Builds the request passed to ApproachState_Enter(). Button interpretation
 * and permission to enter Approach remain supervisor responsibilities. */
bool ApproachProjectAdapter_BuildRequest(
    ApproachProjectAdapter *adapter,
    ApproachOperation operation,
    const float (*clearance_pose_rad)[APPROACH_DOF],
    uint8_t clearance_pose_count,
    ApproachRequest *request);

bool ApproachProjectAdapter_BuildServices(
    ApproachProjectAdapter *adapter,
    ApproachServices *services);

/* Copies limits and conversion-independent policy from Path Validation.
 * Approach-only tolerances and jerk limits remain explicit arguments. */
bool ApproachProjectAdapter_BuildConfig(
    const PathValidationConfig *path_config,
    const float jerk_max_rad_s3[APPROACH_DOF],
    bool use_jerk_limits,
    float duration_safety_factor,
    float minimum_leg_duration_s,
    float maximum_leg_duration_s,
    float final_position_tolerance_rad,
    float following_error_limit_rad,
    uint32_t required_stable_cycles,
    uint32_t maximum_verification_cycles,
    uint16_t validation_samples_per_step,
    bool require_collision_check,
    ApproachConfig *config);

#ifdef __cplusplus
}
#endif
#endif
