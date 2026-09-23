#include "approach_project_adapter.h"

#include <limits.h>
#include <math.h>
#include <string.h>

static bool adapter_read_first(ApproachExecutionSample *sample, void *context)
{
    ApproachProjectAdapter *adapter = (ApproachProjectAdapter *)context;
    PvExecutionSample pv_sample;
    if (adapter == NULL || sample == NULL || adapter->read_pv_sample == NULL ||
        !adapter->read_pv_sample(0U, &pv_sample, adapter->storage_context)) {
        return false;
    }
    memcpy(sample->target_position_units, pv_sample.target_position_units,
           sizeof(sample->target_position_units));
    return true;
}

static bool adapter_units_to_rad(const int32_t units[APPROACH_DOF],
                                 float radians[APPROACH_DOF], void *context)
{
    ApproachProjectAdapter *adapter = (ApproachProjectAdapter *)context;
    uint32_t j;
    if (adapter == NULL || units == NULL || radians == NULL ||
        adapter->path_config == NULL) {
        return false;
    }
    for (j = 0U; j < APPROACH_DOF; ++j) {
        const float scale = adapter->path_config->drive_units_per_joint_rad[j];
        const int8_t sign = adapter->path_config->joint_direction_sign[j];
        if (!isfinite(scale) || scale <= 0.0f || (sign != 1 && sign != -1)) {
            return false;
        }
        radians[j] = (float)sign *
            ((float)units[j] -
             (float)adapter->path_config->drive_zero_offset_units[j]) / scale;
        if (!isfinite(radians[j])) {
            return false;
        }
    }
    return true;
}

static bool adapter_rad_to_units(const float radians[APPROACH_DOF],
                                 int32_t units[APPROACH_DOF], void *context)
{
    ApproachProjectAdapter *adapter = (ApproachProjectAdapter *)context;
    uint32_t j;
    if (adapter == NULL || units == NULL || radians == NULL ||
        adapter->path_config == NULL) {
        return false;
    }
    for (j = 0U; j < APPROACH_DOF; ++j) {
        const float scale = adapter->path_config->drive_units_per_joint_rad[j];
        const int8_t sign = adapter->path_config->joint_direction_sign[j];
        double raw;
        if (!isfinite(radians[j]) || !isfinite(scale) || scale <= 0.0f ||
            (sign != 1 && sign != -1)) {
            return false;
        }
        raw = (double)adapter->path_config->drive_zero_offset_units[j] +
              (double)sign * (double)radians[j] * (double)scale;
        if (raw < (double)INT32_MIN || raw > (double)INT32_MAX) {
            return false;
        }
        units[j] = (int32_t)llround(raw);
    }
    return true;
}

static bool adapter_read_feedback(float actual_rad[APPROACH_DOF],
                                  bool drive_ready[APPROACH_DOF], void *context)
{
    ApproachProjectAdapter *adapter = (ApproachProjectAdapter *)context;
    int32_t actual_units[APPROACH_DOF];
    if (adapter == NULL || adapter->read_drive == NULL ||
        !adapter->read_drive(actual_units, drive_ready,
                             adapter->drive_context)) {
        return false;
    }
    return adapter_units_to_rad(actual_units, actual_rad, adapter);
}

static bool adapter_write(const int32_t units[APPROACH_DOF], void *context)
{
    ApproachProjectAdapter *adapter = (ApproachProjectAdapter *)context;
    return adapter != NULL && adapter->write_drive != NULL &&
           adapter->write_drive(units, adapter->drive_context);
}

static bool adapter_exchange(void *context)
{
    ApproachProjectAdapter *adapter = (ApproachProjectAdapter *)context;
    return adapter != NULL && adapter->exchange != NULL &&
           adapter->exchange(adapter->drive_context);
}

static bool adapter_collision(const float q[APPROACH_DOF], void *context)
{
    ApproachProjectAdapter *adapter = (ApproachProjectAdapter *)context;
    return adapter != NULL && adapter->collision_free != NULL &&
           adapter->collision_free(q, adapter->collision_context);
}

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
    void *collision_context)
{
    if (adapter == NULL || trajectory == NULL || path_config == NULL ||
        read_pv_sample == NULL || read_drive == NULL || write_drive == NULL ||
        exchange == NULL) {
        return false;
    }
    memset(adapter, 0, sizeof(*adapter));
    adapter->trajectory = trajectory;
    adapter->path_config = path_config;
    adapter->committed_artifact_valid = committed_artifact_valid;
    adapter->read_pv_sample = read_pv_sample;
    adapter->storage_context = storage_context;
    adapter->read_drive = read_drive;
    adapter->write_drive = write_drive;
    adapter->exchange = exchange;
    adapter->drive_context = drive_context;
    adapter->collision_free = collision_free;
    adapter->collision_context = collision_context;

    adapter->approach_info.valid = committed_artifact_valid;
    adapter->approach_info.program_id = trajectory->program_id;
    adapter->approach_info.source_revision = trajectory->source_revision;
    adapter->approach_info.artifact_crc = trajectory->artifact_crc;
    adapter->approach_info.sample_data_crc = trajectory->sample_data_crc;
    adapter->approach_info.sample_count = trajectory->sample_count;
    adapter->approach_info.sample_period_us = trajectory->sample_period_us;
    return true;
}

bool ApproachProjectAdapter_BuildRequest(
    ApproachProjectAdapter *adapter,
    ApproachOperation operation,
    const float (*clearance_pose_rad)[APPROACH_DOF],
    uint8_t clearance_pose_count,
    ApproachRequest *request)
{
    if (adapter == NULL || request == NULL ||
        (operation != APPROACH_OPERATION_PREVIEW &&
         operation != APPROACH_OPERATION_WELD) ||
        clearance_pose_count > APPROACH_MAX_CLEARANCE_POSES ||
        (clearance_pose_count > 0U && clearance_pose_rad == NULL)) {
        return false;
    }
    memset(request, 0, sizeof(*request));
    request->operation = operation;
    request->trajectory = &adapter->approach_info;
    request->expected_program_id = adapter->approach_info.program_id;
    request->expected_source_revision = adapter->approach_info.source_revision;
    request->expected_artifact_crc = adapter->approach_info.artifact_crc;
    request->clearance_pose_rad = clearance_pose_rad;
    request->clearance_pose_count = clearance_pose_count;
    return true;
}

bool ApproachProjectAdapter_BuildServices(
    ApproachProjectAdapter *adapter,
    ApproachServices *services)
{
    if (adapter == NULL || services == NULL) {
        return false;
    }
    memset(services, 0, sizeof(*services));
    services->read_first_sample = adapter_read_first;
    services->units_to_rad = adapter_units_to_rad;
    services->rad_to_units = adapter_rad_to_units;
    services->read_feedback = adapter_read_feedback;
    services->write_targets = adapter_write;
    services->exchange_bus = adapter_exchange;
    services->collision_free = adapter->collision_free != NULL ?
                               adapter_collision : NULL;
    services->context = adapter;
    return true;
}

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
    ApproachConfig *config)
{
    uint32_t j;
    if (path_config == NULL || config == NULL ||
        (use_jerk_limits && jerk_max_rad_s3 == NULL)) {
        return false;
    }
    memset(config, 0, sizeof(*config));
    for (j = 0U; j < APPROACH_DOF; ++j) {
        config->joint_min_rad[j] = path_config->joint_min_rad[j];
        config->joint_max_rad[j] = path_config->joint_max_rad[j];
        config->velocity_max_rad_s[j] =
            path_config->joint_velocity_max_rad_s[j];
        config->acceleration_max_rad_s2[j] =
            path_config->joint_acceleration_max_rad_s2[j];
        config->jerk_max_rad_s3[j] = use_jerk_limits ?
                                     jerk_max_rad_s3[j] : 0.0f;
    }
    config->use_acceleration_limits = path_config->check_joint_acceleration;
    config->use_jerk_limits = use_jerk_limits;
    config->duration_safety_factor = duration_safety_factor;
    config->minimum_leg_duration_s = minimum_leg_duration_s;
    config->maximum_leg_duration_s = maximum_leg_duration_s;
    config->final_position_tolerance_rad = final_position_tolerance_rad;
    config->following_error_limit_rad = following_error_limit_rad;
    config->required_stable_cycles = required_stable_cycles;
    config->maximum_verification_cycles = maximum_verification_cycles;
    config->validation_samples_per_step = validation_samples_per_step;
    config->require_collision_check = require_collision_check;
    return true;
}
