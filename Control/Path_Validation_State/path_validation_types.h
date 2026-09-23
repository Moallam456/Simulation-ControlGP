#ifndef PATH_VALIDATION_TYPES_H
#define PATH_VALIDATION_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "teaching_state_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PV_DOF                     TEACHING_DOF
/* CSP execution is intentionally fixed to the EtherCAT control cycle. */
#define PV_SAMPLE_PERIOD_US        (1000UL)
#define PV_SAMPLE_PERIOD_S         (0.001F)

typedef enum {
    PV_PHASE_IDLE = 0,
    PV_PHASE_SNAPSHOT_CHECK,
    PV_PHASE_STRUCTURAL_CHECK,
    PV_PHASE_PREPARE_SEGMENT,
    PV_PHASE_GENERATE_AND_VALIDATE_SAMPLE,
    PV_PHASE_FINALIZE,
    PV_PHASE_VALID,
    PV_PHASE_INVALID
} PathValidationPhase;

typedef enum {
    PV_ERR_NONE = 0,
    PV_ERR_NULL_ARGUMENT,
    PV_ERR_BUSY,
    PV_ERR_EMPTY_PROGRAM,
    PV_ERR_REVISION_MISMATCH,
    PV_ERR_DRAFT_CRC_MISMATCH,
    PV_ERR_UNSUPPORTED_SEGMENT,
    PV_ERR_INCOMPLETE_SEGMENT,
    PV_ERR_INVALID_POINT,
    PV_ERR_DEGENERATE_GEOMETRY,
    PV_ERR_ARC_DIRECTION,
    PV_ERR_INVALID_CIRCLE_DIRECTION,
    PV_ERR_FRAME_MISMATCH,
    PV_ERR_TOOL_MISMATCH,
    PV_ERR_CALIBRATION_MISMATCH,
    PV_ERR_INVALID_PARAMETER,
    PV_ERR_ZERO_LENGTH_SEGMENT,
    PV_ERR_SAMPLE_CAPACITY,
    PV_ERR_IK_FAILED,
    PV_ERR_FK_POSITION,
    PV_ERR_FK_ORIENTATION,
    PV_ERR_JOINT_POSITION,
    PV_ERR_JOINT_VELOCITY,
    PV_ERR_JOINT_ACCELERATION,
    PV_ERR_JOINT_DISCONTINUITY,
    PV_ERR_POSITION_CONVERSION,
    PV_ERR_SINGULARITY_MARGIN,
    PV_ERR_COLLISION,
    PV_ERR_STORAGE,
    PV_ERR_CALLBACK_MISSING,
    PV_ERR_CANCELLED
} PathValidationError;

typedef enum {
    PV_RESULT_NONE = 0,
    PV_RESULT_RUNNING,
    PV_RESULT_VALID,
    PV_RESULT_INVALID,
    PV_RESULT_CANCELLED
} PathValidationResult;

typedef struct {
    float position_m[3];
    float orientation_quat[4]; /* [w x y z] */
} PvCartesianPose;

/* Compact CSP command consumed by Preview/Welding every 1 ms. The unit is the
 * signed 32-bit position unit expected by CiA 402 object 0x607A. No timestamp
 * is stored because sample index * PV_SAMPLE_PERIOD_US is the execution time. */
typedef struct {
    int32_t target_position_units[PV_DOF];
} PvExecutionSample;

/* Maps a taught segment to its contiguous range in samples[]. */
typedef struct {
    uint32_t first_sample;
    uint32_t sample_count;
    uint8_t segment_type;
    uint8_t reserved;
} PvSegmentIndex;

typedef struct {
    uint32_t program_id;
    uint32_t source_revision;
    uint32_t source_crc;
    uint32_t artifact_crc;
    uint32_t sample_data_crc;
    uint32_t sample_count;
    uint16_t segment_count;
    uint32_t sample_period_us;
    float duration_s;
    float path_length_m;
    PvSegmentIndex segments[TEACHING_MAX_SEGMENTS];
} ValidatedTrajectory;

/* Storage is deliberately abstract: the host test can use RAM now, while a
 * later STM32 integration can buffer these writes into the selected external
 * NOR flash without changing Path Validation. */
typedef bool (*PvStorageBeginFn)(void *context);
typedef bool (*PvStorageWriteFn)(uint32_t sample_index,
                                 const PvExecutionSample *sample,
                                 void *context);
typedef bool (*PvStorageCommitFn)(const ValidatedTrajectory *metadata,
                                  void *context);
typedef void (*PvStorageAbortFn)(void *context);

typedef struct {
    PvStorageBeginFn begin;
    PvStorageWriteFn write_sample;
    PvStorageCommitFn commit;
    PvStorageAbortFn abort;
    uint32_t capacity_samples;
    void *context;
} PathValidationStorage;

typedef struct {
    float default_tcp_speed_mps;
    float max_tcp_speed_mps;
    float max_tcp_acceleration_mps2;
    float max_tcp_jerk_mps3;
    float minimum_segment_length_m;
    float maximum_fk_position_error_m;
    float maximum_fk_orientation_error_rad;
    float minimum_singularity_sigma;
    float maximum_joint_step_rad;
    float joint_min_rad[PV_DOF];
    float joint_max_rad[PV_DOF];
    float joint_velocity_max_rad_s[PV_DOF];
    float joint_acceleration_max_rad_s2[PV_DOF];
    /* Conversion between model joint radians and each drive's configured
     * signed CSP position units. Derive these values from encoder resolution,
     * electronic gearing, mechanical gearbox ratio and joint direction. */
    float drive_units_per_joint_rad[PV_DOF];
    int32_t drive_zero_offset_units[PV_DOF];
    int8_t joint_direction_sign[PV_DOF]; /* must be +1 or -1 */
    float maximum_position_quantization_error_rad;
    bool check_joint_acceleration;
    bool require_collision_callback;
} PathValidationConfig;

typedef struct {
    bool converged;
    uint16_t iterations;
    float position_error_m;
    float orientation_error_rad;
    float sigma_min;
} PvIkDiagnostics;

typedef bool (*PvInverseKinematicsFn)(const PvCartesianPose *target,
                                      const float seed_rad[PV_DOF],
                                      float solution_rad[PV_DOF],
                                      PvIkDiagnostics *diagnostics,
                                      void *user_context);

typedef bool (*PvForwardKinematicsFn)(const float joints_rad[PV_DOF],
                                      PvCartesianPose *achieved,
                                      void *user_context);

typedef bool (*PvCollisionCheckFn)(const float joints_rad[PV_DOF],
                                   uint16_t source_segment,
                                   uint32_t sample_index,
                                   void *user_context);

typedef struct {
    PvInverseKinematicsFn inverse_kinematics;
    PvForwardKinematicsFn forward_kinematics;
    PvCollisionCheckFn collision_free;
    void *user_context;
} PathValidationServices;

typedef struct {
    PathValidationPhase phase;
    PathValidationResult result;
    PathValidationError error;
    uint16_t failed_segment;
    uint32_t failed_sample;
    uint8_t failed_joint;
    float progress_0_to_1;
    float max_fk_position_error_m;
    float max_fk_orientation_error_rad;
    float minimum_sigma_seen;
    float peak_joint_velocity_rad_s[PV_DOF];
    float peak_joint_acceleration_rad_s2[PV_DOF];
} PathValidationReport;

typedef struct {
    PathValidationPhase phase;
    PathValidationResult result;
    PathValidationError error;
    PathValidationConfig config;
    PathValidationServices services;
    PathValidationStorage storage;
    const TaughtProgram *source_program;
    uint32_t expected_revision;
    uint32_t expected_crc;
    uint16_t segment_index;
    uint32_t sample_index;
    uint32_t segment_local_sample;
    uint32_t segment_sample_count;
    float segment_start_time_s;
    float segment_length_m;
    float segment_duration_s;
    float s_curve_tj_s;
    float s_curve_ta_s;
    float s_curve_tv_s;
    float s_curve_jerk_norm;
    /* Prepared geometry for LINE/ARC/CIRCLE. */
    float geometry_center_m[3];
    float geometry_axis[3];
    float geometry_e1[3];
    float geometry_e2[3];
    float geometry_radius_m;
    float geometry_theta_total_rad;
    float geometry_q_start[4];
    float geometry_q_end[4];
    float seed_rad[PV_DOF];
    /* Validation-only derivative history. It is deliberately not retained in
     * the compact Preview/Welding artifact. */
    float previous_q_rad[PV_DOF];
    float previous_qd_rad_s[PV_DOF];
    float previous_time_s;
    uint32_t sample_crc_state;
    bool previous_q_valid;
    bool previous_qd_valid;
    ValidatedTrajectory *artifact;
    PathValidationReport report;
} PathValidationState;

#ifdef __cplusplus
}
#endif
#endif
