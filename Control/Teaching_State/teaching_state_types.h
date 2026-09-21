#ifndef TEACHING_STATE_TYPES_H
#define TEACHING_STATE_TYPES_H

/*
 * Portable data types for the robot Teaching state.
 *
 * This file intentionally has no STM32 HAL dependency. The application layer
 * may use it from either STM32 core, unit tests, or a desktop simulator.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TEACHING_DOF                 (6U)
#define TEACHING_MAX_SEGMENTS        (32U)
#define TEACHING_MAX_POINTS_PER_SEG  (3U)

typedef enum
{
    TEACH_SEGMENT_NONE = 0,
    TEACH_SEGMENT_LINE,
    TEACH_SEGMENT_ARC,
    TEACH_SEGMENT_CIRCLE
} TeachingSegmentType;

typedef enum
{
    TEACH_CIRCLE_DIRECTION_UNSPECIFIED = 0,
    TEACH_CIRCLE_CCW = 1,
    TEACH_CIRCLE_CW = -1
} TeachingCircleDirection;

typedef enum
{
    TEACH_ORIENTATION_CONSTANT = 0,
    TEACH_ORIENTATION_INTERPOLATED
} TeachingOrientationMode;

typedef enum
{
    TEACH_PHASE_WAIT_SEGMENT_SELECTION = 0,
    TEACH_PHASE_WAIT_POINT,
    TEACH_PHASE_SEGMENT_COMPLETE,
    TEACH_PHASE_SUBMITTING_DRAFT
} TeachingPhase;

typedef enum
{
    TEACH_DRAFT_EMPTY = 0,
    TEACH_DRAFT_EDITING,
    TEACH_DRAFT_COMPLETE,
    TEACH_DRAFT_SUBMITTED
} TeachingDraftStatus;

typedef enum
{
    TEACH_ERR_NONE = 0,
    TEACH_ERR_NULL_ARGUMENT,
    TEACH_ERR_NO_SEGMENT_SELECTED,
    TEACH_ERR_ROBOT_NOT_HOMED,
    TEACH_ERR_DRIVES_NOT_READY,
    TEACH_ERR_ROBOT_NOT_SETTLED,
    TEACH_ERR_GUIDANCE_INACTIVE,
    TEACH_ERR_MOTION_NOT_PERMITTED,
    TEACH_ERR_SAFETY_ACTIVE,
    TEACH_ERR_GLOBAL_FAULT_ACTIVE,
    TEACH_ERR_DUPLICATE_POINT,
    TEACH_ERR_COLLINEAR_POINTS,
    TEACH_ERR_SEGMENT_INCOMPLETE,
    TEACH_ERR_DRAFT_EMPTY,
    TEACH_ERR_SEGMENT_CAPACITY,
    TEACH_ERR_INVALID_SPEED,
    TEACH_ERR_INVALID_MEASUREMENT,
    TEACH_ERR_FRAME_CHANGED,
    TEACH_ERR_TOOL_CHANGED,
    TEACH_ERR_CALIBRATION_CHANGED,
    TEACH_ERR_DRAFT_LOCKED
} TeachingError;

typedef enum
{
    TEACH_MSG_SELECT_SEGMENT = 0,
    TEACH_MSG_MOVE_TO_POINT,
    TEACH_MSG_POINT_RECORDED,
    TEACH_MSG_SEGMENT_RECORDED,
    TEACH_MSG_DRAFT_RESET,
    TEACH_MSG_DRAFT_SUBMITTED,
    TEACH_MSG_ACTION_REJECTED
} TeachingMessageId;

typedef enum
{
    TEACH_EVENT_NONE = 0,
    TEACH_EVENT_SELECT_LINE,
    TEACH_EVENT_SELECT_ARC,
    TEACH_EVENT_SELECT_CIRCLE,
    TEACH_EVENT_RECORD_POINT,
    TEACH_EVENT_VALIDATE_PATH,
    TEACH_EVENT_SPEED_INCREASE,
    TEACH_EVENT_SPEED_DECREASE,
    TEACH_EVENT_SPEED_DEFAULT,
    TEACH_EVENT_RESET
} TeachingEvent;

typedef struct
{
    float position_m[3];
    float orientation_quat[4]; /* [w, x, y, z] */
    float joint_position_rad[TEACHING_DOF];

    uint32_t record_timestamp_ms;
    uint32_t calibration_version;
    uint16_t frame_id;
    uint16_t tool_id;
    bool point_valid;
} TaughtPoint;

typedef struct
{
    uint16_t segment_id;
    TeachingSegmentType type;
    uint8_t point_count;
    TaughtPoint points[TEACHING_MAX_POINTS_PER_SEG];

    float speed_mps;
    TeachingCircleDirection circle_direction;
    TeachingOrientationMode orientation_mode;
    bool segment_valid; /* Structurally complete; not feasibility-validated. */
} TaughtSegment;

typedef struct
{
    uint32_t program_id;
    uint32_t draft_revision;
    uint32_t draft_crc;
    uint16_t segment_count;
    TaughtSegment segments[TEACHING_MAX_SEGMENTS];
    float global_speed_scale;
    TeachingDraftStatus status;
} TaughtProgram;

/* Snapshot owned by motion/safety/configuration modules and read by Teaching. */
typedef struct
{
    float actual_position_m[3];
    float actual_orientation_quat[4];
    float actual_joint_position_rad[TEACHING_DOF];

    uint32_t timestamp_ms;
    uint32_t calibration_version;
    uint16_t active_frame_id;
    uint16_t active_tool_id;

    bool measurement_valid;
    bool robot_motion_settled;
    bool manual_guidance_active;
    bool motion_permitted;
    bool estop_active;
    bool protective_stop_active;
    bool global_fault_active;
    bool drives_ready;
    bool robot_homed;
} TeachingInputs;

typedef struct
{
    float default_speed_mps;
    float minimum_speed_mps;
    float maximum_speed_mps;
    float speed_step_mps;
    float minimum_point_separation_m;
    float collinearity_epsilon_m2;
} TeachingConfig;

/* One-cycle outputs. The caller clears/consumes these after each update. */
typedef struct
{
    bool validation_request;
    uint32_t submitted_program_id;
    uint32_t submitted_revision;
    uint32_t submitted_crc;

    TeachingMessageId status_message_id;
    TeachingError error;
    TeachingPhase phase;
    uint16_t current_segment_number;
    uint8_t next_point_number;
    float displayed_speed_mps;
    bool record_allowed;
    bool validate_allowed;
    bool draft_was_reset;
} TeachingOutputs;

typedef struct
{
    TeachingPhase phase;
    TeachingError error;
    TeachingMessageId status_message_id;

    TeachingSegmentType selected_segment_type;
    TeachingCircleDirection selected_circle_direction;
    TeachingOrientationMode selected_orientation_mode;
    uint8_t required_point_count;
    uint8_t captured_point_count;
    uint16_t active_segment_index;

    float teaching_speed_mps;
    bool draft_dirty;
    bool teaching_complete;

    TaughtSegment working_segment;
    TaughtProgram draft;
    TeachingConfig config;
} TeachingState;

#ifdef __cplusplus
}
#endif

#endif /* TEACHING_STATE_TYPES_H */
