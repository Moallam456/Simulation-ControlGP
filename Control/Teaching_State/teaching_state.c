#include "teaching_state.h"
#include "teaching_debug.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TEACHING_CRC32_POLYNOMIAL (0xEDB88320UL)

static void clear_outputs(TeachingOutputs *outputs)
{
    memset(outputs, 0, sizeof(*outputs));
}

static uint8_t required_points(TeachingSegmentType type)
{
    switch (type)
    {
        case TEACH_SEGMENT_LINE:   return 2U;
        case TEACH_SEGMENT_ARC:    return 3U;
        case TEACH_SEGMENT_CIRCLE: return 3U;
        default:                   return 0U;
    }
}

static bool finite_array(const float *values, size_t count)
{
    size_t i;
    for (i = 0U; i < count; ++i)
    {
        if (!isfinite(values[i]))
        {
            return false;
        }
    }
    return true;
}

static float squared_distance3(const float a[3], const float b[3])
{
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return (dx * dx) + (dy * dy) + (dz * dz);
}

static float cross_squared(const float p1[3],
                           const float p2[3],
                           const float p3[3])
{
    const float ax = p2[0] - p1[0];
    const float ay = p2[1] - p1[1];
    const float az = p2[2] - p1[2];
    const float bx = p3[0] - p1[0];
    const float by = p3[1] - p1[1];
    const float bz = p3[2] - p1[2];
    const float cx = (ay * bz) - (az * by);
    const float cy = (az * bx) - (ax * bz);
    const float cz = (ax * by) - (ay * bx);
    return (cx * cx) + (cy * cy) + (cz * cz);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
    size_t i;
    uint8_t bit;
    for (i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1U) ^ (TEACHING_CRC32_POLYNOMIAL & mask);
        }
    }
    return crc;
}

/*
 * CRC is calculated field-by-field to avoid compiler padding bytes. It is an
 * integrity check, not a safety-certified diagnostic by itself.
 */
static uint32_t calculate_draft_crc(const TaughtProgram *program)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t s;
    uint8_t p;

#define CRC_FIELD(field_) \
    do { crc = crc32_update(crc, (const uint8_t *)&(field_), sizeof(field_)); } while (0)

    CRC_FIELD(program->program_id);
    CRC_FIELD(program->draft_revision);
    CRC_FIELD(program->segment_count);
    CRC_FIELD(program->global_speed_scale);

    for (s = 0U; s < program->segment_count; ++s)
    {
        const TaughtSegment *segment = &program->segments[s];
        CRC_FIELD(segment->segment_id);
        CRC_FIELD(segment->type);
        CRC_FIELD(segment->point_count);
        CRC_FIELD(segment->speed_mps);
        CRC_FIELD(segment->circle_direction);
        CRC_FIELD(segment->orientation_mode);

        for (p = 0U; p < segment->point_count; ++p)
        {
            const TaughtPoint *point = &segment->points[p];
            crc = crc32_update(crc, (const uint8_t *)point->position_m,
                               sizeof(point->position_m));
            crc = crc32_update(crc, (const uint8_t *)point->orientation_quat,
                               sizeof(point->orientation_quat));
            crc = crc32_update(crc, (const uint8_t *)point->joint_position_rad,
                               sizeof(point->joint_position_rad));
            CRC_FIELD(point->record_timestamp_ms);
            CRC_FIELD(point->calibration_version);
            CRC_FIELD(point->frame_id);
            CRC_FIELD(point->tool_id);
        }
    }

#undef CRC_FIELD
    return ~crc;
}

static void set_error(TeachingState *state, TeachingError error)
{
    state->error = error;
    state->status_message_id = TEACH_MSG_ACTION_REJECTED;
    TEACH_LOG_ERROR("Rejected: %s\r\n", TeachingState_ErrorName(error));
}

static bool config_is_valid(const TeachingConfig *config)
{
    return isfinite(config->default_speed_mps) &&
           isfinite(config->minimum_speed_mps) &&
           isfinite(config->maximum_speed_mps) &&
           isfinite(config->speed_step_mps) &&
           isfinite(config->minimum_point_separation_m) &&
           isfinite(config->collinearity_epsilon_m2) &&
           (config->minimum_speed_mps > 0.0F) &&
           (config->maximum_speed_mps >= config->minimum_speed_mps) &&
           (config->default_speed_mps >= config->minimum_speed_mps) &&
           (config->default_speed_mps <= config->maximum_speed_mps) &&
           (config->speed_step_mps > 0.0F) &&
           (config->minimum_point_separation_m > 0.0F) &&
           (config->collinearity_epsilon_m2 > 0.0F);
}

static void begin_segment(TeachingState *state, TeachingSegmentType type)
{
    memset(&state->working_segment, 0, sizeof(state->working_segment));
    state->selected_segment_type = type;
    state->required_point_count = required_points(type);
    state->captured_point_count = 0U;
    state->active_segment_index = state->draft.segment_count;
    state->working_segment.segment_id = (uint16_t)(state->draft.segment_count + 1U);
    state->working_segment.type = type;
    state->working_segment.speed_mps = state->teaching_speed_mps;
    state->working_segment.circle_direction = state->selected_circle_direction;
    state->working_segment.orientation_mode = state->selected_orientation_mode;
    state->phase = TEACH_PHASE_WAIT_POINT;
    state->draft.status = TEACH_DRAFT_EDITING;
    state->status_message_id = TEACH_MSG_MOVE_TO_POINT;
    state->error = TEACH_ERR_NONE;

    TEACH_LOG_INFO("Segment %u selected: %s, requires %u point(s), speed=%.4f m/s\r\n",
                   (unsigned)state->working_segment.segment_id,
                   TeachingState_SegmentName(type),
                   (unsigned)state->required_point_count,
                   (double)state->teaching_speed_mps);
}

static bool capture_is_permitted(TeachingState *state,
                                 const TeachingInputs *inputs)
{
    if (inputs->estop_active || inputs->protective_stop_active)
    {
        set_error(state, TEACH_ERR_SAFETY_ACTIVE);
        return false;
    }
    if (inputs->global_fault_active)
    {
        set_error(state, TEACH_ERR_GLOBAL_FAULT_ACTIVE);
        return false;
    }
    if (!inputs->robot_homed)
    {
        set_error(state, TEACH_ERR_ROBOT_NOT_HOMED);
        return false;
    }
    if (!inputs->drives_ready)
    {
        set_error(state, TEACH_ERR_DRIVES_NOT_READY);
        return false;
    }
    if (!inputs->motion_permitted)
    {
        set_error(state, TEACH_ERR_MOTION_NOT_PERMITTED);
        return false;
    }
    if (!inputs->manual_guidance_active)
    {
        set_error(state, TEACH_ERR_GUIDANCE_INACTIVE);
        return false;
    }
    if (!inputs->robot_motion_settled)
    {
        set_error(state, TEACH_ERR_ROBOT_NOT_SETTLED);
        return false;
    }
    if (!inputs->measurement_valid ||
        !finite_array(inputs->actual_position_m, 3U) ||
        !finite_array(inputs->actual_orientation_quat, 4U) ||
        !finite_array(inputs->actual_joint_position_rad, TEACHING_DOF))
    {
        set_error(state, TEACH_ERR_INVALID_MEASUREMENT);
        return false;
    }
    return true;
}

static bool check_program_identity(TeachingState *state,
                                   const TeachingInputs *inputs)
{
    const TaughtPoint *reference;

    if (state->draft.segment_count > 0U)
    {
        reference = &state->draft.segments[0].points[0];
    }
    else if (state->captured_point_count > 0U)
    {
        reference = &state->working_segment.points[0];
    }
    else
    {
        return true;
    }

    if (inputs->active_frame_id != reference->frame_id)
    {
        set_error(state, TEACH_ERR_FRAME_CHANGED);
        return false;
    }
    if (inputs->active_tool_id != reference->tool_id)
    {
        set_error(state, TEACH_ERR_TOOL_CHANGED);
        return false;
    }
    if (inputs->calibration_version != reference->calibration_version)
    {
        set_error(state, TEACH_ERR_CALIBRATION_CHANGED);
        return false;
    }
    return true;
}

static bool record_point(TeachingState *state, const TeachingInputs *inputs)
{
    TaughtPoint *point;
    uint8_t i;
    const float min_distance_sq = state->config.minimum_point_separation_m *
                                  state->config.minimum_point_separation_m;

    if (state->phase != TEACH_PHASE_WAIT_POINT ||
        state->selected_segment_type == TEACH_SEGMENT_NONE)
    {
        set_error(state, TEACH_ERR_NO_SEGMENT_SELECTED);
        return false;
    }
    if (!capture_is_permitted(state, inputs) ||
        !check_program_identity(state, inputs))
    {
        return false;
    }
    if ((state->captured_point_count > 0U) &&
        (squared_distance3(inputs->actual_position_m,
                           state->working_segment.points[state->captured_point_count - 1U].position_m)
         < min_distance_sq))
    {
        set_error(state, TEACH_ERR_DUPLICATE_POINT);
        return false;
    }

    point = &state->working_segment.points[state->captured_point_count];
    memcpy(point->position_m, inputs->actual_position_m, sizeof(point->position_m));
    memcpy(point->orientation_quat, inputs->actual_orientation_quat,
           sizeof(point->orientation_quat));
    memcpy(point->joint_position_rad, inputs->actual_joint_position_rad,
           sizeof(point->joint_position_rad));
    point->record_timestamp_ms = inputs->timestamp_ms;
    point->calibration_version = inputs->calibration_version;
    point->frame_id = inputs->active_frame_id;
    point->tool_id = inputs->active_tool_id;
    point->point_valid = true;

    ++state->captured_point_count;
    state->working_segment.point_count = state->captured_point_count;

    TEACH_LOG_INFO("Recorded %s segment=%u point=%u pos=[%.5f %.5f %.5f] m\r\n",
                   TeachingState_SegmentName(state->selected_segment_type),
                   (unsigned)state->working_segment.segment_id,
                   (unsigned)state->captured_point_count,
                   (double)point->position_m[0],
                   (double)point->position_m[1],
                   (double)point->position_m[2]);
    TEACH_LOG_INFO("Joint seed rad=[");
    for (i = 0U; i < TEACHING_DOF; ++i)
    {
        TEACHING_DEBUG_PRINTF("%.5f%s", (double)point->joint_position_rad[i],
                              (i + 1U < TEACHING_DOF) ? ", " : "]\r\n");
    }

    if (state->captured_point_count < state->required_point_count)
    {
        state->status_message_id = TEACH_MSG_POINT_RECORDED;
        state->error = TEACH_ERR_NONE;
        return true;
    }

    if ((state->selected_segment_type == TEACH_SEGMENT_ARC ||
         state->selected_segment_type == TEACH_SEGMENT_CIRCLE) &&
        (cross_squared(state->working_segment.points[0].position_m,
                       state->working_segment.points[1].position_m,
                       state->working_segment.points[2].position_m)
         <= state->config.collinearity_epsilon_m2))
    {
        /* Keep P1 and P2, discard only invalid P3 so the operator can retry. */
        memset(&state->working_segment.points[2], 0, sizeof(TaughtPoint));
        state->captured_point_count = 2U;
        state->working_segment.point_count = 2U;
        set_error(state, TEACH_ERR_COLLINEAR_POINTS);
        return false;
    }

    if (state->draft.segment_count >= TEACHING_MAX_SEGMENTS)
    {
        set_error(state, TEACH_ERR_SEGMENT_CAPACITY);
        return false;
    }

    state->working_segment.segment_valid = true;
    state->draft.segments[state->draft.segment_count] = state->working_segment;
    ++state->draft.segment_count;
    ++state->draft.draft_revision;
    state->draft.draft_crc = 0U; /* Recomputed only when the draft is frozen. */
    state->draft.status = TEACH_DRAFT_COMPLETE;
    state->draft_dirty = true;
    state->teaching_complete = true;
    state->phase = TEACH_PHASE_SEGMENT_COMPLETE;
    state->status_message_id = TEACH_MSG_SEGMENT_RECORDED;
    state->error = TEACH_ERR_NONE;

    TEACH_LOG_INFO("Segment committed; count=%u revision=%lu\r\n",
                   (unsigned)state->draft.segment_count,
                   (unsigned long)state->draft.draft_revision);
    return true;
}

static bool change_speed(TeachingState *state, float requested_speed)
{
    if (!isfinite(requested_speed))
    {
        set_error(state, TEACH_ERR_INVALID_SPEED);
        return false;
    }
    if (requested_speed < state->config.minimum_speed_mps)
    {
        requested_speed = state->config.minimum_speed_mps;
    }
    if (requested_speed > state->config.maximum_speed_mps)
    {
        requested_speed = state->config.maximum_speed_mps;
    }
    state->teaching_speed_mps = requested_speed;
    state->error = TEACH_ERR_NONE;

    /* A segment snapshots its speed when selected. */
    TEACH_LOG_INFO("Displayed speed changed to %.4f m/s\r\n",
                   (double)state->teaching_speed_mps);
    return true;
}

static bool submit_for_validation(TeachingState *state, TeachingOutputs *outputs)
{
    if (state->phase == TEACH_PHASE_WAIT_POINT && state->captured_point_count > 0U)
    {
        set_error(state, TEACH_ERR_SEGMENT_INCOMPLETE);
        return false;
    }
    if (state->draft.segment_count == 0U)
    {
        set_error(state, TEACH_ERR_DRAFT_EMPTY);
        return false;
    }

    state->draft.status = TEACH_DRAFT_SUBMITTED;
    state->draft.draft_crc = calculate_draft_crc(&state->draft);
    state->draft_dirty = false;
    state->phase = TEACH_PHASE_SUBMITTING_DRAFT;
    state->status_message_id = TEACH_MSG_DRAFT_SUBMITTED;
    state->error = TEACH_ERR_NONE;

    outputs->validation_request = true;
    outputs->submitted_program_id = state->draft.program_id;
    outputs->submitted_revision = state->draft.draft_revision;
    outputs->submitted_crc = state->draft.draft_crc;

    TEACH_LOG_INFO("Validation requested: program=%lu revision=%lu CRC=0x%08lX segments=%u\r\n",
                   (unsigned long)state->draft.program_id,
                   (unsigned long)state->draft.draft_revision,
                   (unsigned long)state->draft.draft_crc,
                   (unsigned)state->draft.segment_count);
    return true;
}

bool TeachingState_Init(TeachingState *state,
                        const TeachingConfig *config,
                        uint32_t program_id)
{
    if (state == NULL || config == NULL || !config_is_valid(config))
    {
        return false;
    }

    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->draft.program_id = program_id;
    state->draft.global_speed_scale = 1.0F;
    state->teaching_speed_mps = config->default_speed_mps;
    state->selected_orientation_mode = TEACH_ORIENTATION_CONSTANT;
    state->selected_circle_direction = TEACH_CIRCLE_CCW;
    state->phase = TEACH_PHASE_WAIT_SEGMENT_SELECTION;
    state->draft.status = TEACH_DRAFT_EMPTY;
    state->status_message_id = TEACH_MSG_SELECT_SEGMENT;

    TEACH_LOG_INFO("Initialized program=%lu default_speed=%.4f m/s capacity=%u\r\n",
                   (unsigned long)program_id,
                   (double)config->default_speed_mps,
                   (unsigned)TEACHING_MAX_SEGMENTS);
    return true;
}

void TeachingState_ResetDraft(TeachingState *state)
{
    TeachingConfig config;
    uint32_t program_id;
    uint32_t next_revision;
    float speed;

    if (state == NULL)
    {
        return;
    }

    config = state->config;
    program_id = state->draft.program_id;
    next_revision = state->draft.draft_revision + 1U;
    speed = state->teaching_speed_mps;

    memset(state, 0, sizeof(*state));
    state->config = config;
    state->draft.program_id = program_id;
    state->draft.draft_revision = next_revision;
    state->draft.global_speed_scale = 1.0F;
    state->draft.status = TEACH_DRAFT_EMPTY;
    state->teaching_speed_mps = speed;
    state->selected_orientation_mode = TEACH_ORIENTATION_CONSTANT;
    state->selected_circle_direction = TEACH_CIRCLE_CCW;
    state->phase = TEACH_PHASE_WAIT_SEGMENT_SELECTION;
    state->status_message_id = TEACH_MSG_DRAFT_RESET;

    TEACH_LOG_WARN("Draft erased immediately; revision=%lu. No homing or fault clear requested.\r\n",
                   (unsigned long)next_revision);
}

bool TeachingState_ProcessEvent(TeachingState *state,
                                const TeachingInputs *inputs,
                                TeachingEvent event,
                                TeachingOutputs *outputs)
{
    bool accepted = false;

    if (state == NULL || inputs == NULL || outputs == NULL)
    {
        return false;
    }

    clear_outputs(outputs);

    /* Reset is non-motion and deliberately has immediate priority. */
    if (event == TEACH_EVENT_RESET)
    {
        TeachingState_ResetDraft(state);
        outputs->draft_was_reset = true;
        accepted = true;
        TeachingState_GetOutputs(state, outputs);
        outputs->draft_was_reset = true;
        return accepted;
    }

    if (state->draft.status == TEACH_DRAFT_SUBMITTED)
    {
        set_error(state, TEACH_ERR_DRAFT_LOCKED);
        TeachingState_GetOutputs(state, outputs);
        return false;
    }

    switch (event)
    {
        case TEACH_EVENT_NONE:
            accepted = true;
            break;

        case TEACH_EVENT_SELECT_LINE:
        case TEACH_EVENT_SELECT_ARC:
        case TEACH_EVENT_SELECT_CIRCLE:
            if (state->phase == TEACH_PHASE_WAIT_POINT &&
                state->captured_point_count > 0U)
            {
                set_error(state, TEACH_ERR_SEGMENT_INCOMPLETE);
                break;
            }
            begin_segment(state,
                          (event == TEACH_EVENT_SELECT_LINE) ? TEACH_SEGMENT_LINE :
                          (event == TEACH_EVENT_SELECT_ARC) ? TEACH_SEGMENT_ARC :
                                                             TEACH_SEGMENT_CIRCLE);
            accepted = true;
            break;

        case TEACH_EVENT_RECORD_POINT:
            accepted = record_point(state, inputs);
            break;

        case TEACH_EVENT_VALIDATE_PATH:
            accepted = submit_for_validation(state, outputs);
            break;

        case TEACH_EVENT_SPEED_INCREASE:
            accepted = change_speed(state, state->teaching_speed_mps +
                                           state->config.speed_step_mps);
            break;

        case TEACH_EVENT_SPEED_DECREASE:
            accepted = change_speed(state, state->teaching_speed_mps -
                                           state->config.speed_step_mps);
            break;

        case TEACH_EVENT_SPEED_DEFAULT:
            accepted = change_speed(state, state->config.default_speed_mps);
            break;

        default:
            set_error(state, TEACH_ERR_NULL_ARGUMENT);
            break;
    }

    /* Preserve pulse outputs set by submit_for_validation(). */
    {
        const bool validation_request = outputs->validation_request;
        const uint32_t program_id = outputs->submitted_program_id;
        const uint32_t revision = outputs->submitted_revision;
        const uint32_t crc = outputs->submitted_crc;
        TeachingState_GetOutputs(state, outputs);
        outputs->validation_request = validation_request;
        outputs->submitted_program_id = program_id;
        outputs->submitted_revision = revision;
        outputs->submitted_crc = crc;
    }
    return accepted;
}

void TeachingState_GetOutputs(const TeachingState *state,
                              TeachingOutputs *outputs)
{
    if (state == NULL || outputs == NULL)
    {
        return;
    }

    clear_outputs(outputs);
    outputs->status_message_id = state->status_message_id;
    outputs->error = state->error;
    outputs->phase = state->phase;
    outputs->current_segment_number = (uint16_t)(state->draft.segment_count + 1U);
    outputs->next_point_number = (state->phase == TEACH_PHASE_WAIT_POINT) ?
                                 (uint8_t)(state->captured_point_count + 1U) : 0U;
    outputs->displayed_speed_mps = state->teaching_speed_mps;
    outputs->record_allowed = (state->phase == TEACH_PHASE_WAIT_POINT);
    outputs->validate_allowed = (state->draft.segment_count > 0U) &&
                                !(state->phase == TEACH_PHASE_WAIT_POINT &&
                                  state->captured_point_count > 0U) &&
                                (state->draft.status != TEACH_DRAFT_SUBMITTED);
}

bool TeachingState_SetCircleDirection(TeachingState *state,
                                      TeachingCircleDirection direction)
{
    if (state == NULL ||
        (direction != TEACH_CIRCLE_CCW && direction != TEACH_CIRCLE_CW))
    {
        return false;
    }
    state->selected_circle_direction = direction;
    return true;
}

bool TeachingState_SetOrientationMode(TeachingState *state,
                                      TeachingOrientationMode mode)
{
    if (state == NULL ||
        (mode != TEACH_ORIENTATION_CONSTANT &&
         mode != TEACH_ORIENTATION_INTERPOLATED))
    {
        return false;
    }
    state->selected_orientation_mode = mode;
    return true;
}

const char *TeachingState_PhaseName(TeachingPhase phase)
{
    switch (phase)
    {
        case TEACH_PHASE_WAIT_SEGMENT_SELECTION: return "WAIT_SEGMENT_SELECTION";
        case TEACH_PHASE_WAIT_POINT:             return "WAIT_POINT";
        case TEACH_PHASE_SEGMENT_COMPLETE:       return "SEGMENT_COMPLETE";
        case TEACH_PHASE_SUBMITTING_DRAFT:       return "SUBMITTING_DRAFT";
        default:                                 return "UNKNOWN_PHASE";
    }
}

const char *TeachingState_ErrorName(TeachingError error)
{
    switch (error)
    {
        case TEACH_ERR_NONE:                    return "NONE";
        case TEACH_ERR_NULL_ARGUMENT:           return "NULL_ARGUMENT";
        case TEACH_ERR_NO_SEGMENT_SELECTED:     return "NO_SEGMENT_SELECTED";
        case TEACH_ERR_ROBOT_NOT_HOMED:         return "ROBOT_NOT_HOMED";
        case TEACH_ERR_DRIVES_NOT_READY:        return "DRIVES_NOT_READY";
        case TEACH_ERR_ROBOT_NOT_SETTLED:       return "ROBOT_NOT_SETTLED";
        case TEACH_ERR_GUIDANCE_INACTIVE:       return "GUIDANCE_INACTIVE";
        case TEACH_ERR_MOTION_NOT_PERMITTED:    return "MOTION_NOT_PERMITTED";
        case TEACH_ERR_SAFETY_ACTIVE:           return "SAFETY_ACTIVE";
        case TEACH_ERR_GLOBAL_FAULT_ACTIVE:     return "GLOBAL_FAULT_ACTIVE";
        case TEACH_ERR_DUPLICATE_POINT:         return "DUPLICATE_POINT";
        case TEACH_ERR_COLLINEAR_POINTS:        return "COLLINEAR_POINTS";
        case TEACH_ERR_SEGMENT_INCOMPLETE:      return "SEGMENT_INCOMPLETE";
        case TEACH_ERR_DRAFT_EMPTY:             return "DRAFT_EMPTY";
        case TEACH_ERR_SEGMENT_CAPACITY:        return "SEGMENT_CAPACITY";
        case TEACH_ERR_INVALID_SPEED:           return "INVALID_SPEED";
        case TEACH_ERR_INVALID_MEASUREMENT:     return "INVALID_MEASUREMENT";
        case TEACH_ERR_FRAME_CHANGED:           return "FRAME_CHANGED";
        case TEACH_ERR_TOOL_CHANGED:            return "TOOL_CHANGED";
        case TEACH_ERR_CALIBRATION_CHANGED:     return "CALIBRATION_CHANGED";
        case TEACH_ERR_DRAFT_LOCKED:            return "DRAFT_LOCKED";
        default:                                return "UNKNOWN_ERROR";
    }
}

const char *TeachingState_SegmentName(TeachingSegmentType type)
{
    switch (type)
    {
        case TEACH_SEGMENT_LINE:   return "LINE";
        case TEACH_SEGMENT_ARC:    return "ARC";
        case TEACH_SEGMENT_CIRCLE: return "CIRCLE";
        default:                   return "NONE";
    }
}
