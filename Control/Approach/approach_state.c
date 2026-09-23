#include "approach_state.h"
#include "approach_state_debug.h"

#include <math.h>
#include <string.h>

#define QUINTIC_PEAK_VELOCITY     1.875f
#define QUINTIC_PEAK_ACCELERATION 5.773502692f
#define QUINTIC_PEAK_JERK         60.0f

static bool finite_vector(const float q[APPROACH_DOF])
{
    uint32_t j;
    for (j = 0U; j < APPROACH_DOF; ++j) {
        if (!isfinite(q[j])) {
            return false;
        }
    }
    return true;
}

static void copy_vector(float dst[APPROACH_DOF],
                        const float src[APPROACH_DOF])
{
    memcpy(dst, src, sizeof(float) * APPROACH_DOF);
}

static ApproachResult fail_state(ApproachState *state,
                                 ApproachError error,
                                 uint8_t failed_joint)
{
    state->error = error;
    state->failed_joint = failed_joint;
    state->phase = APPROACH_PHASE_FAILED;
    state->result = APPROACH_RESULT_FAILED;
    APPROACH_LOG("FAILED: %s, joint=%u",
                 ApproachState_ErrorString(error), (unsigned)failed_joint);
    return state->result;
}

static ApproachResult abort_state(ApproachState *state, ApproachError reason)
{
    state->error = reason;
    state->phase = APPROACH_PHASE_ABORTED;
    state->result = APPROACH_RESULT_ABORTED;
    APPROACH_LOG("ABORTED: %s", ApproachState_ErrorString(reason));
    return state->result;
}

static bool config_valid(const ApproachConfig *c)
{
    uint32_t j;
    if (c == NULL || c->duration_safety_factor < 1.0f ||
        c->minimum_leg_duration_s <= 0.0f ||
        c->maximum_leg_duration_s < c->minimum_leg_duration_s ||
        c->final_position_tolerance_rad <= 0.0f ||
        c->following_error_limit_rad <= 0.0f ||
        c->required_stable_cycles == 0U ||
        c->maximum_verification_cycles == 0U ||
        c->validation_samples_per_step == 0U) {
        return false;
    }
    for (j = 0U; j < APPROACH_DOF; ++j) {
        if (!isfinite(c->joint_min_rad[j]) ||
            !isfinite(c->joint_max_rad[j]) ||
            c->joint_min_rad[j] >= c->joint_max_rad[j] ||
            c->velocity_max_rad_s[j] <= 0.0f ||
            (c->use_acceleration_limits && c->acceleration_max_rad_s2[j] <= 0.0f) ||
            (c->use_jerk_limits && c->jerk_max_rad_s3[j] <= 0.0f)) {
            return false;
        }
    }
    return true;
}

static bool services_valid(const ApproachServices *s, bool collision_required)
{
    return s != NULL && s->read_first_sample != NULL &&
           s->units_to_rad != NULL && s->rad_to_units != NULL &&
           s->read_feedback != NULL && s->write_targets != NULL &&
           s->exchange_bus != NULL &&
           (!collision_required || s->collision_free != NULL);
}

static bool vector_in_limits(const ApproachState *state,
                             const float q[APPROACH_DOF],
                             uint8_t *failed_joint)
{
    uint32_t j;
    for (j = 0U; j < APPROACH_DOF; ++j) {
        if (!isfinite(q[j]) || q[j] < state->config.joint_min_rad[j] ||
            q[j] > state->config.joint_max_rad[j]) {
            if (failed_joint != NULL) {
                *failed_joint = (uint8_t)(j + 1U);
            }
            return false;
        }
    }
    return true;
}

static float calculate_leg_duration(const ApproachState *state,
                                    const float start[APPROACH_DOF],
                                    const float goal[APPROACH_DOF])
{
    float duration = state->config.minimum_leg_duration_s;
    uint32_t j;
    for (j = 0U; j < APPROACH_DOF; ++j) {
        const float dq = fabsf(goal[j] - start[j]);
        float required = QUINTIC_PEAK_VELOCITY * dq /
                         state->config.velocity_max_rad_s[j];
        if (required > duration) {
            duration = required;
        }
        if (state->config.use_acceleration_limits) {
            required = sqrtf(QUINTIC_PEAK_ACCELERATION * dq /
                             state->config.acceleration_max_rad_s2[j]);
            if (required > duration) {
                duration = required;
            }
        }
        if (state->config.use_jerk_limits) {
            required = cbrtf(QUINTIC_PEAK_JERK * dq /
                             state->config.jerk_max_rad_s3[j]);
            if (required > duration) {
                duration = required;
            }
        }
    }
    return duration * state->config.duration_safety_factor;
}

static void evaluate_leg(const ApproachState *state,
                         uint8_t leg,
                         uint32_t sample_index,
                         float q[APPROACH_DOF])
{
    const uint32_t intervals = state->route_intervals[leg];
    const float tau = (intervals == 0U) ? 1.0f :
                      (float)sample_index / (float)intervals;
    const float tau2 = tau * tau;
    const float tau3 = tau2 * tau;
    const float tau4 = tau3 * tau;
    const float tau5 = tau4 * tau;
    const float scale = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;
    uint32_t j;

    for (j = 0U; j < APPROACH_DOF; ++j) {
        q[j] = state->route_pose_rad[leg][j] +
               (state->route_pose_rad[leg + 1U][j] -
                state->route_pose_rad[leg][j]) * scale;
    }
    if (sample_index >= intervals) {
        copy_vector(q, state->route_pose_rad[leg + 1U]);
    }
}

static bool read_feedback_ready(ApproachState *state,
                                float actual[APPROACH_DOF])
{
    bool ready[APPROACH_DOF];
    uint32_t j;
    if (!state->services.read_feedback(actual, ready,
                                       state->services.context) ||
        !finite_vector(actual)) {
        state->error = APPROACH_ERR_FEEDBACK;
        return false;
    }
    for (j = 0U; j < APPROACH_DOF; ++j) {
        if (!ready[j]) {
            state->error = APPROACH_ERR_DRIVE_NOT_READY;
            state->failed_joint = (uint8_t)(j + 1U);
            return false;
        }
    }
    return true;
}

static bool send_joint_target(ApproachState *state,
                              const float target_rad[APPROACH_DOF],
                              bool use_exact_final_units)
{
    int32_t units[APPROACH_DOF];
    if (use_exact_final_units) {
        memcpy(units, state->final_target_units, sizeof(units));
    } else if (!state->services.rad_to_units(target_rad, units,
                                              state->services.context)) {
        state->error = APPROACH_ERR_POSITION_CONVERSION;
        return false;
    }
    if (!state->services.write_targets(units, state->services.context)) {
        state->error = APPROACH_ERR_BUS;
        return false;
    }
    if (!state->services.exchange_bus(state->services.context)) {
        state->error = APPROACH_ERR_BUS;
        return false;
    }
    copy_vector(state->last_command_rad, target_rad);
    state->last_command_valid = true;
    return true;
}

static bool hold_actual(ApproachState *state)
{
    float actual[APPROACH_DOF];
    if (!read_feedback_ready(state, actual)) {
        return false;
    }
    return send_joint_target(state, actual, false);
}

static bool following_error_valid(ApproachState *state,
                                  const float actual[APPROACH_DOF])
{
    uint32_t j;
    if (!state->last_command_valid) {
        return true;
    }
    for (j = 0U; j < APPROACH_DOF; ++j) {
        if (fabsf(actual[j] - state->last_command_rad[j]) >
            state->config.following_error_limit_rad) {
            state->failed_joint = (uint8_t)(j + 1U);
            return false;
        }
    }
    return true;
}

static void build_route_from_request(ApproachState *state)
{
    uint8_t i;
    copy_vector(state->route_pose_rad[0], state->q_start_rad);
    for (i = 0U; i < state->request.clearance_pose_count; ++i) {
        copy_vector(state->route_pose_rad[i + 1U],
                    state->request.clearance_pose_rad[i]);
    }
    copy_vector(state->route_pose_rad[state->request.clearance_pose_count + 1U],
                state->final_target_rad);
    state->total_legs = (uint8_t)(state->request.clearance_pose_count + 1U);
}

static void replan_from_actual(ApproachState *state,
                               const float actual[APPROACH_DOF])
{
    float remaining_goals[APPROACH_MAX_LEGS][APPROACH_DOF];
    const uint8_t remaining = (uint8_t)(state->total_legs - state->active_leg);
    uint8_t i;
    for (i = 0U; i < remaining; ++i) {
        copy_vector(remaining_goals[i],
                    state->route_pose_rad[state->active_leg + i + 1U]);
    }
    copy_vector(state->route_pose_rad[0], actual);
    for (i = 0U; i < remaining; ++i) {
        copy_vector(state->route_pose_rad[i + 1U], remaining_goals[i]);
    }
    state->total_legs = remaining;
    state->active_leg = 0U;
    state->sample_index = 0U;
    state->validation_index = 0U;
    state->last_command_valid = false;
    ++state->replan_count;
}

void ApproachState_Init(ApproachState *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
        state->phase = APPROACH_PHASE_IDLE;
    }
}

bool ApproachState_Enter(ApproachState *state,
                         const ApproachRequest *request,
                         const ApproachConfig *config,
                         const ApproachServices *services)
{
    if (state == NULL || request == NULL || config == NULL || services == NULL) {
        return false;
    }
    ApproachState_Init(state);
    state->request = *request;
    state->config = *config;
    state->services = *services;
    state->phase = APPROACH_PHASE_CHECK_REQUEST;
    state->result = APPROACH_RESULT_RUNNING;
    APPROACH_LOG("entered, operation=%u", (unsigned)request->operation);
    return true;
}

ApproachResult ApproachState_Step(ApproachState *state,
                                  const ApproachControlInputs *inputs)
{
    uint32_t j;
    if (state == NULL || inputs == NULL) {
        return APPROACH_RESULT_FAILED;
    }
    if (state->result == APPROACH_RESULT_COMPLETE ||
        state->result == APPROACH_RESULT_ABORTED ||
        state->result == APPROACH_RESULT_FAILED) {
        return state->result;
    }

    if (inputs->estop_active) {
        return fail_state(state, APPROACH_ERR_ESTOP, 0U);
    }
    if (inputs->external_fault_active) {
        return fail_state(state, APPROACH_ERR_EXTERNAL_FAULT, 0U);
    }
    if (inputs->reset_requested) {
        return abort_state(state, APPROACH_ERR_RESET_REQUESTED);
    }
    if (inputs->home_requested) {
        return abort_state(state, APPROACH_ERR_HOME_REQUESTED);
    }

    switch (state->phase) {
    case APPROACH_PHASE_CHECK_REQUEST:
        if (!config_valid(&state->config)) {
            return fail_state(state, APPROACH_ERR_INVALID_CONFIG, 0U);
        }
        if (!services_valid(&state->services,
                            state->config.require_collision_check)) {
            return fail_state(state,
                state->config.require_collision_check &&
                state->services.collision_free == NULL ?
                APPROACH_ERR_COLLISION_CHECK_MISSING :
                APPROACH_ERR_INVALID_CONFIG, 0U);
        }
        if ((state->request.operation != APPROACH_OPERATION_PREVIEW &&
             state->request.operation != APPROACH_OPERATION_WELD) ||
            state->request.clearance_pose_count > APPROACH_MAX_CLEARANCE_POSES ||
            (state->request.clearance_pose_count > 0U &&
             state->request.clearance_pose_rad == NULL)) {
            return fail_state(state, APPROACH_ERR_INVALID_REQUEST, 0U);
        }
        state->phase = APPROACH_PHASE_LOAD_TARGET;
        return state->result;

    case APPROACH_PHASE_LOAD_TARGET: {
        ApproachExecutionSample first;
        const ApproachTrajectoryInfo *t = state->request.trajectory;
        if (t == NULL || !t->valid) {
            return fail_state(state, APPROACH_ERR_TRAJECTORY_NOT_VALID, 0U);
        }
        if (t->program_id != state->request.expected_program_id ||
            t->source_revision != state->request.expected_source_revision ||
            t->artifact_crc != state->request.expected_artifact_crc ||
            t->sample_period_us != APPROACH_SAMPLE_PERIOD_US) {
            return fail_state(state, APPROACH_ERR_TRAJECTORY_MISMATCH, 0U);
        }
        if (t->sample_count == 0U) {
            return fail_state(state, APPROACH_ERR_EMPTY_TRAJECTORY, 0U);
        }
        if (!state->services.read_first_sample(&first,
                                               state->services.context)) {
            return fail_state(state, APPROACH_ERR_STORAGE_READ, 0U);
        }
        memcpy(state->final_target_units, first.target_position_units,
               sizeof(state->final_target_units));
        if (!state->services.units_to_rad(first.target_position_units,
                                          state->final_target_rad,
                                          state->services.context) ||
            !finite_vector(state->final_target_rad)) {
            return fail_state(state, APPROACH_ERR_POSITION_CONVERSION, 0U);
        }
        if (!vector_in_limits(state, state->final_target_rad,
                              &state->failed_joint)) {
            return fail_state(state, APPROACH_ERR_TARGET_LIMIT,
                              state->failed_joint);
        }
        state->phase = APPROACH_PHASE_READ_START;
        return state->result;
    }

    case APPROACH_PHASE_READ_START: {
        float actual[APPROACH_DOF];
        if (!read_feedback_ready(state, actual)) {
            return fail_state(state, state->error, state->failed_joint);
        }
        copy_vector(state->q_start_rad, actual);
        build_route_from_request(state);
        state->active_leg = 0U;
        state->phase = APPROACH_PHASE_PREPARE_LEG;
        return state->result;
    }

    case APPROACH_PHASE_PREPARE_LEG:
        if (state->active_leg >= state->total_legs) {
            state->active_leg = 0U;
            state->validation_index = 0U;
            state->phase = APPROACH_PHASE_VALIDATE_LEG;
            APPROACH_LOG("route planned: %u leg(s)",
                         (unsigned)state->total_legs);
            return state->result;
        }
        if (!vector_in_limits(state, state->route_pose_rad[state->active_leg],
                              &state->failed_joint) ||
            !vector_in_limits(state, state->route_pose_rad[state->active_leg + 1U],
                              &state->failed_joint)) {
            return fail_state(state, APPROACH_ERR_TARGET_LIMIT,
                              state->failed_joint);
        }
        state->route_duration_s[state->active_leg] =
            calculate_leg_duration(state,
                state->route_pose_rad[state->active_leg],
                state->route_pose_rad[state->active_leg + 1U]);
        if (!isfinite(state->route_duration_s[state->active_leg]) ||
            state->route_duration_s[state->active_leg] >
            state->config.maximum_leg_duration_s) {
            return fail_state(state, APPROACH_ERR_PLAN_LIMIT, 0U);
        }
        state->route_intervals[state->active_leg] = (uint32_t)ceilf(
            state->route_duration_s[state->active_leg] /
            APPROACH_SAMPLE_PERIOD_S);
        if (state->route_intervals[state->active_leg] == 0U) {
            state->route_intervals[state->active_leg] = 1U;
        }
        state->route_duration_s[state->active_leg] =
            state->route_intervals[state->active_leg] *
            APPROACH_SAMPLE_PERIOD_S;
        ++state->active_leg;
        return state->result;

    case APPROACH_PHASE_VALIDATE_LEG: {
        uint16_t work;
        float q[APPROACH_DOF];
        if (state->active_leg >= state->total_legs) {
            state->active_leg = 0U;
            state->sample_index = 0U;
            state->phase = APPROACH_PHASE_WAIT_PERMISSION;
            APPROACH_LOG("complete route validated before motion");
            return state->result;
        }
        for (work = 0U; work < state->config.validation_samples_per_step; ++work) {
            evaluate_leg(state, state->active_leg,
                         state->validation_index, q);
            if (!vector_in_limits(state, q, &state->failed_joint)) {
                return fail_state(state, APPROACH_ERR_TARGET_LIMIT,
                                  state->failed_joint);
            }
            if (state->services.collision_free != NULL &&
                !state->services.collision_free(q,
                                                state->services.context)) {
                return fail_state(state, APPROACH_ERR_COLLISION, 0U);
            }
            if (state->validation_index >=
                state->route_intervals[state->active_leg]) {
                ++state->active_leg;
                state->validation_index = 0U;
                break;
            }
            ++state->validation_index;
        }
        return state->result;
    }

    case APPROACH_PHASE_WAIT_PERMISSION:
        if (inputs->pause_requested || inputs->protective_stop_active ||
            !inputs->motion_permission) {
            if (!hold_actual(state)) {
                return fail_state(state, state->error, state->failed_joint);
            }
            return state->result;
        }
        state->phase = APPROACH_PHASE_EXECUTE_LEG;
        APPROACH_LOG("execution started");
        return state->result;

    case APPROACH_PHASE_EXECUTE_LEG: {
        float actual[APPROACH_DOF];
        float command[APPROACH_DOF];
        bool exact_final;
        if (inputs->pause_requested || inputs->protective_stop_active ||
            !inputs->motion_permission) {
            if (!hold_actual(state)) {
                return fail_state(state, state->error, state->failed_joint);
            }
            state->phase = APPROACH_PHASE_PAUSED;
            state->paused_during_verification = false;
            APPROACH_LOG("paused; replan required before resume");
            return state->result;
        }
        if (!read_feedback_ready(state, actual)) {
            return fail_state(state, state->error, state->failed_joint);
        }
        if (!following_error_valid(state, actual)) {
            return fail_state(state, APPROACH_ERR_FOLLOWING_ERROR,
                              state->failed_joint);
        }
        evaluate_leg(state, state->active_leg, state->sample_index, command);
        exact_final = (state->active_leg + 1U == state->total_legs) &&
                      (state->sample_index >=
                       state->route_intervals[state->active_leg]);
        if (!send_joint_target(state, command, exact_final)) {
            return fail_state(state, state->error, state->failed_joint);
        }
        ++state->samples_sent;
        if (state->sample_index >= state->route_intervals[state->active_leg]) {
            ++state->active_leg;
            state->sample_index = 0U;
            state->last_command_valid = false;
            if (state->active_leg >= state->total_legs) {
                state->phase = APPROACH_PHASE_VERIFY_TARGET;
                APPROACH_LOG("final command sent; verifying feedback");
            }
        } else {
            ++state->sample_index;
        }
        return state->result;
    }

    case APPROACH_PHASE_PAUSED: {
        float actual[APPROACH_DOF];
        if (!hold_actual(state)) {
            return fail_state(state, state->error, state->failed_joint);
        }
        if (inputs->pause_requested || inputs->protective_stop_active ||
            !inputs->motion_permission) {
            return state->result;
        }
        if (state->paused_during_verification) {
            state->paused_during_verification = false;
            state->phase = APPROACH_PHASE_VERIFY_TARGET;
            APPROACH_LOG("verification resumed");
            return state->result;
        }
        if (!read_feedback_ready(state, actual)) {
            return fail_state(state, state->error, state->failed_joint);
        }
        replan_from_actual(state, actual);
        state->phase = APPROACH_PHASE_PREPARE_LEG;
        APPROACH_LOG("resume accepted; replanning from actual position");
        return state->result;
    }

    case APPROACH_PHASE_VERIFY_TARGET: {
        float actual[APPROACH_DOF];
        bool within = true;
        if (inputs->pause_requested || inputs->protective_stop_active ||
            !inputs->motion_permission) {
            if (!hold_actual(state)) {
                return fail_state(state, state->error, state->failed_joint);
            }
            state->phase = APPROACH_PHASE_PAUSED;
            state->paused_during_verification = true;
            return state->result;
        }
        if (!read_feedback_ready(state, actual)) {
            return fail_state(state, state->error, state->failed_joint);
        }
        for (j = 0U; j < APPROACH_DOF; ++j) {
            if (fabsf(actual[j] - state->final_target_rad[j]) >
                state->config.final_position_tolerance_rad) {
                within = false;
            }
        }
        if (!send_joint_target(state, state->final_target_rad, true)) {
            return fail_state(state, state->error, state->failed_joint);
        }
        ++state->verification_cycles;
        state->stable_cycles = within ? state->stable_cycles + 1U : 0U;
        if (state->stable_cycles >= state->config.required_stable_cycles) {
            state->phase = APPROACH_PHASE_COMPLETE;
            state->result = APPROACH_RESULT_COMPLETE;
            APPROACH_LOG("complete after %lu samples",
                         (unsigned long)state->samples_sent);
        } else if (state->verification_cycles >=
                   state->config.maximum_verification_cycles) {
            return fail_state(state, APPROACH_ERR_VERIFY_TIMEOUT, 0U);
        }
        return state->result;
    }

    case APPROACH_PHASE_COMPLETE:
        state->result = APPROACH_RESULT_COMPLETE;
        return state->result;
    case APPROACH_PHASE_ABORTED:
        state->result = APPROACH_RESULT_ABORTED;
        return state->result;
    case APPROACH_PHASE_FAILED:
        state->result = APPROACH_RESULT_FAILED;
        return state->result;
    case APPROACH_PHASE_IDLE:
    default:
        return fail_state(state, APPROACH_ERR_INVALID_REQUEST, 0U);
    }
}

void ApproachState_GetReport(const ApproachState *state,
                             ApproachReport *report)
{
    uint32_t total_intervals = 0U;
    uint32_t completed = 0U;
    uint8_t i;
    if (state == NULL || report == NULL) {
        return;
    }
    memset(report, 0, sizeof(*report));
    report->phase = state->phase;
    report->result = state->result;
    report->error = state->error;
    report->failed_joint = state->failed_joint;
    report->active_leg = state->active_leg;
    report->total_legs = state->total_legs;
    report->sample_index = state->sample_index;
    report->leg_intervals = state->active_leg < state->total_legs ?
                            state->route_intervals[state->active_leg] : 0U;
    report->samples_sent = state->samples_sent;
    report->stable_cycles = state->stable_cycles;
    report->verification_cycles = state->verification_cycles;
    report->replan_count = state->replan_count;
    report->leg_duration_s = state->active_leg < state->total_legs ?
                             state->route_duration_s[state->active_leg] : 0.0f;
    copy_vector(report->q_start_rad, state->q_start_rad);
    if (state->active_leg < state->total_legs) {
        copy_vector(report->q_goal_rad,
                    state->route_pose_rad[state->active_leg + 1U]);
    } else {
        copy_vector(report->q_goal_rad, state->final_target_rad);
    }
    copy_vector(report->final_target_rad, state->final_target_rad);
    memcpy(report->final_target_units, state->final_target_units,
           sizeof(report->final_target_units));
    for (i = 0U; i < state->total_legs; ++i) {
        total_intervals += state->route_intervals[i];
        if (i < state->active_leg) {
            completed += state->route_intervals[i];
        }
    }
    completed += state->sample_index;
    report->progress_0_to_1 = total_intervals == 0U ? 0.0f :
        (float)completed / (float)total_intervals;
    if (report->progress_0_to_1 > 1.0f ||
        state->result == APPROACH_RESULT_COMPLETE) {
        report->progress_0_to_1 = 1.0f;
    }
}

const char *ApproachState_PhaseString(ApproachPhase phase)
{
    switch (phase) {
    case APPROACH_PHASE_IDLE: return "IDLE";
    case APPROACH_PHASE_CHECK_REQUEST: return "CHECK_REQUEST";
    case APPROACH_PHASE_LOAD_TARGET: return "LOAD_TARGET";
    case APPROACH_PHASE_READ_START: return "READ_START";
    case APPROACH_PHASE_PREPARE_LEG: return "PREPARE_ROUTE";
    case APPROACH_PHASE_VALIDATE_LEG: return "VALIDATE_ROUTE";
    case APPROACH_PHASE_WAIT_PERMISSION: return "WAIT_PERMISSION";
    case APPROACH_PHASE_EXECUTE_LEG: return "EXECUTE";
    case APPROACH_PHASE_PAUSED: return "PAUSED";
    case APPROACH_PHASE_VERIFY_TARGET: return "VERIFY_TARGET";
    case APPROACH_PHASE_COMPLETE: return "COMPLETE";
    case APPROACH_PHASE_ABORTED: return "ABORTED";
    case APPROACH_PHASE_FAILED: return "FAILED";
    default: return "UNKNOWN";
    }
}

const char *ApproachState_ErrorString(ApproachError error)
{
    switch (error) {
    case APPROACH_ERR_NONE: return "none";
    case APPROACH_ERR_NULL_ARGUMENT: return "null argument";
    case APPROACH_ERR_INVALID_CONFIG: return "invalid configuration";
    case APPROACH_ERR_INVALID_REQUEST: return "invalid request";
    case APPROACH_ERR_TRAJECTORY_NOT_VALID: return "trajectory not valid";
    case APPROACH_ERR_TRAJECTORY_MISMATCH: return "trajectory identity mismatch";
    case APPROACH_ERR_EMPTY_TRAJECTORY: return "empty trajectory";
    case APPROACH_ERR_STORAGE_READ: return "trajectory storage read failed";
    case APPROACH_ERR_POSITION_CONVERSION: return "position conversion failed";
    case APPROACH_ERR_TARGET_LIMIT: return "joint target outside limits";
    case APPROACH_ERR_FEEDBACK: return "joint feedback unavailable";
    case APPROACH_ERR_DRIVE_NOT_READY: return "drive not ready";
    case APPROACH_ERR_PLAN_LIMIT: return "motion plan violates limits";
    case APPROACH_ERR_COLLISION: return "collision detected";
    case APPROACH_ERR_COLLISION_CHECK_MISSING: return "collision check missing";
    case APPROACH_ERR_MOTION_TIMEOUT: return "motion timeout";
    case APPROACH_ERR_VERIFY_TIMEOUT: return "target verification timeout";
    case APPROACH_ERR_FOLLOWING_ERROR: return "following error exceeded";
    case APPROACH_ERR_BUS: return "fieldbus exchange failed";
    case APPROACH_ERR_EXTERNAL_FAULT: return "external fault";
    case APPROACH_ERR_ESTOP: return "emergency stop active";
    case APPROACH_ERR_RESET_REQUESTED: return "reset requested";
    case APPROACH_ERR_HOME_REQUESTED: return "home requested";
    default: return "unknown";
    }
}
