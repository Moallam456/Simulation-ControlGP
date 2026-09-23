#include "approach_state.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    float actual[APPROACH_DOF];
    int32_t first_units[APPROACH_DOF];
    uint32_t exchanges;
    bool collision_allowed;
} MockHardware;

static bool mock_read_first(ApproachExecutionSample *sample, void *context)
{
    MockHardware *mock = (MockHardware *)context;
    memcpy(sample->target_position_units, mock->first_units,
           sizeof(mock->first_units));
    return true;
}

static bool mock_units_to_rad(const int32_t units[APPROACH_DOF],
                              float radians[APPROACH_DOF], void *context)
{
    uint32_t j;
    (void)context;
    for (j = 0U; j < APPROACH_DOF; ++j) {
        radians[j] = (float)units[j] / 100000.0f;
    }
    return true;
}

static bool mock_rad_to_units(const float radians[APPROACH_DOF],
                              int32_t units[APPROACH_DOF], void *context)
{
    uint32_t j;
    (void)context;
    for (j = 0U; j < APPROACH_DOF; ++j) {
        units[j] = (int32_t)lroundf(radians[j] * 100000.0f);
    }
    return true;
}

static bool mock_read_feedback(float actual[APPROACH_DOF],
                               bool ready[APPROACH_DOF], void *context)
{
    MockHardware *mock = (MockHardware *)context;
    uint32_t j;
    memcpy(actual, mock->actual, sizeof(mock->actual));
    for (j = 0U; j < APPROACH_DOF; ++j) {
        ready[j] = true;
    }
    return true;
}

static bool mock_write(const int32_t units[APPROACH_DOF], void *context)
{
    MockHardware *mock = (MockHardware *)context;
    uint32_t j;
    for (j = 0U; j < APPROACH_DOF; ++j) {
        mock->actual[j] = (float)units[j] / 100000.0f;
    }
    return true;
}

static bool mock_exchange(void *context)
{
    MockHardware *mock = (MockHardware *)context;
    ++mock->exchanges;
    return true;
}

static bool mock_collision(const float q[APPROACH_DOF], void *context)
{
    MockHardware *mock = (MockHardware *)context;
    (void)q;
    return mock->collision_allowed;
}

static ApproachConfig make_config(void)
{
    ApproachConfig c;
    uint32_t j;
    memset(&c, 0, sizeof(c));
    for (j = 0U; j < APPROACH_DOF; ++j) {
        c.joint_min_rad[j] = -3.2f;
        c.joint_max_rad[j] = 3.2f;
        c.velocity_max_rad_s[j] = 1.5f;
        c.acceleration_max_rad_s2[j] = 3.0f;
        c.jerk_max_rad_s3[j] = 20.0f;
    }
    c.use_acceleration_limits = true;
    c.use_jerk_limits = true;
    c.duration_safety_factor = 1.10f;
    c.minimum_leg_duration_s = 0.050f;
    c.maximum_leg_duration_s = 10.0f;
    c.final_position_tolerance_rad = 0.001f;
    c.following_error_limit_rad = 0.10f;
    c.required_stable_cycles = 5U;
    c.maximum_verification_cycles = 100U;
    c.validation_samples_per_step = 64U;
    c.require_collision_check = true;
    return c;
}

static ApproachServices make_services(MockHardware *mock)
{
    ApproachServices s = {
        .read_first_sample = mock_read_first,
        .units_to_rad = mock_units_to_rad,
        .rad_to_units = mock_rad_to_units,
        .read_feedback = mock_read_feedback,
        .write_targets = mock_write,
        .exchange_bus = mock_exchange,
        .collision_free = mock_collision,
        .context = mock
    };
    return s;
}

static ApproachRequest make_request(const ApproachTrajectoryInfo *info)
{
    ApproachRequest r;
    memset(&r, 0, sizeof(r));
    r.operation = APPROACH_OPERATION_PREVIEW;
    r.trajectory = info;
    r.expected_program_id = info->program_id;
    r.expected_source_revision = info->source_revision;
    r.expected_artifact_crc = info->artifact_crc;
    return r;
}

static ApproachTrajectoryInfo make_info(void)
{
    ApproachTrajectoryInfo info = {
        .valid = true,
        .program_id = 17U,
        .source_revision = 4U,
        .artifact_crc = 0x12345678U,
        .sample_data_crc = 0xABCDEF01U,
        .sample_count = 1001U,
        .sample_period_us = APPROACH_SAMPLE_PERIOD_US
    };
    return info;
}

static void test_direct_motion_and_pause_replan(void)
{
    ApproachState state;
    ApproachTrajectoryInfo info = make_info();
    ApproachConfig config = make_config();
    MockHardware mock;
    ApproachServices services;
    ApproachRequest request;
    ApproachControlInputs inputs = { .motion_permission = true };
    uint32_t ticks;

    memset(&mock, 0, sizeof(mock));
    mock.collision_allowed = true;
    mock.first_units[0] = 30000;
    mock.first_units[1] = -20000;
    services = make_services(&mock);
    request = make_request(&info);

    assert(ApproachState_Enter(&state, &request, &config, &services));
    for (ticks = 0U; ticks < 30000U &&
         state.result == APPROACH_RESULT_RUNNING; ++ticks) {
        inputs.pause_requested = (ticks >= 300U && ticks < 310U);
        (void)ApproachState_Step(&state, &inputs);
    }
    assert(state.result == APPROACH_RESULT_COMPLETE);
    assert(state.replan_count == 1U);
    assert(fabsf(mock.actual[0] - 0.3f) < 0.00002f);
    assert(fabsf(mock.actual[1] + 0.2f) < 0.00002f);
    printf("PASS direct motion + pause/replan (%lu ticks)\n",
           (unsigned long)ticks);
}

static void test_collision_blocks_all_motion(void)
{
    ApproachState state;
    ApproachTrajectoryInfo info = make_info();
    ApproachConfig config = make_config();
    MockHardware mock;
    ApproachServices services;
    ApproachRequest request;
    ApproachControlInputs inputs = { .motion_permission = true };
    uint32_t ticks;

    memset(&mock, 0, sizeof(mock));
    mock.collision_allowed = false;
    mock.first_units[0] = 10000;
    services = make_services(&mock);
    request = make_request(&info);
    assert(ApproachState_Enter(&state, &request, &config, &services));
    for (ticks = 0U; ticks < 100U &&
         state.result == APPROACH_RESULT_RUNNING; ++ticks) {
        (void)ApproachState_Step(&state, &inputs);
    }
    assert(state.result == APPROACH_RESULT_FAILED);
    assert(state.error == APPROACH_ERR_COLLISION);
    assert(mock.exchanges == 0U);
    printf("PASS collision rejected before first motion command\n");
}

static void test_clearance_pose_route(void)
{
    ApproachState state;
    ApproachTrajectoryInfo info = make_info();
    ApproachConfig config = make_config();
    MockHardware mock;
    ApproachServices services;
    ApproachRequest request;
    ApproachControlInputs inputs = { .motion_permission = true };
    static const float clearance[1][APPROACH_DOF] = {
        {0.0f, -0.30f, 0.40f, 0.0f, 0.0f, 0.0f}
    };
    uint32_t ticks;

    memset(&mock, 0, sizeof(mock));
    mock.collision_allowed = true;
    mock.first_units[0] = 20000;
    mock.first_units[2] = 10000;
    services = make_services(&mock);
    request = make_request(&info);
    request.clearance_pose_rad = clearance;
    request.clearance_pose_count = 1U;
    assert(ApproachState_Enter(&state, &request, &config, &services));
    for (ticks = 0U; ticks < 30000U &&
         state.result == APPROACH_RESULT_RUNNING; ++ticks) {
        (void)ApproachState_Step(&state, &inputs);
    }
    assert(state.result == APPROACH_RESULT_COMPLETE);
    assert(state.total_legs == 2U);
    printf("PASS clearance-pose route\n");
}

static void test_pause_during_final_verification(void)
{
    ApproachState state;
    ApproachTrajectoryInfo info = make_info();
    ApproachConfig config = make_config();
    MockHardware mock;
    ApproachServices services;
    ApproachRequest request;
    ApproachControlInputs inputs = { .motion_permission = true };
    uint32_t ticks;
    uint8_t pause_ticks = 0U;

    memset(&mock, 0, sizeof(mock));
    mock.collision_allowed = true;
    mock.first_units[0] = 5000;
    services = make_services(&mock);
    request = make_request(&info);
    assert(ApproachState_Enter(&state, &request, &config, &services));

    for (ticks = 0U; ticks < 10000U &&
         state.result == APPROACH_RESULT_RUNNING; ++ticks) {
        if (state.phase == APPROACH_PHASE_VERIFY_TARGET &&
            state.stable_cycles == 1U && pause_ticks < 3U) {
            inputs.pause_requested = true;
            ++pause_ticks;
        } else {
            inputs.pause_requested = false;
        }
        (void)ApproachState_Step(&state, &inputs);
    }
    assert(state.result == APPROACH_RESULT_COMPLETE);
    assert(state.replan_count == 0U);
    printf("PASS pause during target verification\n");
}

int main(void)
{
    test_direct_motion_and_pause_replan();
    test_collision_blocks_all_motion();
    test_clearance_pose_route();
    test_pause_during_final_verification();
    puts("All Approach State tests passed.");
    return 0;
}
