#include "approach_project_adapter.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    PvExecutionSample sample_zero;
    int32_t actual_units[APPROACH_DOF];
    uint32_t writes;
    uint32_t exchanges;
} ProjectMock;

static bool read_sample(uint32_t index, PvExecutionSample *sample, void *ctx)
{
    ProjectMock *mock = (ProjectMock *)ctx;
    if (index != 0U) {
        return false;
    }
    *sample = mock->sample_zero;
    return true;
}

static bool read_drive(int32_t actual[APPROACH_DOF],
                       bool ready[APPROACH_DOF], void *ctx)
{
    ProjectMock *mock = (ProjectMock *)ctx;
    uint32_t j;
    memcpy(actual, mock->actual_units, sizeof(mock->actual_units));
    for (j = 0U; j < APPROACH_DOF; ++j) {
        ready[j] = true;
    }
    return true;
}

static bool write_drive(const int32_t target[APPROACH_DOF], void *ctx)
{
    ProjectMock *mock = (ProjectMock *)ctx;
    memcpy(mock->actual_units, target, sizeof(mock->actual_units));
    ++mock->writes;
    return true;
}

static bool exchange(void *ctx)
{
    ++((ProjectMock *)ctx)->exchanges;
    return true;
}

static bool collision_free(const float q[APPROACH_DOF], void *ctx)
{
    (void)q;
    (void)ctx;
    return true;
}

int main(void)
{
    ValidatedTrajectory trajectory;
    PathValidationConfig pv;
    ApproachProjectAdapter adapter;
    ApproachRequest request;
    ApproachServices services;
    ApproachConfig config;
    ApproachState state;
    ApproachControlInputs inputs = { .motion_permission = true };
    ProjectMock mock;
    float jerk[APPROACH_DOF];
    uint32_t j;
    uint32_t ticks;

    memset(&trajectory, 0, sizeof(trajectory));
    memset(&pv, 0, sizeof(pv));
    memset(&mock, 0, sizeof(mock));

    trajectory.program_id = 42U;
    trajectory.source_revision = 7U;
    trajectory.artifact_crc = 0x11112222U;
    trajectory.sample_data_crc = 0x33334444U;
    trajectory.sample_count = 2000U;
    trajectory.sample_period_us = PV_SAMPLE_PERIOD_US;

    for (j = 0U; j < APPROACH_DOF; ++j) {
        pv.joint_min_rad[j] = -3.0f;
        pv.joint_max_rad[j] = 3.0f;
        pv.joint_velocity_max_rad_s[j] = 1.5f;
        pv.joint_acceleration_max_rad_s2[j] = 3.0f;
        pv.drive_units_per_joint_rad[j] = 100000.0f + 1000.0f * (float)j;
        pv.drive_zero_offset_units[j] = (int32_t)(100 * (int32_t)j);
        pv.joint_direction_sign[j] = (j & 1U) == 0U ? 1 : -1;
        jerk[j] = 20.0f;
    }
    pv.check_joint_acceleration = true;

    /* Sample zero is already in each drive's native calibrated units. */
    mock.sample_zero.target_position_units[0] = 25100;
    mock.sample_zero.target_position_units[1] = -30100;
    mock.sample_zero.target_position_units[2] = 10400;

    assert(ApproachProjectAdapter_Init(
        &adapter, &trajectory, &pv, true, read_sample, &mock,
        read_drive, write_drive, exchange, &mock,
        collision_free, &mock));
    assert(ApproachProjectAdapter_BuildRequest(
        &adapter, APPROACH_OPERATION_PREVIEW, NULL, 0U, &request));
    assert(ApproachProjectAdapter_BuildServices(&adapter, &services));
    assert(ApproachProjectAdapter_BuildConfig(
        &pv, jerk, true, 1.10f, 0.05f, 10.0f,
        0.001f, 0.1f, 5U, 100U, 64U, true, &config));
    assert(ApproachState_Enter(&state, &request, &config, &services));

    for (ticks = 0U; ticks < 20000U &&
         state.result == APPROACH_RESULT_RUNNING; ++ticks) {
        (void)ApproachState_Step(&state, &inputs);
    }

    assert(state.result == APPROACH_RESULT_COMPLETE);
    assert(mock.writes > 0U && mock.exchanges == mock.writes);
    for (j = 0U; j < APPROACH_DOF; ++j) {
        assert(mock.actual_units[j] ==
               mock.sample_zero.target_position_units[j]);
    }
    printf("PASS Path Validation adapter; exact sample zero reached in %lu ticks\n",
           (unsigned long)ticks);
    return 0;
}
