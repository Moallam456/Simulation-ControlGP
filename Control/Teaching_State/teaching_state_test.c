#include "teaching_state.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static TeachingInputs valid_inputs(float x, float y, float z, uint32_t time_ms)
{
    TeachingInputs in;
    memset(&in, 0, sizeof(in));
    in.actual_position_m[0] = x;
    in.actual_position_m[1] = y;
    in.actual_position_m[2] = z;
    in.actual_orientation_quat[0] = 1.0F;
    in.timestamp_ms = time_ms;
    in.calibration_version = 7U;
    in.active_frame_id = 2U;
    in.active_tool_id = 1U;
    in.measurement_valid = true;
    in.robot_motion_settled = true;
    in.manual_guidance_active = true;
    in.motion_permitted = true;
    in.drives_ready = true;
    in.robot_homed = true;
    return in;
}

int main(void)
{
    TeachingState state;
    TeachingOutputs out;
    TeachingConfig cfg = {
        .default_speed_mps = 0.010F,
        .minimum_speed_mps = 0.001F,
        .maximum_speed_mps = 0.100F,
        .speed_step_mps = 0.001F,
        .minimum_point_separation_m = 0.002F,
        .collinearity_epsilon_m2 = 1.0e-10F
    };
    TeachingInputs p1 = valid_inputs(0.30F, 0.10F, 0.20F, 1000U);
    TeachingInputs p2 = valid_inputs(0.50F, 0.10F, 0.20F, 2000U);

    assert(TeachingState_Init(&state, &cfg, 42U));
    assert(TeachingState_ProcessEvent(&state, &p1, TEACH_EVENT_SELECT_LINE, &out));
    assert(TeachingState_ProcessEvent(&state, &p1, TEACH_EVENT_RECORD_POINT, &out));
    assert(!TeachingState_ProcessEvent(&state, &p1, TEACH_EVENT_RECORD_POINT, &out));
    assert(out.error == TEACH_ERR_DUPLICATE_POINT);
    assert(TeachingState_ProcessEvent(&state, &p2, TEACH_EVENT_RECORD_POINT, &out));
    assert(state.draft.segment_count == 1U);
    assert(TeachingState_ProcessEvent(&state, &p2, TEACH_EVENT_VALIDATE_PATH, &out));
    assert(out.validation_request);
    assert(out.submitted_crc != 0U);

    /* Reset is immediate and does not require a second event. */
    assert(TeachingState_ProcessEvent(&state, &p2, TEACH_EVENT_RESET, &out));
    assert(out.draft_was_reset);
    assert(state.draft.segment_count == 0U);
    assert(state.phase == TEACH_PHASE_WAIT_SEGMENT_SELECTION);

    puts("Teaching state tests passed.");
    return 0;
}
