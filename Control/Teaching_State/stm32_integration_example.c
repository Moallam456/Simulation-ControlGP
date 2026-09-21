/*
 * Integration sketch only; adapt task names and drivers to your project.
 * Do not add this file to the build unchanged.
 */

#if 0
#include "teaching_state.h"

static TeachingState g_teaching;

void App_TeachingInit(void)
{
    const TeachingConfig config = {
        .default_speed_mps = 0.010F,
        .minimum_speed_mps = 0.001F,
        .maximum_speed_mps = 0.100F,
        .speed_step_mps = 0.001F,
        .minimum_point_separation_m = 0.002F,
        .collinearity_epsilon_m2 = 1.0e-10F
    };

    (void)TeachingState_Init(&g_teaching, &config, 1U);
}

void App_TeachingTask(void)
{
    TeachingInputs inputs;
    TeachingOutputs outputs;
    TeachingEvent event;

    /* These application functions must return one coherent input snapshot
       and one debounced rising-edge HMI event. */
    inputs = App_BuildTeachingInputSnapshot();
    event = Hmi_GetNextTeachingEvent();

    (void)TeachingState_ProcessEvent(&g_teaching, &inputs, event, &outputs);
    Hmi_UpdateTeachingScreen(&outputs);

    if (outputs.validation_request)
    {
        /* Transfer a frozen copy/message. Do not let two cores write
           g_teaching.draft concurrently. */
        PathValidation_Submit(&g_teaching.draft,
                              outputs.submitted_revision,
                              outputs.submitted_crc);
        MainState_RequestTransition(STATE_PATH_VALIDATION);
    }
}
#endif
