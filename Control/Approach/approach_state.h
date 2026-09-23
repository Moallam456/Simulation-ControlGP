#ifndef APPROACH_STATE_H
#define APPROACH_STATE_H

#include "approach_state_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void ApproachState_Init(ApproachState *state);

bool ApproachState_Enter(ApproachState *state,
                         const ApproachRequest *request,
                         const ApproachConfig *config,
                         const ApproachServices *services);

/* Call exactly once per 1 ms supervisory/control tick. */
ApproachResult ApproachState_Step(ApproachState *state,
                                  const ApproachControlInputs *inputs);

void ApproachState_GetReport(const ApproachState *state,
                             ApproachReport *report);

const char *ApproachState_PhaseString(ApproachPhase phase);
const char *ApproachState_ErrorString(ApproachError error);

#ifdef __cplusplus
}
#endif
#endif
