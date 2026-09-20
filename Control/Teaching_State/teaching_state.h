#ifndef TEACHING_STATE_H
#define TEACHING_STATE_H

#include "teaching_state_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the state and erase its RAM draft. */
bool TeachingState_Init(TeachingState *state,
                        const TeachingConfig *config,
                        uint32_t program_id);

/*
 * Process at most one edge-triggered HMI event.
 * Inputs must be a coherent snapshot captured by the application layer.
 */
bool TeachingState_ProcessEvent(TeachingState *state,
                                const TeachingInputs *inputs,
                                TeachingEvent event,
                                TeachingOutputs *outputs);

/* Refresh HMI/status outputs when no event occurs. */
void TeachingState_GetOutputs(const TeachingState *state,
                              TeachingOutputs *outputs);

/* Immediate operational reset: erases the draft without confirmation. */
void TeachingState_ResetDraft(TeachingState *state);

/* Optional selections without adding HMI buttons. */
bool TeachingState_SetCircleDirection(TeachingState *state,
                                      TeachingCircleDirection direction);
bool TeachingState_SetOrientationMode(TeachingState *state,
                                      TeachingOrientationMode mode);

const char *TeachingState_PhaseName(TeachingPhase phase);
const char *TeachingState_ErrorName(TeachingError error);
const char *TeachingState_SegmentName(TeachingSegmentType type);

#ifdef __cplusplus
}
#endif

#endif /* TEACHING_STATE_H */
