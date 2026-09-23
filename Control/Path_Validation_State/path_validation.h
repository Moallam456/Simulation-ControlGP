#ifndef PATH_VALIDATION_H
#define PATH_VALIDATION_H

#include "path_validation_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool PathValidation_Init(PathValidationState *state,
                         const PathValidationConfig *config,
                         const PathValidationServices *services,
                         const PathValidationStorage *storage,
                         ValidatedTrajectory *artifact_storage);

bool PathValidation_Start(PathValidationState *state,
                          const TaughtProgram *frozen_program,
                          uint32_t submitted_revision,
                          uint32_t submitted_crc);

/* Executes at most sample_budget trajectory samples. Call periodically. */
PathValidationResult PathValidation_Step(PathValidationState *state,
                                         uint16_t sample_budget);

void PathValidation_Cancel(PathValidationState *state);
/* Same field-wise CRC-32 contract used when Teaching freezes the draft. */
uint32_t PathValidation_CalculateDraftCrc(const TaughtProgram *program);
const PathValidationReport *PathValidation_GetReport(const PathValidationState *state);
const char *PathValidation_PhaseName(PathValidationPhase phase);
const char *PathValidation_ErrorName(PathValidationError error);

#ifdef __cplusplus
}
#endif
#endif
