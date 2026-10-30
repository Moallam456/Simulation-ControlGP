#ifndef ROBOT_VER2_KINEMATICS_H
#define ROBOT_VER2_KINEMATICS_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
#define RB2_DOF 6
#define RB2_MAX_SOLUTIONS 8
/* Row-major transforms; metres/radians; URDF joint signs; TCP, not flange. */
int rb2_fk(const double q[6], double T_base_tcp[16]);
int rb2_jacobian(const double q[6], double J_linear_angular[36]);
/* Base, Link_1..Link_5, End_Effector, TCP; eight row-major transforms. */
int rb2_body_transforms(const double q[6], double transforms[8][16]);
/* Allocation-free closed form, nearest seed first, limits/FK independently checked.
 * Exact wrist-degenerate branches are not enumerated; zero is not an
 * unreachability proof. Collision/route checks are a separate responsibility. */
size_t rb2_ik(const double T_base_tcp[16], const double seed[6],
              const double lower[6], const double upper[6],
              double solutions[RB2_MAX_SOLUTIONS][6]);
#ifdef __cplusplus
}
#endif
#endif
