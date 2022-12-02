#ifndef RENIKHELPER_H
#define RENIKHELPER_H
#ifndef _3D_DISABLED

#include <scene/main/node.h>

namespace RenIKHelper {
Quaternion align_vectors(Vector3 a, Vector3 b, float influence = 1);
float smoothCurve(float number, float modifier = 0.5);
Vector3 vector_rejection(Vector3 v, Vector3 normal);
float safe_acos(float f);
float safe_asin(float f);
Vector3 get_perpendicular_vector(Vector3 v);

float log_clamp(float value, float target, float looseness);
Vector3 log_clamp(Vector3 vector, Vector3 target, float looseness);
Quaternion log_clamp(Quaternion quat, Quaternion target, float looseness);
Basis log_clamp(Basis basis, Basis target, float looseness);
} // namespace RenIKHelper

#endif
#endif
