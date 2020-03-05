#include "renTest.h"

RenIKTest::RenIKTest(){};

void RenIKTest::test() {
	printf("Running RenIK Unit Tests...\n");
	Quat rightAngle = RenIK::align_vectors(Vector3(1, 0, 0), Vector3(0, 1, 0));
	Quat rightAngleCheck = Quat(Vector3(0, 0, 1), Math::deg2rad(90.0));
	assert(rightAngle == rightAngleCheck);
	Quat noRotation = RenIK::align_vectors(Vector3(1, 0, 0), Vector3(0.5, 0, 0));
	Quat noRotationCheck = Quat(Vector3(0, 0, 1), 0);
	assert(noRotation == noRotationCheck);

	assert(Vector3(1, 0, 0).angle_to(Vector3(0, 1, 0)), Math_PI / 2);
	assert(Vector3(1, 0, 0).angle_to(Vector3(0, 0, 1)), Math_PI / 2);
	assert(Vector3(1, 0, 0).angle_to(Vector3(0, 1, 1)), Math_PI / 2);
	assert(Vector3(1, 0, 0).angle_to(Vector3(1, 1, 0)), Math_PI / 4);
	assert(Vector3(1, 0, 0).angle_to(Vector3(-1, 0, 0)), Math_PI);
	assert(Vector3(1, 0, 0).angle_to(Vector3(-1, -1, 0)), Math_PI * .75);
	assert(Vector3(3, 7, -13).angle_to(Vector3(-14, -12, -10)), 1.558139);

	printf("Unit Tests Complete!\n");
}

void RenIKTest::assert(bool assertion) {
	if (!assertion) {
		printf("ASSERTION FAILED\n");
	}
}
void RenIKTest::assert(float value1, float value2) {
	if (abs(value1 - value2) > 0.01) {
		printf("ASSERTION FAILED! %f != %f\n", value1, value2);
	}
}