#include "renTest.h"

RenIKTest::RenIKTest(){};

void RenIKTest::test() {
	printf("Running RenIK Unit Tests...\n");
	Quat rightAngle = RenIKHelper::align_vectors(Vector3(1, 0, 0), Vector3(0, 1, 0));
	Quat rightAngleCheck = Quat(Vector3(0, 0, 1), Math::deg2rad(90.0));
	assert("align_vectors", rightAngle == rightAngleCheck);
	Quat noRotation = RenIKHelper::align_vectors(Vector3(1, 0, 0), Vector3(0.5, 0, 0));
	Quat noRotationCheck = Quat(Vector3(0, 0, 1), 0);
	assert("align_vectors 2", noRotation == noRotationCheck);

	assert("math 1", Vector3(1, 0, 0).angle_to(Vector3(0, 1, 0)), Math_PI / 2);
	assert("math 2", Vector3(1, 0, 0).angle_to(Vector3(0, 0, 1)), Math_PI / 2);
	assert("math 3", Vector3(1, 0, 0).angle_to(Vector3(0, 1, 1)), Math_PI / 2);
	assert("math 4", Vector3(1, 0, 0).angle_to(Vector3(1, 1, 0)), Math_PI / 4);
	assert("math 5", Vector3(1, 0, 0).angle_to(Vector3(-1, 0, 0)), Math_PI);
	assert("math 6", Vector3(1, 0, 0).angle_to(Vector3(-1, -1, 0)), Math_PI * .75);
	assert("math 7", Vector3(3, 7, -13).angle_to(Vector3(-14, -12, -10)), 1.558139);

	printf("Unit Tests Complete!\n");
}

void RenIKTest::assert(char* test_name, bool assertion) {
	if (!assertion) {
		printf("ASSERTION FAILED %s\n", test_name);
	}
}
void RenIKTest::assert(char* test_name, float value1, float value2) {
	if (abs(value1 - value2) > 0.01) {
		printf("ASSERTION FAILED! %f != %f\n", value1, value2);
	}
}