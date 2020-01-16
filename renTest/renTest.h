/*
RenIK Unit Testing
These tests are only meant for testing the small helper functions within RenIK
For example, we'll test the Fabrik algorithm, the quaternion rotation algorithms, and some basic foot placement logic
*/

#ifndef RENIK_UNIT_TEST_H
#define RENIK_UNIT_TEST_H

#include "../renik.h"

class RenIKTest : public Object {
	GDCLASS(RenIKTest, Object);
	RenIK ik;

public:
	RenIKTest();

	static void test();
};

#endif