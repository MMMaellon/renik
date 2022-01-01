/*************************************************************************/
/*  test_renik.h                                                         */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2020 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2020 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef TEST_RENIK_H
#define TEST_RENIK_H

#include "core/math/basis.h"

#include "tests/test_macros.h"

namespace TestRenIK {

/*
RenIK Unit Testing
These tests are for testing the small helper functions within RenIK
For example, we'll test the Fabrik algorithm, the quaternion rotation
algorithms, and some basic foot placement logic
*/
TEST_CASE("[Modules][RENIK] math") {
  Quaternion rightAngle =
      RenIKHelper::align_vectors(Vector3(1, 0, 0), Vector3(0, 1, 0));
  Quaternion rightAngleCheck =
      Quaternion(Vector3(0, 0, 1), Math::deg2rad(90.0));
  CHECK_MESSAGE(rightAngle == rightAngleCheck,
                String("align_vectors").utf8().ptr());
  Quaternion noRotation =
      RenIKHelper::align_vectors(Vector3(1, 0, 0), Vector3(0.5, 0, 0));
  Quaternion noRotationCheck = Quaternion(Vector3(0, 0, 1), 0);
  CHECK_MESSAGE(noRotation == noRotationCheck,
                String("align_vectors 2").utf8().ptr());
  CHECK_MESSAGE(Vector3(1, 0, 0).angle_to(Vector3(0, 1, 0)), Math_PI / 2,
                String("math 1").utf8().ptr());
  CHECK_MESSAGE(Vector3(1, 0, 0).angle_to(Vector3(0, 0, 1)), Math_PI / 2,
                String("math 2").utf8().ptr());
  CHECK_MESSAGE(Vector3(1, 0, 0).angle_to(Vector3(0, 1, 1)), Math_PI / 2,
                String("math 3").utf8().ptr());
  CHECK_MESSAGE(Vector3(1, 0, 0).angle_to(Vector3(1, 1, 0)), Math_PI / 4,
                String("math 4").utf8().ptr());
  CHECK_MESSAGE(Vector3(1, 0, 0).angle_to(Vector3(-1, 0, 0)), Math_PI,
                String("math 5").utf8().ptr());
  CHECK_MESSAGE(Vector3(1, 0, 0).angle_to(Vector3(-1, -1, 0)), Math_PI * .75,
                String("math 6").utf8().ptr());
  CHECK_MESSAGE(Vector3(3, 7, -13).angle_to(Vector3(-14, -12, -10)), 1.558139,
                String("math 7").utf8().ptr());
}

} // namespace TestRenIK

#endif // TEST_REINK_H
