/**************************************************************************/
/*  qcp.cpp                                                               */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "qcp.h"

RenQCP::RenQCP(double p_evec_prec, double p_eval_prec) {
	evec_prec = p_eval_prec;
	eval_prec = p_evec_prec;
}

void RenQCP::set(PackedVector3Array &r_target, PackedVector3Array &r_moved) {
	target = r_target;
	moved = r_moved;
	rmsd_calculated = false;
	transformation_calculated = false;
	inner_product_calculated = false;
}

double RenQCP::get_rmsd() {
	if (!rmsd_calculated) {
		calculate_rmsd(moved, target);
		rmsd_calculated = true;
	}
	return rmsd;
}

Quaternion RenQCP::get_rotation() {
	Quaternion result;
	if (!transformation_calculated) {
		if (!inner_product_calculated) {
			inner_product(target, moved);
		}
		result = calculate_rotation();
		transformation_calculated = true;
	}
	return result;
}

void RenQCP::calculate_rmsd(double r_length) {
	rmsd = Math::sqrt(Math::abs(2.0f * (e0 - max_eigenvalue) / r_length));
}

Quaternion RenQCP::calculate_rotation() {
	Quaternion result;

	if (moved.size() == 1) {
		Vector3 u = moved[0];
		Vector3 v = target[0];
		double norm_product = u.length() * v.length();

		if (norm_product == 0.0) {
			return Quaternion();
		}

		double dot = u.dot(v);

		if (dot < ((2.0e-15 - 1.0) * norm_product)) {
			Vector3 w = u.normalized();
			result = Quaternion(-w.x, -w.y, -w.z, 0.0f).normalized();
		} else {
			double q0 = Math::sqrt(0.5 * (1.0 + dot / norm_product));
			double coeff = 1.0 / (2.0 * q0 * norm_product);
			Vector3 q = v.cross(u);
			result = Quaternion(-coeff * q.x, -coeff * q.y, -coeff * q.z, q0).normalized();
		}
	} else {
		double a13 = -sum_xz_minus_zx;
		double a14 = sum_xy_minus_yx;
		double a21 = sum_yz_minus_zy;
		double a22 = sum_xx_minus_yy - sum_zz - max_eigenvalue;
		double a23 = sum_xy_plus_yx;
		double a24 = sum_xz_plus_zx;
		double a31 = a13;
		double a32 = a23;
		double a33 = sum_yy - sum_xx - sum_zz - max_eigenvalue;
		double a34 = sum_yz_plus_zy;
		double a41 = a14;
		double a42 = a24;
		double a43 = a34;
		double a44 = sum_zz - sum_xx_plus_yy - max_eigenvalue;

		double a3344_4334 = a33 * a44 - a43 * a34;
		double a3244_4234 = a32 * a44 - a42 * a34;
		double a3243_4233 = a32 * a43 - a42 * a33;
		double a3143_4133 = a31 * a43 - a41 * a33;
		double a3144_4134 = a31 * a44 - a41 * a34;
		double a3142_4132 = a31 * a42 - a41 * a32;

		double quaternion_w = a22 * a3344_4334 - a23 * a3244_4234 + a24 * a3243_4233;
		double quaternion_x = -a21 * a3344_4334 + a23 * a3144_4134 - a24 * a3143_4133;
		double quaternion_y = a21 * a3244_4234 - a22 * a3144_4134 + a24 * a3142_4132;
		double quaternion_z = -a21 * a3243_4233 + a22 * a3143_4133 - a23 * a3142_4132;
		double qsqr = quaternion_w * quaternion_w + quaternion_x * quaternion_x + quaternion_y * quaternion_y + quaternion_z * quaternion_z;

		if (qsqr < evec_prec) {
			return Quaternion();
		}

		quaternion_x *= -1;
		quaternion_y *= -1;
		quaternion_z *= -1;
		real_t min = quaternion_w;
		min = quaternion_x < min ? quaternion_x : min;
		min = quaternion_y < min ? quaternion_y : min;
		min = quaternion_z < min ? quaternion_z : min;
		quaternion_w /= min;
		quaternion_x /= min;
		quaternion_y /= min;
		quaternion_z /= min;

		result = Quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w).normalized();
	}

	return result;
}

double RenQCP::get_rmsd(PackedVector3Array &r_fixed, PackedVector3Array &r_moved) {
	set(r_fixed, r_moved);
	return get_rmsd();
}

void RenQCP::translate(Vector3 r_translate, PackedVector3Array &r_x) {
	for (Vector3 &p : r_x) {
		p += r_translate;
	}
}

Vector3 RenQCP::get_translation() {
	return target_center - moved_center;
}

Vector3 RenQCP::move_to_weighted_center(PackedVector3Array &r_to_center, Vector<real_t> &r_weight) {
	Vector3 center;
	real_t total_weight = 0;
	bool weight_is_empty = r_weight.is_empty();
	int size = r_to_center.size();

	for (int i = 0; i < size; i++) {
		if (!weight_is_empty) {
			total_weight += r_weight[i];
			center += r_to_center[i] * r_weight[i];
		} else {
			center += r_to_center[i];
			total_weight++;
		}
	}

	if (total_weight > 0) {
		center /= total_weight;
	}

	return center;
}

void RenQCP::inner_product(PackedVector3Array &coords1, PackedVector3Array &coords2) {
	Vector3 weighted_coord1, weighted_coord2;
	double sum_of_squares1 = 0, sum_of_squares2 = 0;

	sum_xx = 0;
	sum_xy = 0;
	sum_xz = 0;
	sum_yx = 0;
	sum_yy = 0;
	sum_yz = 0;
	sum_zx = 0;
	sum_zy = 0;
	sum_zz = 0;

	bool weight_is_empty = weight.is_empty();
	int size = coords1.size();

	for (int i = 0; i < size; i++) {
		if (!weight_is_empty) {
			weighted_coord1 = weight[i] * coords1[i];
			sum_of_squares1 += weighted_coord1.dot(coords1[i]);
		} else {
			weighted_coord1 = coords1[i];
			sum_of_squares1 += weighted_coord1.dot(weighted_coord1);
		}

		weighted_coord2 = coords2[i];

		sum_of_squares2 += weight_is_empty ? weighted_coord2.dot(weighted_coord2) : (weight[i] * weighted_coord2.dot(weighted_coord2));

		sum_xx += (weighted_coord1.x * weighted_coord2.x);
		sum_xy += (weighted_coord1.x * weighted_coord2.y);
		sum_xz += (weighted_coord1.x * weighted_coord2.z);

		sum_yx += (weighted_coord1.y * weighted_coord2.x);
		sum_yy += (weighted_coord1.y * weighted_coord2.y);
		sum_yz += (weighted_coord1.y * weighted_coord2.z);

		sum_zx += (weighted_coord1.z * weighted_coord2.x);
		sum_zy += (weighted_coord1.z * weighted_coord2.y);
		sum_zz += (weighted_coord1.z * weighted_coord2.z);
	}

	double initial_eigenvalue = (sum_of_squares1 + sum_of_squares2) * 0.5;

	sum_xz_plus_zx = sum_xz + sum_zx;
	sum_yz_plus_zy = sum_yz + sum_zy;
	sum_xy_plus_yx = sum_xy + sum_yx;
	sum_yz_minus_zy = sum_yz - sum_zy;
	sum_xz_minus_zx = sum_xz - sum_zx;
	sum_xy_minus_yx = sum_xy - sum_yx;
	sum_xx_plus_yy = sum_xx + sum_yy;
	sum_xx_minus_yy = sum_xx - sum_yy;
	max_eigenvalue = initial_eigenvalue;

	inner_product_calculated = true;
}

void RenQCP::calculate_rmsd(PackedVector3Array &x, PackedVector3Array &y) {
	// QCP doesn't handle alignment of single values, so if we only have one point
	// we just compute regular distance.
	if (x.size() == 1) {
		rmsd = x[0].distance_to(y[0]);
		rmsd_calculated = true;
	} else {
		if (!inner_product_calculated) {
			inner_product(y, x);
		}
		calculate_rmsd(w_sum);
	}
}

Quaternion RenQCP::weighted_superpose(PackedVector3Array &p_moved, PackedVector3Array &p_target, Vector<real_t> &p_weight, bool translate) {
	set(p_moved, p_target, p_weight, translate);
	return get_rotation();
}

void RenQCP::set(PackedVector3Array &p_moved, PackedVector3Array &p_target, Vector<real_t> &p_weight, bool p_translate) {
	rmsd_calculated = false;
	transformation_calculated = false;
	inner_product_calculated = false;

	moved = p_moved;
	target = p_target;
	weight = p_weight;

	if (p_translate) {
		moved_center = move_to_weighted_center(moved, weight);
		w_sum = 0; // set wsum to 0 so we don't double up.
		target_center = move_to_weighted_center(target, weight);
		translate(moved_center * -1, moved);
		translate(target_center * -1, target);
	} else {
		if (!p_weight.is_empty()) {
			for (int i = 0; i < p_weight.size(); i++) {
				w_sum += p_weight[i];
			}
		} else {
			w_sum = p_moved.size();
		}
	}
}

Quaternion RenQCP::compute_reference_and_target_positions(const Vector<Transform3D> &global_transforms,
		const Transform3D &target,
		const Vector3 &priority,
		Vector<Vector3> &rest_positions,
		Vector<Vector3> &target_positions,
		Vector<real_t> &weights) {
	int num_joints = global_transforms.size();

	Vector3 target_origin = target.origin;
	Vector3 target_col_0 = target.basis.get_column(0);
	Vector3 target_col_1 = target.basis.get_column(1);
	Vector3 target_col_2 = target.basis.get_column(2);
	Vector3 target_col_0_priority = target_col_0 * priority.x;
	Vector3 target_col_1_priority = target_col_1 * priority.y;
	Vector3 target_col_2_priority = target_col_2 * priority.z;

	for (int joint_i = 0; joint_i < num_joints; ++joint_i) {
		const Transform3D &current_transform = global_transforms[joint_i];
		Vector3 col_0 = current_transform.basis.get_column(0);
		Vector3 col_1 = current_transform.basis.get_column(1);
		Vector3 col_2 = current_transform.basis.get_column(2);

		int idx = joint_i * 7;
		rest_positions.write[idx + 0] = current_transform.origin;
		rest_positions.write[idx + 1] = col_0;
		rest_positions.write[idx + 2] = col_1;
		rest_positions.write[idx + 3] = col_2;
		rest_positions.write[idx + 4] = col_0 * priority.x;
		rest_positions.write[idx + 5] = col_1 * priority.y;
		rest_positions.write[idx + 6] = col_2 * priority.z;

		target_positions.write[idx + 0] = target_origin;
		target_positions.write[idx + 1] = target_col_0;
		target_positions.write[idx + 2] = target_col_1;
		target_positions.write[idx + 3] = target_col_2;
		target_positions.write[idx + 4] = target_col_0_priority;
		target_positions.write[idx + 5] = target_col_1_priority;
		target_positions.write[idx + 6] = target_col_2_priority;
	}

	Quaternion solved_global_pose = weighted_superpose(rest_positions, target_positions, weights, false);
	return solved_global_pose;
}
