#include "renik_limb.h"

void RenIKLimb::init(float p_upper_twist_offset, float p_lower_twist_offset, float p_roll_offset, float p_upper_limb_twist, float p_lower_limb_twist, float p_twist_inflection_point_offset, float p_twist_overflow, float p_target_rotation_influence, Vector3 p_pole_offset, Vector3 p_target_position_influence) {
	upper_twist_offset = p_upper_twist_offset;
	lower_twist_offset = p_lower_twist_offset;
	roll_offset = p_roll_offset;
	upper_limb_twist = p_upper_limb_twist;
	lower_limb_twist = p_lower_limb_twist;
	twist_inflection_point_offset = p_twist_inflection_point_offset;
	twist_overflow = p_twist_overflow;
	target_rotation_influence = p_target_rotation_influence;
	pole_offset = p_pole_offset;
	target_position_influence = p_target_position_influence;
}

void RenIKLimb::set_leaf(Skeleton *skeleton, BoneId p_leaf_id) {
	leaf_id = p_leaf_id;
	if (skeleton && leaf_id >= 0) {
		lower_id = skeleton->get_bone_parent(leaf_id);
		if (lower_id >= 0) {
			upper_id = skeleton->get_bone_parent(lower_id);
			if (upper_id >= 0) {
				leaf = skeleton->get_bone_rest(leaf_id);
				lower = skeleton->get_bone_rest(lower_id);
				upper = skeleton->get_bone_rest(upper_id);
			}
		}
	}
}

bool RenIKLimb::is_valid() {
	return upper_id >= 0; //upper id can only exist if leaf_id and lower_id exist
}

BoneId RenIKLimb::get_leaf_bone() {
	return leaf_id;
}
BoneId RenIKLimb::get_lower_bone() {
	return lower_id;
}
BoneId RenIKLimb::get_upper_bone() {
	return upper_id;
}
Transform RenIKLimb::get_upper() {
	return upper;
}
Transform RenIKLimb::get_lower() {
	return lower;
}
Transform RenIKLimb::get_leaf() {
	return leaf;
}
