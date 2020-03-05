#include "renik_limb.h"

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