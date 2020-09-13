#include "renik_chain.h"

void RenIKChain::init(Vector3 p_chain_curve_direction, float p_root_influence, float p_leaf_influence, float p_twist_influence, float p_twist_start) {
	chain_curve_direction = p_chain_curve_direction;
	root_influence = p_root_influence;
	leaf_influence = p_leaf_influence;
	twist_influence = p_twist_influence;
	twist_start = p_twist_start;
}

void RenIKChain::init_chain(Skeleton *skeleton) {
	joints.clear();
	total_length = 0;
	if (skeleton && root_bone >= 0 && leaf_bone >= 0 && root_bone < skeleton->get_bone_count() && leaf_bone < skeleton->get_bone_count()) {
		BoneId bone = skeleton->get_bone_parent(leaf_bone);
		//generate the chain of bones
		Vector<BoneId> chain;
		float last_length;
		rest_leaf = skeleton->get_bone_rest(leaf_bone);
		while (bone != root_bone) {
			Transform rest_pose = skeleton->get_bone_rest(bone);
			rest_leaf = rest_pose * rest_leaf.orthonormalized();
			last_length = rest_pose.origin.length();
			total_length += last_length;
			if (bone < 0) { //invalid chain
				total_length = 0;
				first_bone = -1;
				rest_leaf = Transform();
				return;
			}
			chain.push_back(bone);
			first_bone = bone;
			bone = skeleton->get_bone_parent(bone);
		}
		total_length -= last_length;
		total_length += skeleton->get_bone_rest(leaf_bone).origin.length();

		if (total_length <= 0) { //invalid chain
			total_length = 0;
			first_bone = -1;
			rest_leaf = Transform();
			return;
		}

		Basis totalRotation;
		float progress = 0;
		//flip the order and figure out the relative distances of these joints
		for (int i = chain.size() - 1; i >= 0; i--) {
			RenIKChain::Joint j;
			j.id = chain[i];
			Transform boneTransform = skeleton->get_bone_rest(j.id);
			j.rotation = boneTransform.basis.get_rotation_quat();
			j.relative_prev = totalRotation.xform_inv(boneTransform.origin);
			j.prev_distance = j.relative_prev.length();

			//calc influences
			progress += j.prev_distance;
			float percentage = (progress / total_length);
			float effectiveRootInfluence = root_influence <= 0 || percentage >= root_influence ? 0 : (percentage - root_influence) / -root_influence;
			float effectiveLeafInfluence = leaf_influence <= 0 || percentage <= 1 - leaf_influence ? 0 : (percentage - (1 - leaf_influence)) / leaf_influence;
			float effectiveTwistInfluence = twist_start >= 1 || twist_influence <= 0 || percentage <= twist_start ? 0 : (percentage - twist_start) * (twist_influence / (1 - twist_start));
			j.root_influence = effectiveRootInfluence > 1 ? 1 : effectiveRootInfluence;
			j.leaf_influence = effectiveLeafInfluence > 1 ? 1 : effectiveLeafInfluence;
			j.twist_influence = effectiveTwistInfluence > 1 ? 1 : effectiveTwistInfluence;

			if (!joints.empty()) {
				RenIKChain::Joint oldJ = joints[joints.size() - 1];
				oldJ.relative_next = -j.relative_prev;
				oldJ.next_distance = j.prev_distance;
				joints.set(joints.size() - 1, oldJ);
			}
			joints.push_back(j);
			totalRotation = (totalRotation * boneTransform.basis).orthonormalized();
		}
		if (!joints.empty()) {
			RenIKChain::Joint oldJ = joints[joints.size() - 1];
			oldJ.relative_next = -skeleton->get_bone_rest(leaf_bone).origin;
			oldJ.next_distance = oldJ.relative_next.length();
			joints.set(joints.size() - 1, oldJ);
		}
	}
}

void RenIKChain::set_root_bone(Skeleton *skeleton, BoneId p_root_bone) {
	root_bone = p_root_bone;
	init_chain(skeleton);
}
void RenIKChain::set_leaf_bone(Skeleton *skeleton, BoneId p_leaf_bone) {
	leaf_bone = p_leaf_bone;
	init_chain(skeleton);
}

bool RenIKChain::is_valid() {
	return !joints.empty();
}

float RenIKChain::get_total_length() {
	return total_length;
}

Vector<RenIKChain::Joint> RenIKChain::get_joints() {
	return joints;
}

Transform RenIKChain::get_relative_rest_leaf() {
	return rest_leaf;
}

BoneId RenIKChain::get_first_bone() {
	return first_bone;
}

BoneId RenIKChain::get_root_bone() {
	return root_bone;
}

BoneId RenIKChain::get_leaf_bone() {
	return leaf_bone;
}

float RenIKChain::get_root_stiffness() {
	return root_influence;
}

void RenIKChain::set_root_stiffness(Skeleton *skeleton, float stiffness) {
	root_influence = stiffness;
	init_chain(skeleton);
}

float RenIKChain::get_leaf_stiffness() {
	return leaf_influence;
}

void RenIKChain::set_leaf_stiffness(Skeleton *skeleton, float stiffness) {
	leaf_influence = stiffness;
	init_chain(skeleton);
}

float RenIKChain::get_twist() {
	return twist_influence;
}

void RenIKChain::set_twist(Skeleton *skeleton, float p_twist) {
	twist_influence = p_twist;
	init_chain(skeleton);
}

float RenIKChain::get_twist_start() {
	return twist_start;
}

void RenIKChain::set_twist_start(Skeleton *skeleton, float p_twist_start) {
	twist_start = p_twist_start;
	init_chain(skeleton);
}

bool RenIKChain::contains_bone(Skeleton *skeleton, BoneId bone) {
	if (skeleton) {
		BoneId spineBone = leaf_bone;
		while (spineBone >= 0) {
			if(spineBone == bone){
				return true;
			}
			spineBone = skeleton->get_bone_parent(spineBone);
		}
	}
	return false;
}
