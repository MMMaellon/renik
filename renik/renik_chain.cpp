#include "renik_chain.h"

void RenIKChain::set_chain(Skeleton *skeleton, BoneId p_start_bone, BoneId p_end_bone) {
	start_bone = p_start_bone;
	end_bone = p_end_bone;
	bones.clear();
	if (skeleton && start_bone >= 0 && end_bone >= 0) {
		BoneId bone = start_bone;
		while (bone >= 0 && bone != end_bone) {
			bones.push_back(skeleton->get_bone_rest(bone));
			bone = skeleton->get_bone_parent(bone);
		}
		if (bone >= 0) {
			bones.push_back(skeleton->get_bone_rest(bone));
		}
	}
}

BoneId RenIKChain::get_start_bone() {
	return start_bone;
}

BoneId RenIKChain::get_end_bone() {
	return end_bone;
}

bool RenIKChain::is_valid() {
	return bones.size() > 0;
}