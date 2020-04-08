#ifndef RENIK_CHAIN_H
#define RENIK_CHAIN_H

#include <scene/3d/skeleton.h>

struct RenIKChain : public Reference {
	GDCLASS(RenIKChain, Reference);

	BoneId root_bone = -1;
	BoneId leaf_bone = -1;

	Vector<Joint> joints;
	Vector<float> bone_lengths;
	float total_length = 0;
	Transform rest_start;

public:
	struct Joint {
		Quat rotation;
		BoneId id;
		Vector3 relative_prev;
		Vector3 relative_next;
		float prev_distance = 0;
		float next_distance = 0;

		float root_influence = 0;
		float leaf_influence = 0;
		float twist_influence = 0;
	};
	void set_chain(Skeleton *skeleton, BoneId p_root_bone, BoneId p_leaf_bone);
	bool is_valid();
	float twist_influence = 0; //How much the chain tries to twist to follow the end when the start is facing a different direction
	float twist_start = 0; //Where along the chain the twisting starts
	Vector3 chain_curve_direction; //This defines which way to prebend it
	float root_influence = 0; //how much the start bone is influenced by the root rotation
	float leaf_influence = 0; //how much the end bone is influenced by the goal rotation
	float get_total_length();
	Vector<Vector3> get_joints();
};

#endif