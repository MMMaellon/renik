#ifndef RENIK_CHAIN_H
#define RENIK_CHAIN_H

#include <scene/3d/skeleton.h>

struct RenIKChain : public Reference {
	GDCLASS(RenIKChain, Reference);

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
		float twist_influence = 1;
	};

private:
	BoneId root_bone = -1;
	BoneId first_bone = -1;
	BoneId leaf_bone = -1;

	Vector<Joint> joints;
	float total_length = 0;
	Transform rest_leaf;
	Vector3 root_bone_direction;
	void init_chain(Skeleton *skeleton);

public:
	void set_root_bone(Skeleton *skeleton, BoneId p_root_bone);
	void set_leaf_bone(Skeleton *skeleton, BoneId p_leaf_bone);
	bool is_valid();
	float twist_influence = 1; //How much the chain tries to twist to follow the end when the start is facing a different direction
	float twist_start = 0; //Where along the chain the twisting starts
	Vector3 chain_curve_direction; //This defines which way to prebend it
	float root_influence = 0; //how much the start bone is influenced by the root rotation
	float leaf_influence = 0; //how much the end bone is influenced by the goal rotation
	float get_total_length();
	Vector<RenIKChain::Joint> get_joints();
	Transform get_relative_rest_leaf();
	Vector3 get_root_bone_direction();
	BoneId get_first_bone();
	BoneId get_root_bone();
	BoneId get_leaf_bone();
};

#endif