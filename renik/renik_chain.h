#ifndef RENIK_CHAIN_H
#define RENIK_CHAIN_H

#include <scene/3d/skeleton.h>

struct RenIKChain : public Reference {
	GDCLASS(RenIKChain, Reference);
	Vector<Transform> bones;
	BoneId start_bone = -1;
	BoneId end_bone = -1;

public:
	void set_chain(Skeleton *skeleton, BoneId p_start_bone, BoneId p_end_bone);
	BoneId get_start_bone();
	BoneId get_end_bone();
	bool is_valid();
	float twist_influence = 0; //How much the chain tries to twist to follow the end when the start is facing a different direction
	float twist_start = 0; //Where along the chain the twisting starts
	float chain_curve = 0; //To prevent chains from bending the wrong way, we precurve it before solving the IK. This controls how much to precurve it
	float chain_curve_angle = 0; //This defines which way to precurve it
	float start_influence = 0; //how much the start bone is influenced by the root rotation
	float end_influence = 0; //how much the end bone is influenced by the goal rotation
};

#endif