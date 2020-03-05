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
	float twist_influence; //How much the chain tries to twist to follow the end when the start is facing a different direction
	float twist_start; //Where along the chain the twisting starts
	float spineCurve; //To prevent chains from bending the wrong way, we precurve it before solving the IK. This controls how much to precurve it
	float spineCurveAngle; //This defines which way to precurve it
};

#endif