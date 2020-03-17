#ifndef RENIK_LIMB_H
#define RENIK_LIMB_H

#include <scene/3d/skeleton.h>

struct RenIKLimb : public Reference {
	GDCLASS(RenIKLimb, Reference);

public:
	Transform upper;
	Transform lower;
	Transform leaf;
	BoneId leaf_id = -1;
	BoneId lower_id = -1;
	BoneId upper_id = -1;

	void set_leaf(Skeleton *skeleton, BoneId p_leaf_id);
	BoneId get_leaf_bone();
	BoneId get_lower_bone();
	BoneId get_upper_bone();
	bool is_valid();
	Transform get_upper();
	Transform get_lower();
	Transform get_leaf();

	float twist;
	float upper_twist_offset;
	float lower_twist_offset;
	float roll_offset; //Rolls the entire limb so the joint points in a different direction
	float upper_limb_twist; //How much the upper limb follows the lower limb
	float lower_limb_twist; //How much the lower limb follows the leaf limb
	float twist_inflection_point_offset; //When the limb snaps from twisting in the positive direction to twisting in the negative direction

	Quat pole_offset; /*ADVANCED - Moving the limb 180 degrees from rest tends to be a bit unpredictable
		as there is a pole in the forward vector sphere at that spot.
		This offsets the rest position so that the pole is in a place where the limb is unlikely to go*/

	Vector3 target_position_influence; //ADVANCED - How much each of the leaf's axis of translation from rest affects the ik
	float target_rotation_influence; //ADVANCED - How much the rotation the leaf points in affects the ik

	RenIKLimb() :
			twist(0),
			upper_twist_offset(0),
			lower_twist_offset(0),
			roll_offset(0),
			upper_limb_twist(0),
			lower_limb_twist(0),
			twist_inflection_point_offset(0),
			target_rotation_influence(0) {}
	RenIKLimb(float p_twist, float p_upper_twist_offset, float p_lower_twist_offset, float p_roll_offset, float p_upper_limb_twist, float p_lower_limb_twist, float p_twist_inflection_point_offset, float p_target_rotation_influence, Vector3 p_pole_offset, Vector3 p_target_position_influence) :
			twist(p_twist),
			upper_twist_offset(p_upper_twist_offset),
			lower_twist_offset(p_lower_twist_offset),
			roll_offset(p_roll_offset),
			upper_limb_twist(p_upper_limb_twist),
			lower_limb_twist(p_lower_limb_twist),
			twist_inflection_point_offset(p_twist_inflection_point_offset),
			target_rotation_influence(p_target_rotation_influence),
			pole_offset(p_pole_offset),
			target_position_influence(p_target_position_influence) {}
};

#endif