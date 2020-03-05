#ifndef RENIK_LIMB_H
#define RENIK_LIMB_H

#include <scene/3d/skeleton.h>

struct RenIKLimb : public Reference {
	GDCLASS(RenIKLimb, Reference);

	Transform upper;
	Transform lower;
	Transform leaf;
	BoneId leaf_id = -1;
	BoneId lower_id = -1;
	BoneId upper_id = -1;

public:
	void set_leaf(Skeleton *skeleton, BoneId p_leaf_id);
	BoneId get_leaf_bone();
	BoneId get_lower_bone();
	BoneId get_upper_bone();
	bool is_valid();
	Transform get_upper();
	Transform get_lower();
	Transform get_leaf();

	float angle_offset; //Defines which way is bending forward and which way is backward
	float rest_roll_offset; //Rolls the entire limb so it bends a certain direction at rest
	float upper_limb_twist; //How much the upper limb follows the lower limb
	float lower_limb_twist; //How much the lower limb follows the leaf limb

	Quat pole_offset; /*ADVANCED - Moving the limb 180 degrees from rest tends to be a bit unpredictable
		as there is a pole in the forward vector sphere at that spot.
		This offsets the rest position so that the pole is in a place where the limb is unlikely to go*/

	Vector3 target_position_influence; //ADVANCED - How much each of the leaf's axis of translation from rest affects the ik
	float target_direction_influence; //ADVANCED - How much the direction the leaf points in affects the ik
};

#endif