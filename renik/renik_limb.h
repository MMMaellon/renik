#ifndef RENIK_LIMB_H
#define RENIK_LIMB_H

#include <scene/3d/skeleton.h>

struct RenIKLimb : public Resource {
	GDCLASS(RenIKLimb, Resource);

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

	float upper_twist_offset = 0;
	float lower_twist_offset = 0;
	float roll_offset = 0; //Rolls the entire limb so the joint points in a different direction
	float upper_limb_twist = 0; //How much the upper limb follows the lower limb
	float lower_limb_twist = 0; //How much the lower limb follows the leaf limb
	float twist_inflection_point_offset = 0; //When the limb snaps from twisting in the positive direction to twisting in the negative direction
	float twist_overflow = 0; //How much past the inflection point we go before snapping

	Quat pole_offset; /*ADVANCED - Moving the limb 180 degrees from rest tends to be a bit unpredictable
		as there is a pole in the forward vector sphere at that spot.
		This offsets the rest position so that the pole is in a place where the limb is unlikely to go*/

	Vector3 target_position_influence; //ADVANCED - How much each of the leaf's axis of translation from rest affects the ik
	float target_rotation_influence; //ADVANCED - How much the rotation the leaf points in affects the ik

	//STATE: We're keeping a little bit of state now... kinda goes against the design, but it makes life easier so fuck it
	int overflow_state = 0;// 0 means no twist overflow. -1 means underflow. 1 means overflow.

	void init(float p_upper_twist_offset, float p_lower_twist_offset, float p_roll_offset, float p_upper_limb_twist, float p_lower_limb_twist, float p_twist_inflection_point_offset, float p_twist_overflow, float p_target_rotation_influence, Vector3 p_pole_offset, Vector3 p_target_position_influence);
};

#endif
