#include "renik.h"
#ifndef _3D_DISABLED

const float DEFAULT_THRESHOLD = 0.0005;
const int DEFAULT_LOOP_LIMIT = 16;

#define RENIK_PROPERTY_STRING_SKELETON_PATH "armature_skeleton_path"

#define RENIK_PROPERTY_STRING_HEAD_BONE "armature_head"
#define RENIK_PROPERTY_STRING_HAND_LEFT_BONE "armature_left_hand"
#define RENIK_PROPERTY_STRING_HAND_RIGHT_BONE "armature_right_hand"
#define RENIK_PROPERTY_STRING_HIP_BONE "armature_hip"
#define RENIK_PROPERTY_STRING_FOOT_LEFT_BONE "armature_left_foot"
#define RENIK_PROPERTY_STRING_FOOT_RIGHT_BONE "armature_right_foot"

#define RENIK_PROPERTY_STRING_HEAD_TARGET_PATH "armature_head_target"
#define RENIK_PROPERTY_STRING_HAND_LEFT_TARGET_PATH "armature_left_hand_target"
#define RENIK_PROPERTY_STRING_HAND_RIGHT_TARGET_PATH "armature_right_hand_target"
#define RENIK_PROPERTY_STRING_HIP_TARGET_PATH "armature_hip_target"
#define RENIK_PROPERTY_STRING_FOOT_LEFT_TARGET_PATH "armature_left_foot_target"
#define RENIK_PROPERTY_STRING_FOOT_RIGHT_TARGET_PATH "armature_right_foot_target"

RenIK::RenIK() :
		//IK DEFAULTS
		left_shoulder_offset(Math::deg2rad(-20.0), Math::deg2rad(-10.0), Math::deg2rad(-10.0)),
		right_shoulder_offset(Math::deg2rad(-20.0), Math::deg2rad(10.0), Math::deg2rad(10.0)),
		left_shoulder_pole_offset(Math::deg2rad(78.0), Math::deg2rad(0.0), Math::deg2rad(0.0)),
		right_shoulder_pole_offset(Math::deg2rad(78.0), Math::deg2rad(0.0), Math::deg2rad(0.0)){
	spine_chain.instance();
	spine_chain->init(Vector3(0, 15, -15), 1, 1, 1, 0);
	limb_arm_left.instance();
	limb_arm_left->init(0, 0, Math_PI, 0.5, 0.66666, Math::deg2rad(20.0), Math::deg2rad(45.0), 0.25, Vector3(Math::deg2rad(60.0), 0, 0), Vector3(2.0, -1, -2.0));
	limb_arm_right.instance();
	limb_arm_right->init(0, 0, -Math_PI, 0.5, 0.66666, Math::deg2rad(-20.0), Math::deg2rad(45.0), 0.25, Vector3(Math::deg2rad(60.0), 0, 0), Vector3(2.0, 1, 2.0));
	limb_leg_left.instance();
	limb_leg_left->init(0, 0, 0, 0.25, 0.25, 0, Math::deg2rad(45.0), 0.5, Vector3(0, 0, Math_PI), Vector3());
	limb_leg_right.instance();
	limb_leg_right->init(0, 0, 0, 0.25, 0.25, 0, Math::deg2rad(45.0), 0.5, Vector3(0, 0, -Math_PI), Vector3());
};

void RenIK::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_live_preview", "p_enable"), &RenIK::set_live_preview);
	ClassDB::bind_method(D_METHOD("get_live_preview"), &RenIK::get_live_preview);

	ClassDB::bind_method(D_METHOD("enable_solve_ik_every_frame", "p_enable"), &RenIK::enable_solve_ik_every_frame);
	ClassDB::bind_method(D_METHOD("enable_hip_placement", "p_enable"), &RenIK::enable_hip_placement);
	ClassDB::bind_method(D_METHOD("enable_foot_placement", "p_enable"), &RenIK::enable_foot_placement);

	ClassDB::bind_method(D_METHOD("set_skeleton_path", "p_path"), &RenIK::set_skeleton_path);
	ClassDB::bind_method(D_METHOD("set_skeleton_", "p_node"), &RenIK::set_skeleton);
	ClassDB::bind_method(D_METHOD("get_skeleton_path"), &RenIK::get_skeleton_path);

	ClassDB::bind_method(D_METHOD("set_head_bone_by_name", "p_bone"), &RenIK::set_head_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_head_bone_name"), &RenIK::get_head_bone_name);
	ClassDB::bind_method(D_METHOD("set_hand_left_bone_by_name", "p_bone"), &RenIK::set_hand_left_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_hand_left_bone_name"), &RenIK::get_hand_left_bone_name);
	ClassDB::bind_method(D_METHOD("set_hand_right_bone_by_name", "p_bone"), &RenIK::set_hand_right_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_hand_right_bone_name"), &RenIK::get_hand_right_bone_name);
	ClassDB::bind_method(D_METHOD("set_hip_bone_by_name", "p_bone"), &RenIK::set_hip_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_hip_bone_name"), &RenIK::get_hip_bone_name);
	ClassDB::bind_method(D_METHOD("set_foot_left_bone_by_name", "p_bone"), &RenIK::set_foot_left_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_foot_left_bone_name"), &RenIK::get_foot_left_bone_name);
	ClassDB::bind_method(D_METHOD("set_foot_right_bone_by_name", "p_bone"), &RenIK::set_foot_right_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_foot_right_bone_name"), &RenIK::get_foot_right_bone_name);

	ClassDB::bind_method(D_METHOD("set_head_bone", "p_bone"), &RenIK::set_head_bone);
	ClassDB::bind_method(D_METHOD("get_head_bone"), &RenIK::get_head_bone);
	ClassDB::bind_method(D_METHOD("set_hand_left_bone", "p_bone"), &RenIK::set_hand_left_bone);
	ClassDB::bind_method(D_METHOD("get_hand_left_bone"), &RenIK::get_hand_left_bone);
	ClassDB::bind_method(D_METHOD("set_hand_right_bone", "p_bone"), &RenIK::set_hand_right_bone);
	ClassDB::bind_method(D_METHOD("get_hand_right_bone"), &RenIK::get_hand_right_bone);
	ClassDB::bind_method(D_METHOD("set_hip_bone", "p_bone"), &RenIK::set_hip_bone);
	ClassDB::bind_method(D_METHOD("get_hip_bone"), &RenIK::get_hip_bone);
	ClassDB::bind_method(D_METHOD("set_foot_left_bone", "p_bone"), &RenIK::set_foot_left_bone);
	ClassDB::bind_method(D_METHOD("get_foot_left_bone"), &RenIK::get_foot_left_bone);
	ClassDB::bind_method(D_METHOD("set_foot_right_bone", "p_bone"), &RenIK::set_foot_right_bone);
	ClassDB::bind_method(D_METHOD("get_foot_right_bone"), &RenIK::get_foot_right_bone);

	ClassDB::bind_method(D_METHOD("set_head_target_path", "p_path"), &RenIK::set_head_target_path);
	ClassDB::bind_method(D_METHOD("get_head_target_path"), &RenIK::get_head_target_path);
	ClassDB::bind_method(D_METHOD("set_hand_left_target_path", "p_path"), &RenIK::set_hand_left_target_path);
	ClassDB::bind_method(D_METHOD("get_hand_left_target_path"), &RenIK::get_hand_left_target_path);
	ClassDB::bind_method(D_METHOD("set_hand_right_target_path", "p_path"), &RenIK::set_hand_right_target_path);
	ClassDB::bind_method(D_METHOD("get_hand_right_target_path"), &RenIK::get_hand_right_target_path);
	ClassDB::bind_method(D_METHOD("set_hip_target_path", "p_path"), &RenIK::set_hip_target_path);
	ClassDB::bind_method(D_METHOD("get_hip_target_path"), &RenIK::get_hip_target_path);
	ClassDB::bind_method(D_METHOD("set_foot_left_target_path", "p_path"), &RenIK::set_foot_left_target_path);
	ClassDB::bind_method(D_METHOD("get_foot_left_target_path"), &RenIK::get_foot_left_target_path);
	ClassDB::bind_method(D_METHOD("set_foot_right_target_path", "p_path"), &RenIK::set_foot_right_target_path);
	ClassDB::bind_method(D_METHOD("get_foot_right_target_path"), &RenIK::get_foot_right_target_path);

	ClassDB::bind_method(D_METHOD("get_arm_upper_twist_offset"), &RenIK::get_arm_upper_twist_offset);
	ClassDB::bind_method(D_METHOD("set_arm_upper_twist_offset", "degrees"), &RenIK::set_arm_upper_twist_offset);
	ClassDB::bind_method(D_METHOD("get_arm_lower_twist_offset"), &RenIK::get_arm_lower_twist_offset);
	ClassDB::bind_method(D_METHOD("set_arm_lower_twist_offset", "degrees"), &RenIK::set_arm_lower_twist_offset);
	ClassDB::bind_method(D_METHOD("get_arm_roll_offset"), &RenIK::get_arm_roll_offset);
	ClassDB::bind_method(D_METHOD("set_arm_roll_offset", "degrees"), &RenIK::set_arm_roll_offset);
	ClassDB::bind_method(D_METHOD("get_arm_upper_limb_twist"), &RenIK::get_arm_upper_limb_twist);
	ClassDB::bind_method(D_METHOD("set_arm_upper_limb_twist", "ratio"), &RenIK::set_arm_upper_limb_twist);
	ClassDB::bind_method(D_METHOD("get_arm_lower_limb_twist"), &RenIK::get_arm_lower_limb_twist);
	ClassDB::bind_method(D_METHOD("set_arm_lower_limb_twist", "ratio"), &RenIK::set_arm_lower_limb_twist);
	ClassDB::bind_method(D_METHOD("get_arm_twist_inflection_point_offset"), &RenIK::get_arm_twist_inflection_point_offset);
	ClassDB::bind_method(D_METHOD("set_arm_twist_inflection_point_offset", "degrees"), &RenIK::set_arm_twist_inflection_point_offset);
	ClassDB::bind_method(D_METHOD("get_arm_twist_overflow"), &RenIK::get_arm_twist_overflow);
	ClassDB::bind_method(D_METHOD("set_arm_twist_overflow", "degrees"), &RenIK::set_arm_twist_overflow);

	ClassDB::bind_method(D_METHOD("get_arm_pole_offset"), &RenIK::get_arm_pole_offset);
	ClassDB::bind_method(D_METHOD("set_arm_pole_offset", "euler"), &RenIK::set_arm_pole_offset);
	ClassDB::bind_method(D_METHOD("get_arm_target_position_influence"), &RenIK::get_arm_target_position_influence);
	ClassDB::bind_method(D_METHOD("set_arm_target_position_influence", "xyz"), &RenIK::set_arm_target_position_influence);
	ClassDB::bind_method(D_METHOD("get_arm_target_rotation_influence"), &RenIK::get_arm_target_rotation_influence);
	ClassDB::bind_method(D_METHOD("set_arm_target_rotation_influence", "influence"), &RenIK::set_arm_target_rotation_influence);

	ClassDB::bind_method(D_METHOD("get_leg_upper_twist_offset"), &RenIK::get_leg_upper_twist_offset);
	ClassDB::bind_method(D_METHOD("set_leg_upper_twist_offset", "degrees"), &RenIK::set_leg_upper_twist_offset);
	ClassDB::bind_method(D_METHOD("get_leg_lower_twist_offset"), &RenIK::get_leg_lower_twist_offset);
	ClassDB::bind_method(D_METHOD("set_leg_lower_twist_offset", "degrees"), &RenIK::set_leg_lower_twist_offset);
	ClassDB::bind_method(D_METHOD("get_leg_roll_offset"), &RenIK::get_leg_roll_offset);
	ClassDB::bind_method(D_METHOD("set_leg_roll_offset", "degrees"), &RenIK::set_leg_roll_offset);
	ClassDB::bind_method(D_METHOD("get_leg_upper_limb_twist"), &RenIK::get_leg_upper_limb_twist);
	ClassDB::bind_method(D_METHOD("set_leg_upper_limb_twist", "ratio"), &RenIK::set_leg_upper_limb_twist);
	ClassDB::bind_method(D_METHOD("get_leg_lower_limb_twist"), &RenIK::get_leg_lower_limb_twist);
	ClassDB::bind_method(D_METHOD("set_leg_lower_limb_twist", "ratio"), &RenIK::set_leg_lower_limb_twist);
	ClassDB::bind_method(D_METHOD("get_leg_twist_inflection_point_offset"), &RenIK::get_leg_twist_inflection_point_offset);
	ClassDB::bind_method(D_METHOD("set_leg_twist_inflection_point_offset", "degrees"), &RenIK::set_leg_twist_inflection_point_offset);
	ClassDB::bind_method(D_METHOD("get_leg_twist_overflow"), &RenIK::get_leg_twist_overflow);
	ClassDB::bind_method(D_METHOD("set_leg_twist_overflow", "degrees"), &RenIK::set_leg_twist_overflow);

	ClassDB::bind_method(D_METHOD("get_leg_pole_offset"), &RenIK::get_leg_pole_offset);
	ClassDB::bind_method(D_METHOD("set_leg_pole_offset", "euler"), &RenIK::set_leg_pole_offset);
	ClassDB::bind_method(D_METHOD("get_leg_target_position_influence"), &RenIK::get_leg_target_position_influence);
	ClassDB::bind_method(D_METHOD("set_leg_target_position_influence", "xyz"), &RenIK::set_leg_target_position_influence);
	ClassDB::bind_method(D_METHOD("get_leg_target_rotation_influence"), &RenIK::get_leg_target_rotation_influence);
	ClassDB::bind_method(D_METHOD("set_leg_target_rotation_influence", "influence"), &RenIK::set_leg_target_rotation_influence);

	ClassDB::bind_method(D_METHOD("get_spine_curve"), &RenIK::get_spine_curve);
	ClassDB::bind_method(D_METHOD("set_spine_curve", "direction"), &RenIK::set_spine_curve);
	ClassDB::bind_method(D_METHOD("get_upper_spine_stiffness"), &RenIK::get_upper_spine_stiffness);
	ClassDB::bind_method(D_METHOD("set_upper_spine_stiffness", "influence"), &RenIK::set_upper_spine_stiffness);
	ClassDB::bind_method(D_METHOD("get_lower_spine_stiffness"), &RenIK::get_lower_spine_stiffness);
	ClassDB::bind_method(D_METHOD("set_lower_spine_stiffness", "influence"), &RenIK::set_lower_spine_stiffness);
	ClassDB::bind_method(D_METHOD("get_spine_twist"), &RenIK::get_spine_twist);
	ClassDB::bind_method(D_METHOD("set_spine_twist", "influence"), &RenIK::set_spine_twist);
	ClassDB::bind_method(D_METHOD("get_spine_twist_start"), &RenIK::get_spine_twist_start);
	ClassDB::bind_method(D_METHOD("set_spine_twist_start", "influence"), &RenIK::set_spine_twist_start);
	ClassDB::bind_method(D_METHOD("get_shoulder_influence"), &RenIK::get_shoulder_influence);
	ClassDB::bind_method(D_METHOD("set_shoulder_influence", "influence"), &RenIK::set_shoulder_influence);
	ClassDB::bind_method(D_METHOD("get_shoulder_offset"), &RenIK::get_shoulder_offset);
	ClassDB::bind_method(D_METHOD("set_shoulder_offset", "euler"), &RenIK::set_shoulder_offset);
	ClassDB::bind_method(D_METHOD("get_shoulder_pole_offset"), &RenIK::get_shoulder_pole_offset);
	ClassDB::bind_method(D_METHOD("set_shoulder_pole_offset", "euler"), &RenIK::set_shoulder_pole_offset);

	ClassDB::bind_method(D_METHOD("set_collide_with_areas", "enable"), &RenIK::set_collide_with_areas);
	ClassDB::bind_method(D_METHOD("is_collide_with_areas_enabled"), &RenIK::is_collide_with_areas_enabled);

	ClassDB::bind_method(D_METHOD("set_collide_with_bodies", "enable"), &RenIK::set_collide_with_bodies);
	ClassDB::bind_method(D_METHOD("is_collide_with_bodies_enabled"), &RenIK::is_collide_with_bodies_enabled);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &RenIK::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &RenIK::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_collision_mask_bit", "bit", "value"), &RenIK::set_collision_mask_bit);
	ClassDB::bind_method(D_METHOD("get_collision_mask_bit", "bit"), &RenIK::get_collision_mask_bit);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "live_preview"), "set_live_preview", "get_live_preview");

	ADD_GROUP("Armature", "armature_");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, RENIK_PROPERTY_STRING_SKELETON_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Skeleton", PROPERTY_USAGE_DEFAULT), "set_skeleton_path", "get_skeleton_path");

	ADD_PROPERTY(PropertyInfo(Variant::STRING, RENIK_PROPERTY_STRING_HEAD_BONE), "set_head_bone_by_name", "get_head_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, RENIK_PROPERTY_STRING_HAND_LEFT_BONE), "set_hand_left_bone_by_name", "get_hand_left_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, RENIK_PROPERTY_STRING_HAND_RIGHT_BONE), "set_hand_right_bone_by_name", "get_hand_right_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, RENIK_PROPERTY_STRING_HIP_BONE), "set_hip_bone_by_name", "get_hip_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, RENIK_PROPERTY_STRING_FOOT_LEFT_BONE), "set_foot_left_bone_by_name", "get_foot_left_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, RENIK_PROPERTY_STRING_FOOT_RIGHT_BONE), "set_foot_right_bone_by_name", "get_foot_right_bone_name");

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, RENIK_PROPERTY_STRING_HEAD_TARGET_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"), "set_head_target_path", "get_head_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, RENIK_PROPERTY_STRING_HAND_LEFT_TARGET_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"), "set_hand_left_target_path", "get_hand_left_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, RENIK_PROPERTY_STRING_HAND_RIGHT_TARGET_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"), "set_hand_right_target_path", "get_hand_right_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, RENIK_PROPERTY_STRING_HIP_TARGET_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"), "set_hip_target_path", "get_hip_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, RENIK_PROPERTY_STRING_FOOT_LEFT_TARGET_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"), "set_foot_left_target_path", "get_foot_left_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, RENIK_PROPERTY_STRING_FOOT_RIGHT_TARGET_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"), "set_foot_right_target_path", "get_foot_right_target_path");

	ADD_GROUP("Arm IK Settings", "arm_");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_elbow_direction_offset", PROPERTY_HINT_RANGE, "-360,360,0.1"), "set_arm_roll_offset", "get_arm_roll_offset");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_upper_arm_twisting", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_arm_upper_limb_twist", "get_arm_upper_limb_twist");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_upper_arm_twist_offset", PROPERTY_HINT_RANGE, "-360,360,0.1"), "set_arm_upper_twist_offset", "get_arm_upper_twist_offset");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_forearm_twisting", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_arm_lower_limb_twist", "get_arm_lower_limb_twist");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_forearm_twist_offset", PROPERTY_HINT_RANGE, "-360,360,0.1"), "set_arm_lower_twist_offset", "get_arm_lower_twist_offset");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_twist_inflection_point", PROPERTY_HINT_RANGE, "-180,180,0.1"), "set_arm_twist_inflection_point_offset", "get_arm_twist_inflection_point_offset");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_twist_overflow", PROPERTY_HINT_RANGE, "0,180,0.1"), "set_arm_twist_overflow", "get_arm_twist_overflow");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_shoulder_influence", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_shoulder_influence", "get_shoulder_influence");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "arm_shoulder_offset"), "set_shoulder_offset", "get_shoulder_offset");

	ADD_GROUP("Arm IK Settings (Advanced)", "arm_");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "arm_pole_offset"), "set_arm_pole_offset", "get_arm_pole_offset");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "arm_target_position_influence"), "set_arm_target_position_influence", "get_arm_target_position_influence");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_target_rotation_influence", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_arm_target_rotation_influence", "get_arm_target_rotation_influence");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "arm_shoulder_pole_offset"), "set_shoulder_pole_offset", "get_shoulder_pole_offset");

	ADD_GROUP("Leg IK Settings", "leg_");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "leg_knee_direction_offset", PROPERTY_HINT_RANGE, "-360,360,0.1"), "set_leg_roll_offset", "get_leg_roll_offset");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "leg_thigh_twisting", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_leg_upper_limb_twist", "get_leg_upper_limb_twist");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "leg_thigh_twist_offset", PROPERTY_HINT_RANGE, "-360,360,0.1"), "set_leg_upper_twist_offset", "get_leg_upper_twist_offset");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "leg_lower_leg_twisting", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_leg_lower_limb_twist", "get_leg_lower_limb_twist");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "leg_lower_leg_twist_offset", PROPERTY_HINT_RANGE, "-360,360,0.1"), "set_leg_lower_twist_offset", "get_leg_lower_twist_offset");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "leg_twist_inflection_point", PROPERTY_HINT_RANGE, "-180,180,0.1"), "set_leg_twist_inflection_point_offset", "get_leg_twist_inflection_point_offset");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "leg_twist_overflow", PROPERTY_HINT_RANGE, "0,180,0.1"), "set_leg_twist_overflow", "get_leg_twist_overflow");

	ADD_GROUP("Leg IK Settings (Advanced)", "leg_");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "leg_pole_offset"), "set_leg_pole_offset", "get_leg_pole_offset");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "leg_target_position_influence"), "set_leg_target_position_influence", "get_leg_target_position_influence");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "leg_target_rotation_influence", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_leg_target_rotation_influence", "get_leg_target_rotation_influence");

	ADD_GROUP("Torso IK Settings", "torso_");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "torso_spine_curve"), "set_spine_curve", "get_spine_curve");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "torso_upper_spine_stiffness", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_upper_spine_stiffness", "get_upper_spine_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "torso_lower_spine_stiffness", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_lower_spine_stiffness", "get_lower_spine_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "torso_spine_twist", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_spine_twist", "get_spine_twist");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "torso_spine_twist_start", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_spine_twist_start", "get_spine_twist_start");

	ADD_GROUP("Walk Settings (Advanced)", "walk_");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "walk_collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "walk_collide_with_areas", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collide_with_areas", "is_collide_with_areas_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "walk_collide_with_bodies", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collide_with_bodies", "is_collide_with_bodies_enabled");

	ClassDB::bind_method(D_METHOD("update_ik"), &RenIK::update_ik);
	ClassDB::bind_method(D_METHOD("update_placement"), &RenIK::update_placement);
}

void RenIK::_validate_property(PropertyInfo &property) const {
	if (property.name == RENIK_PROPERTY_STRING_HEAD_BONE || property.name == RENIK_PROPERTY_STRING_HIP_BONE || property.name == RENIK_PROPERTY_STRING_HAND_LEFT_BONE || property.name == RENIK_PROPERTY_STRING_HAND_RIGHT_BONE || property.name == RENIK_PROPERTY_STRING_FOOT_LEFT_BONE || property.name == RENIK_PROPERTY_STRING_FOOT_RIGHT_BONE) {
		if (skeleton) {
			String names(",");
			for (int i = 0; i < skeleton->get_bone_count(); i++) {
				if (i > 0)
					names += ",";
				names += skeleton->get_bone_name(i);
			}
			property.hint = PROPERTY_HINT_ENUM;
			property.hint_string = names;
		} else {
			property.hint = PROPERTY_HINT_NONE;
			property.hint_string = "";
		}
	}
}

void RenIK::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY:
			_initialize();
			break;
		case NOTIFICATION_INTERNAL_PROCESS:
			if (!Engine::get_singleton()->is_editor_hint() || live_preview) {
				update_ik();
			}
			break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS:
			if (!Engine::get_singleton()->is_editor_hint() || live_preview) {
				update_placement(get_physics_process_delta_time());
			}
			break;
	}
}

void RenIK::_initialize() {
	//set the skeleton to the parent if we can
	Node *parent = get_parent();
	if (skeleton_path.is_empty() && parent) {
		set_skeleton(parent);
	} else {
		set_skeleton_path(get_skeleton_path());
	}
	set_head_target_path(get_head_target_path());
	set_hip_target_path(get_hip_target_path());
	set_hand_left_target_path(get_hand_left_target_path());
	set_hand_right_target_path(get_hand_right_target_path());
	set_foot_left_target_path(get_foot_left_target_path());
	set_foot_right_target_path(get_foot_right_target_path());

	if(Engine::get_singleton()->is_editor_hint()) {
		set_process_internal(true);
		set_physics_process_internal(true);
	}
	// set_process_priority(1); //makes sure that ik is done last after all physics and movement have taken place
	// enable_solve_ik_every_frame(true);
	// enable_hip_placement(true);
	// enable_foot_placement(true);
	// set_physics_process_internal(true);
}

void RenIK::enable_solve_ik_every_frame(bool automatically_update_ik) {
	set_process_internal(automatically_update_ik);
}

void RenIK::enable_hip_placement(bool enabled) {
	hip_placement = enabled;
}

void RenIK::enable_foot_placement(bool enabled) {
	foot_placement = enabled;
}

void RenIK::update_ik() {
	// Saracen: since the foot placement is updated in the physics frame,
	// interpolate the results to avoid jitter
	placement.interpolate_transforms(
		Engine::get_singleton()->get_physics_interpolation_fraction(),
		!hip_target_spatial,
		foot_placement);

	perform_torso_ik();
	perform_hand_left_ik();
	perform_hand_right_ik();
	perform_foot_left_ik();
	perform_foot_right_ik();
}

void RenIK::update_placement(float delta) {
	// Saracen: save the transforms from the last update for use with interpolation
	placement.save_previous_transforms();

	//Based on head position and delta time, we calc our speed and distance from the ground and place the feet accordingly
	if (foot_placement && head_target_spatial && head_target_spatial->is_inside_world()) {
		placement.foot_place(delta, head_target_spatial->get_global_transform(), head_target_spatial->get_world(), false);
	}
	if (hip_placement && head_target_spatial) {
		//calc twist from hands here
		float twist = 0;
		if (foot_placement) {
			placement.hip_place(delta, head_target_spatial->get_global_transform(), placement.target_left_foot, placement.target_right_foot, twist, false);
		} else {
			placement.hip_place(delta, head_target_spatial->get_global_transform(), skeleton->get_global_transform() * skeleton->get_bone_global_pose(get_foot_left_bone()), skeleton->get_global_transform() * skeleton->get_bone_global_pose(get_foot_right_bone()), twist, false);
		}
	}
}

void RenIK::apply_ik_map(Map<BoneId, Quat> ikMap) {
	if (skeleton) {
		Map<BoneId, Quat>::Element *elem = ikMap.front();
		while (elem) {
			skeleton->set_bone_custom_pose(elem->key(), Transform(elem->value()));
			elem = elem->next();
		}
	}
}

void RenIK::apply_ik_map(Map<BoneId, Basis> ikMap) {
	if (skeleton) {
		Map<BoneId, Basis>::Element *elem = ikMap.front();
		while (elem) {
			skeleton->set_bone_custom_pose(elem->key(), Transform(elem->value()));
			elem = elem->next();
		}
	}
}

void RenIK::perform_torso_ik() {
	if (head_target_spatial && skeleton && spine_chain->is_valid()) {
		Transform headGlobalTransform = head_target_spatial->get_global_transform();
		Transform hipGlobalTransform = hip_target_spatial ? hip_target_spatial->get_global_transform() : placement.interpolated_hip;
		Vector3 delta = hipGlobalTransform.origin + hipGlobalTransform.basis.xform(spine_chain->get_joints()[0].relative_prev) - headGlobalTransform.origin;
		float fullLength = spine_chain->get_total_length();
		if (delta.length() > fullLength) {
			hipGlobalTransform.set_origin(headGlobalTransform.origin + (delta.normalized() * fullLength) - hipGlobalTransform.basis.xform(spine_chain->get_joints()[0].relative_prev));
		}

		int hipParent = skeleton->get_bone_parent(spine_chain->get_root_bone());
		if (hipParent != -1) {
			skeleton->set_bone_custom_pose(spine_chain->get_root_bone(),
				(skeleton->get_bone_global_pose(hipParent)
					* skeleton->get_bone_rest(spine_chain->get_root_bone())
					* skeleton->get_bone_pose(spine_chain->get_root_bone())).affine_inverse()
				* hipGlobalTransform);
		} else {
			skeleton->set_bone_custom_pose(spine_chain->get_root_bone(),
				(skeleton->get_bone_rest(spine_chain->get_root_bone())
					* skeleton->get_bone_pose(spine_chain->get_root_bone())).affine_inverse()
				* hipGlobalTransform);
		}

		apply_ik_map(solve_ifabrik(spine_chain, hipGlobalTransform, headGlobalTransform, DEFAULT_THRESHOLD, DEFAULT_LOOP_LIMIT));

		int headParent = skeleton->get_bone_parent(spine_chain->get_leaf_bone());

		skeleton->set_bone_custom_pose(spine_chain->get_leaf_bone(),
			(skeleton->get_bone_global_pose(headParent) * skeleton->get_bone_rest(spine_chain->get_leaf_bone()) * skeleton->get_bone_pose(spine_chain->get_leaf_bone())).affine_inverse()
			* headGlobalTransform);
	}
}

void RenIK::perform_hand_left_ik() {
	if (hand_left_target_spatial && skeleton && limb_arm_left->is_valid()) {
		Transform root = skeleton->get_global_transform();
		BoneId rootBone = skeleton->get_bone_parent(limb_arm_left->get_upper_bone());
		if (rootBone >= 0) {
			if (left_shoulder_enabled) {
				BoneId shoulderParent = skeleton->get_bone_parent(rootBone);
				if (shoulderParent >= 0) {
					root = root * skeleton->get_bone_global_pose(shoulderParent);
				}
				root = root * skeleton->get_bone_rest(rootBone);
				Vector3 targetVector = root.affine_inverse().xform(hand_left_target_spatial->get_global_transform().origin);
				Quat offsetQuat = Quat(left_shoulder_offset);
				Quat poleOffset = Quat(left_shoulder_pole_offset);
				Quat poleOffsetScaled = poleOffset.slerp(Quat(), 1 - shoulder_influence);
				Quat quatAlignToTarget = poleOffsetScaled * RenIKHelper::align_vectors(Vector3(0, 1, 0), poleOffset.inverse().xform(offsetQuat.inverse().xform(targetVector))).slerp(Quat(), 1 - shoulder_influence);
				Transform customPose = Transform(offsetQuat * quatAlignToTarget, Vector3());
				skeleton->set_bone_custom_pose(rootBone, customPose);
			}
			root = skeleton->get_global_transform() * skeleton->get_bone_global_pose(rootBone);
		}
		apply_ik_map(solve_trig_ik_redux(limb_arm_left, root, hand_left_target_spatial->get_global_transform()));
	}
}

void RenIK::perform_hand_right_ik() {
	if (hand_right_target_spatial && skeleton && limb_arm_right->is_valid()) {
		Transform root = skeleton->get_global_transform();
		BoneId rootBone = skeleton->get_bone_parent(limb_arm_right->get_upper_bone());
		if (rootBone >= 0) {
			if (right_shoulder_enabled) {
				BoneId shoulderParent = skeleton->get_bone_parent(rootBone);
				if (shoulderParent >= 0) {
					root = root * skeleton->get_bone_global_pose(shoulderParent);
				}
				root = root * skeleton->get_bone_rest(rootBone);
				Vector3 targetVector = root.affine_inverse().xform(hand_right_target_spatial->get_global_transform().origin);
				Quat offsetQuat = Quat(right_shoulder_offset);
				Quat poleOffset = Quat(right_shoulder_pole_offset);
				Quat poleOffsetScaled = poleOffset.slerp(Quat(), 1 - shoulder_influence);
				Quat quatAlignToTarget = poleOffsetScaled * RenIKHelper::align_vectors(Vector3(0, 1, 0), poleOffset.inverse().xform(offsetQuat.inverse().xform(targetVector))).slerp(Quat(), 1 - shoulder_influence);
				Transform customPose = Transform(offsetQuat * quatAlignToTarget, Vector3());
				skeleton->set_bone_custom_pose(rootBone, customPose);
			}
			root = skeleton->get_global_transform() * skeleton->get_bone_global_pose(rootBone);
		}
		apply_ik_map(solve_trig_ik_redux(limb_arm_right, root, hand_right_target_spatial->get_global_transform()));
	}
}

void RenIK::perform_foot_left_ik() {
	if (skeleton && limb_leg_left->is_valid()) {
		if (foot_left_target_spatial) {
			Transform root = skeleton->get_global_transform();
			BoneId rootBone = skeleton->get_bone_parent(limb_leg_left->get_upper_bone());
			if (rootBone >= 0) {
				root = root * skeleton->get_bone_global_pose(rootBone);
			}
			apply_ik_map(solve_trig_ik_redux(limb_leg_left, root, foot_left_target_spatial->get_global_transform()));
		} else if (foot_placement) {
			Transform root = skeleton->get_global_transform();
			BoneId rootBone = skeleton->get_bone_parent(limb_leg_left->get_upper_bone());
			if (rootBone >= 0) {
				root = root * skeleton->get_bone_global_pose(rootBone);
			}
			apply_ik_map(solve_trig_ik_redux(limb_leg_left, root, placement.interpolated_left_foot));
		}
	}
}

void RenIK::perform_foot_right_ik() {
	if (skeleton && limb_leg_right->is_valid()) {
		if (foot_right_target_spatial) {
			Transform root = skeleton->get_global_transform();
			BoneId rootBone = skeleton->get_bone_parent(limb_leg_right->get_upper_bone());
			if (rootBone >= 0) {
				root = root * skeleton->get_bone_global_pose(rootBone);
			}
			apply_ik_map(solve_trig_ik_redux(limb_leg_right, root, foot_right_target_spatial->get_global_transform()));
		} else if (foot_placement) {
			Transform root = skeleton->get_global_transform();
			BoneId rootBone = skeleton->get_bone_parent(limb_leg_right->get_upper_bone());
			if (rootBone >= 0) {
				root = root * skeleton->get_bone_global_pose(rootBone);
			}
			apply_ik_map(solve_trig_ik_redux(limb_leg_right, root, placement.interpolated_right_foot));
		}
	}
}

void RenIK::reset_chain(Ref<RenIKChain> chain) {
	if (skeleton && chain->get_leaf_bone() < skeleton->get_bone_count() && chain->get_root_bone() < skeleton->get_bone_count()) {
		BoneId bone = chain->get_leaf_bone();
		while (bone >= 0 && bone != chain->get_root_bone()) {
			//skeleton->set_bone_global_pose_override(bone, Transform(), 0, false);
			skeleton->set_bone_custom_pose(bone, Transform());
			bone = skeleton->get_bone_parent(bone);
		}
		if (bone >= 0) {
			//skeleton->set_bone_global_pose_override(bone, Transform(), 0, false);
			skeleton->set_bone_custom_pose(bone, Transform());
		}
	}
}

void RenIK::reset_limb(Ref<RenIKLimb> limb) {
	if (skeleton && limb->get_upper_bone() >= 0 && limb->get_lower_bone() >= 0 && limb->get_upper_bone() < skeleton->get_bone_count() && limb->get_lower_bone() < skeleton->get_bone_count()) {
		skeleton->set_bone_custom_pose(limb->get_upper_bone(), Transform());
		skeleton->set_bone_custom_pose(limb->get_lower_bone(), Transform());
		skeleton->set_bone_custom_pose(limb->get_leaf_bone(), Transform());
	}
}

//IK SOLVING

Map<BoneId, Quat> RenIK::solve_trig_ik(Ref<RenIKLimb> limb, Transform root, Transform target) {
	Map<BoneId, Quat> map;

	if (limb->is_valid()) { //There's no way to find a valid upperId if any of the other Id's are invalid, so we only check upperId
		Vector3 upperVector = limb->get_lower().get_origin();
		Vector3 lowerVector = limb->get_leaf().get_origin();
		Quat upperRest = limb->get_upper().get_basis().get_rotation_quat();
		Quat lowerRest = limb->get_lower().get_basis().get_rotation_quat();
		Quat upper = upperRest.inverse();
		Quat lower = lowerRest.inverse();
		//The true root of the limb is the point where the upper bone starts
		Transform trueRoot = root.translated(limb->get_upper().get_origin());
		Transform diff = root.affine_inverse() * trueRoot;
		Transform localTarget = trueRoot.affine_inverse() * target;

		//First we offset the pole
		upper = upper * limb->pole_offset.normalized(); //pole_offset is a euler because that's more human readable
		upper.normalize();
		lower.normalize();
		//Then we line up the limb with our target
		Vector3 targetVector = limb->pole_offset.inverse().xform(localTarget.get_origin());
		upper = upper * RenIKHelper::align_vectors(upperVector, targetVector);
		//Then we calculate how much we need to bend so we don't extend past the target
		//Law of Cosines
		float upperLength = upperVector.length();
		float lowerLength = lowerVector.length();
		float upperLength2 = upperVector.length_squared();
		float lowerLength2 = lowerVector.length_squared();
		float targetDistance = targetVector.length();
		float targetDistance2 = targetVector.length_squared();
		float upperAngle = RenIKHelper::safe_acos((upperLength2 + targetDistance2 - lowerLength2) / (2 * upperLength * targetDistance));
		float lowerAngle = RenIKHelper::safe_acos((upperLength2 + lowerLength2 - targetDistance2) / (2 * upperLength * lowerLength)) - Math_PI;
		Vector3 bendAxis = RenIKHelper::get_perpendicular_vector(upperVector); //TODO figure out how to set this automatically to the right axis
		Quat upperBend = Quat(bendAxis, upperAngle);
		Quat lowerBend = Quat(bendAxis, lowerAngle);
		upper = upper * upperBend;
		lower = lower * lowerBend;
		//Then we roll the limb based on the target position
		Vector3 targetRestPosition = upperVector.normalized() * (upperLength + lowerLength);
		Vector3 rollVector = upperBend.inverse().xform(upperVector).normalized();
		float positionalRollAmount = limb->target_position_influence.dot(targetRestPosition - targetVector);
		Quat positionRoll = Quat(rollVector, positionalRollAmount);
		upper = upper.normalized() * positionRoll;
		//And the target rotation

		Quat leafRest = limb->get_leaf().get_basis().get_rotation_quat();
		Quat armCombined = (upperRest * upper * lowerRest * lower).normalized();
		Quat targetQuat = localTarget.get_basis().get_rotation_quat() * leafRest;
		Quat leaf = ((armCombined * leafRest).inverse() * targetQuat).normalized();
		// if we had a plane along the roll vector we can project the leaf and lower limb on it to see which direction we need to roll to reduce the angle between the two
		Vector3 restVector = (armCombined).xform(lowerVector).normalized();
		Vector3 leafVector = leaf.xform(restVector).normalized();
		Vector3 restRejection = RenIKHelper::vector_rejection(restVector.normalized(), rollVector);
		Vector3 leafRejection = RenIKHelper::vector_rejection(leafVector.normalized(), rollVector);
		float directionalRollAmount = RenIKHelper::safe_acos(restRejection.normalized().dot(leafRejection.normalized())) * limb->target_rotation_influence;
		Vector3 directionality = restRejection.normalized().cross(leafRejection.normalized());
		float check = directionality.dot(targetVector.normalized());
		if (check > 0) {
			directionalRollAmount *= -1;
		}
		Quat directionalRoll = Quat(rollVector, directionalRollAmount);
		upper = upper * directionalRoll;

		armCombined = (upperRest * upper * lowerRest * lower).normalized();
		leaf = ((armCombined * leafRest).inverse() * targetQuat).normalized();
		//And finally add the twisting
		// old way: Quat lowerTwist = (align_vectors(lowerVector, leafRest.xform(leaf.xform(lowerVector))).inverse() * (leafRest * leaf)).slerp(Quat(), 1 - limb->lower_limb_twist).normalized();
		Vector3 twist = (leafRest * leaf).get_euler();
		Quat lowerTwist = Quat((leafRest * leaf).get_euler() * lowerVector.normalized() * (limb->lower_limb_twist));
		lower = lower * lowerTwist;
		leaf = (lowerTwist * leafRest).inverse() * leafRest * leaf;

		Quat upperTwist = Quat(twist * upperVector.normalized() * (limb->upper_limb_twist * limb->lower_limb_twist));
		upper = upper * upperTwist;
		lower = (upperTwist * lowerRest).inverse() * lowerRest * lower;

		//save data and return
		map.insert(limb->get_upper_bone(), upper);
		map.insert(limb->get_lower_bone(), lower);
		map.insert(limb->get_leaf_bone(), leaf);
	}
	return map;
}

std::pair<float, float> RenIK::trig_angles(Vector3 const &side1, Vector3 const &side2, Vector3 const &side3) {
	//Law of Cosines
	float length1Squared = side1.length_squared();
	float length2Squared = side2.length_squared();
	float length3Squared = side3.length_squared();
	float length1 = sqrt(length1Squared) * 2;
	float length2 = sqrt(length2Squared);
	float length3 = sqrt(length3Squared); // multiply by 2 here to save on having to multiply by 2 twice later
	float angle1 = RenIKHelper::safe_acos((length1Squared + length3Squared - length2Squared) / (length1 * length3));
	float angle2 = Math_PI - RenIKHelper::safe_acos((length1Squared + length2Squared - length3Squared) / (length1 * length2));
	return std::make_pair(angle1, angle2);
}

Map<BoneId, Basis> RenIK::solve_trig_ik_redux(Ref<RenIKLimb> limb, Transform root, Transform target) {
	Map<BoneId, Basis> map;
	if (limb->is_valid()) {
		//The true root of the limb is the point where the upper bone starts
		Transform trueRoot = root.translated(limb->get_upper().get_origin());
		Transform localTarget = trueRoot.affine_inverse() * target;

		//The Triangle
		Vector3 upperVector = limb->get_lower().get_origin();
		Vector3 lowerVector = limb->get_leaf().get_origin();
		Vector3 targetVector = localTarget.get_origin();
		Vector3 normalizedTargetVector = targetVector.normalized();
		float limbLength = upperVector.length() + lowerVector.length();
		if (targetVector.length() > upperVector.length() + lowerVector.length()) {
			targetVector = normalizedTargetVector * limbLength;
		}
		std::pair<float, float> angles = trig_angles(upperVector, lowerVector, targetVector);

		//The local x-axis of the upper limb is axis along which the limb will bend
		//We take into account how the pole offset and alignment with the target vector will affect this axis
		Vector3 startingPole = limb->pole_offset.xform(Vector3(0, 1, 0)); //the opposite of this vector is where the pole is
		Vector3 jointAxis = RenIKHelper::align_vectors(startingPole, targetVector).xform(limb->pole_offset.xform(Vector3(1, 0, 0)));

		// //We then find how far away from the rest position the leaf is and use that to change the rotational axis more.
		Vector3 leafRestVector = limb->get_upper().get_basis().xform(limb->get_lower().xform(limb->get_leaf().get_origin()));
		float positionalOffset = limb->target_position_influence.dot(targetVector - leafRestVector);
		jointAxis.rotate(normalizedTargetVector, positionalOffset + limb->roll_offset);

		//Leaf Rotations... here we go...
		//Let's always try to avoid having the leaf intersect the lowerlimb
		//First we find the a vector that corresponds with the direction the leaf and lower limbs are pointing local to the true root
		Vector3 localLeafVector = localTarget.get_basis().xform(Vector3(0, 1, 0)); //y axis of the target
		Vector3 localLowerVector = normalizedTargetVector.rotated(jointAxis, angles.first - angles.second).normalized();
		//We then take the vector rejections of the leaf and lower limb against the target vector
		//A rejection is the opposite of a projection. We use the target vector because that's our axis of rotation for the whole limb->
		//We then turn the whole arm along the target vector based on how close the rejections are
		//We scale the amount we rotate with the rotation influence setting and the angle between the leaf and lower vector so if the arm is mostly straight, we rotate less
		Vector3 leafRejection = RenIKHelper::vector_rejection(localLeafVector, normalizedTargetVector);
		Vector3 lowerRejection = RenIKHelper::vector_rejection(localLowerVector, normalizedTargetVector);
		float jointRollAmount = (leafRejection.angle_to(lowerRejection)) * limb->target_rotation_influence;
		jointRollAmount *= abs(localLeafVector.cross(localLowerVector).dot(normalizedTargetVector));
		if (leafRejection.cross(lowerRejection).dot(normalizedTargetVector) > 0) {
			jointRollAmount *= -1;
		}
		jointAxis.rotate(normalizedTargetVector, jointRollAmount);
		float totalRoll = jointRollAmount + positionalOffset + limb->roll_offset;

		//Add a little twist
		//We align the leaf's y axis with the lower limb's y-axis and see how far off the x-axis is from the joint axis to calculate the twist.
		Vector3 leafX = RenIKHelper::align_vectors(localLeafVector.rotated(normalizedTargetVector, jointRollAmount), localLowerVector.rotated(normalizedTargetVector, jointRollAmount)).xform(localTarget.get_basis().xform(Vector3(1, 0, 0)));
		Vector3 rolledJointAxis = jointAxis.rotated(localLowerVector, -totalRoll);
		Vector3 lowerZ = rolledJointAxis.cross(localLowerVector);
		float twistAngle = leafX.angle_to(rolledJointAxis);
		if (leafX.dot(lowerZ) > 0) {
			twistAngle *= -1;
		}

		float inflectionPoint = twistAngle > 0 ? Math_PI - limb->twist_inflection_point_offset : -Math_PI - limb->twist_inflection_point_offset;
		float overflowArea = limb->overflow_state * limb->twist_overflow;
		float inflectionDistance = twistAngle - inflectionPoint;

		if (Math::abs(inflectionDistance) < limb->twist_overflow) {
			if (limb->overflow_state == 0) {
				limb->overflow_state = inflectionDistance < 0 ? 1 : -1;
			}
		} else {
			limb->overflow_state = 0;
		}

		inflectionPoint += overflowArea;
		if (twistAngle > 0 && twistAngle > inflectionPoint) {
			twistAngle -= Math_TAU; //Change to complement angle
		} else if (twistAngle < 0 && twistAngle < inflectionPoint) {
			twistAngle += Math_TAU; //Change to complement angle
		}

		float lowerTwist = twistAngle * limb->lower_limb_twist;
		float upperTwist = lowerTwist * limb->upper_limb_twist + limb->upper_twist_offset - totalRoll;
		lowerTwist += limb->lower_twist_offset - 2 * limb->roll_offset - positionalOffset - jointRollAmount;

		jointAxis.rotate(normalizedTargetVector, twistAngle * limb->target_rotation_influence);

		//Rebuild the rotations
		Vector3 upperJointVector = normalizedTargetVector.rotated(jointAxis, angles.first);
		Vector3 rolledLowerJointAxis = Vector3(1, 0, 0).rotated(Vector3(0, 1, 0), -limb->roll_offset);
		Vector3 lowerJointVector = Vector3(0, 1, 0).rotated(rolledLowerJointAxis, angles.second);
		Vector3 twistedJointAxis = jointAxis.rotated(upperJointVector, upperTwist);
		Basis upperBasis = Basis(twistedJointAxis, upperJointVector, twistedJointAxis.cross(upperJointVector)).inverse();
		Basis lowerBasis = Basis(rolledLowerJointAxis, lowerJointVector, rolledLowerJointAxis.cross(lowerJointVector));
		lowerBasis.rotate_local(Vector3(0, 1, 0), lowerTwist);
		lowerBasis.rotate(Vector3(0, 1, 0), -upperTwist);

		map[limb->get_upper_bone()] = limb->get_upper().get_basis().inverse() * upperBasis;
		map[limb->get_lower_bone()] = limb->get_lower().get_basis().inverse() * lowerBasis;
		map[limb->get_leaf_bone()] = limb->get_leaf().get_basis().inverse() * (upperBasis * lowerBasis).inverse() * localTarget.get_basis() * limb->get_leaf().get_basis();
	}
	return map;
}

Map<BoneId, Quat> RenIK::solve_ifabrik(Ref<RenIKChain> chain, Transform root, Transform target, float threshold, int loopLimit) {
	Map<BoneId, Quat> map;
	if (chain->is_valid()) { //if the chain is valid there's at least one joint in the chain and there's one bone between it and the root
		Vector<RenIKChain::Joint> joints = chain->get_joints(); //just so I don't have to call it all the time
		Transform trueRoot = root.translated(joints[0].relative_prev);
		Transform targetDelta = target * chain->get_relative_rest_leaf().affine_inverse(); //how the change in the target would affect the chain if the chain was parented to the target instead of the root
		Transform trueRelativeTarget = trueRoot.affine_inverse() * target;
		Quat alignToTarget = RenIKHelper::align_vectors(chain->get_relative_rest_leaf().origin - joints[0].relative_prev, trueRelativeTarget.origin);
		float heightDiff = (chain->get_relative_rest_leaf().origin - joints[0].relative_prev).length() - trueRelativeTarget.origin.length();
		heightDiff = heightDiff < 0 ? 0 : heightDiff;
		Transform prebentRoot = Transform(trueRoot.basis * alignToTarget, trueRoot.origin).translated((chain->chain_curve_direction * chain->get_total_length() * heightDiff) - joints[0].relative_prev); //The angle root is rotated to point at the target;

		Vector<Vector3> globalJointPoints;

		//We generate the starting points
		//Here is where we take into account root and target influences and the prebend vector
		Vector3 relativeJoint = joints[0].relative_prev;
		for (int i = 1; i < joints.size(); i++) {
			relativeJoint = relativeJoint + joints[i].relative_prev;
			Vector3 prebentJoint = prebentRoot.xform(relativeJoint); //if you rotated the root around the true root so that the whole chain was pointing to the leaf and then you moved everything along the prebend vector
			Vector3 rootJoint = root.xform(relativeJoint); //if you moved the joint with the root
			Vector3 leafJoint = targetDelta.xform(relativeJoint); //if you moved the joint with the leaf
			prebentJoint = prebentJoint.linear_interpolate(rootJoint, joints[i].root_influence);
			prebentJoint = prebentJoint.linear_interpolate(leafJoint, joints[i].leaf_influence); //leaf influence dominates
			globalJointPoints.push_back(prebentJoint);
		}

		//We then do regular FABRIK
		for (int i = 0; i < loopLimit; i++) {
			Vector3 lastJoint = target.origin;
			//Backwards
			for (int j = joints.size() - 1; j >= 1; j--) { //we skip the first joint because we're not allowed to move that joint
				Vector3 delta = globalJointPoints[j - 1] - lastJoint;
				delta = delta.normalized() * joints[j].next_distance;
				globalJointPoints.set(j - 1, lastJoint + delta);
				lastJoint = globalJointPoints[j - 1];
			}
			lastJoint = trueRoot.origin; //the root joint

			//Forwards
			for (int j = 1; j < joints.size(); j++) { //we skip the first joint because we're not allowed to move that joint
				Vector3 delta = globalJointPoints[j - 1] - lastJoint;
				delta = delta.normalized() * joints[j].prev_distance;
				globalJointPoints.set(j - 1, lastJoint + delta);
				lastJoint = globalJointPoints[j - 1];
			}

			float error = (lastJoint - trueRoot.origin).length();
			if (error < threshold) {
				break;
			}
		}

		//Add a little twist
		//We align the leaf's y axis with the rest_leaf's y-axis and see how far off the x-axes are to calculate the twist.
		trueRelativeTarget.orthonormalize();
		Vector3 leafX = RenIKHelper::align_vectors(trueRelativeTarget.basis.xform(Vector3(0, 1, 0)), chain->get_relative_rest_leaf().basis.xform(Vector3(0, 1, 0))).normalized().xform(trueRelativeTarget.basis.xform(Vector3(1, 0, 0)));
		Vector3 restX = chain->get_relative_rest_leaf().basis.xform(Vector3(1, 0, 0));
		Vector3 restZ = chain->get_relative_rest_leaf().basis.xform(Vector3(0, 0, 1));
		float maxTwist = leafX.angle_to(restX);
		if (leafX.cross(restX).dot(Vector3(0, 1, 0)) > 0) {
			maxTwist *= -1;
		}

		//Convert everything to quaternions and store it in the map
		Quat parentRot = root.get_basis().get_quat();
		Vector3 parentPos = trueRoot.origin;
		Quat prevTwist;
		globalJointPoints.push_back(target.origin);
		for (int i = 0; i < joints.size(); i++) { //the last one's rotation is defined by the leaf position not a joint so we skip it
			Quat pose = RenIKHelper::align_vectors(Vector3(0, 1, 0), Transform(parentRot * joints[i].rotation, parentPos).affine_inverse().xform(globalJointPoints[i])); //offset by one because joints has one extra element
			Quat twist = Quat(Vector3(0, 1, 0), maxTwist * joints[i].twist_influence);
			pose = prevTwist.inverse() * joints[i].rotation * pose * twist;
			prevTwist = twist;
			map.insert(joints[i].id, pose);
			parentRot = parentRot * pose;
			parentPos = globalJointPoints[i];
		}

		// parentRot = parentRot * joints[0].rotation;
		// parent.translate(joints[0].relative_prev);
		// Quat pose = align_vectors(axis, Transform(parentRot, parent.origin).affine_inverse().xform(target.origin)); //offset by one because joints has one extra element
		// map.insert(joints[0].id, pose);
		// parentRot = parentRot * pose;
	}
	return map;
}

Vector<BoneId> RenIK::calculate_bone_chain(BoneId root, BoneId leaf) {
	Vector<BoneId> chain;
	BoneId b = leaf;
	chain.push_back(b);
	if (skeleton) {
		while (b >= 0 && b != root) {
			b = skeleton->get_bone_parent(b);
			chain.push_back(b);
		}
		if (b < 0) {
			chain.clear();
			chain.push_back(leaf);
		} else {
			chain.invert();
			int first = chain[0];
			int last = chain[chain.size() - 1];
		}
	}
	return chain;
}

bool RenIK::get_live_preview() {
	return live_preview;
}
void RenIK::set_live_preview(bool p_enable) {
	live_preview = p_enable;
	if (!live_preview && skeleton) {
		if (head_target_spatial || hip_target_spatial || hip_placement) {
			reset_chain(spine_chain);
		}
		if (hand_left_target_spatial) {
			reset_limb(limb_arm_left);
			if (skeleton && left_shoulder_enabled) {
				skeleton->set_bone_custom_pose(skeleton->get_bone_parent(limb_arm_left->get_upper_bone()), Transform());
			}
		}
		if (hand_right_target_spatial) {
			reset_limb(limb_arm_right);
			if (skeleton && right_shoulder_enabled) {
				skeleton->set_bone_custom_pose(skeleton->get_bone_parent(limb_arm_right->get_upper_bone()), Transform());
			}
		}
		if (foot_left_target_spatial || foot_placement) {
			reset_limb(limb_leg_left);
		}
		if (foot_right_target_spatial || foot_placement) {
			reset_limb(limb_leg_right);
		}
	}
}

NodePath
RenIK::get_skeleton_path() {
	if (skeleton) {
		return skeleton->get_path();
	}
	return skeleton_path;
}
void RenIK::set_skeleton_path(NodePath p_path) {
	skeleton_path = p_path;
	if (is_inside_tree()) {
		Skeleton *new_node = Object::cast_to<Skeleton>(get_node_or_null(p_path));
		skeleton = new_node;
		set_head_bone_by_name(get_head_bone_name());
		set_hip_bone_by_name(get_hip_bone_name());
		set_hand_left_bone_by_name(get_hand_left_bone_name());
		set_hand_right_bone_by_name(get_hand_right_bone_name());
		set_foot_left_bone_by_name(get_foot_left_bone_name());
		set_foot_right_bone_by_name(get_foot_right_bone_name());
	}
}
void RenIK::set_skeleton(Node *p_node) {
	skeleton_path = p_node->get_path();
	Skeleton *skeleton = Object::cast_to<Skeleton>(p_node);
	if (skeleton != nullptr) {
		set_head_bone_by_name(get_head_bone_name());
		set_hip_bone_by_name(get_hip_bone_name());
		set_hand_left_bone_by_name(get_hand_left_bone_name());
		set_hand_right_bone_by_name(get_hand_right_bone_name());
		set_foot_left_bone_by_name(get_foot_left_bone_name());
		set_foot_right_bone_by_name(get_foot_right_bone_name());
	}
}

void RenIK::set_head_bone_by_name(String p_bone) {
	head_bone_name = p_bone;
	if (skeleton) {
		BoneId id = skeleton->find_bone(p_bone);
		if (id >= 0) {
			set_head_bone(id);
		}
	}
}
void RenIK::set_hand_left_bone_by_name(String p_bone) {
	hand_left_bone_name = p_bone;
	if (skeleton) {
		BoneId id = skeleton->find_bone(p_bone);
		if (id >= 0) {
			set_hand_left_bone(id);
		}
	}
}
void RenIK::set_hand_right_bone_by_name(String p_bone) {
	hand_right_bone_name = p_bone;
	if (skeleton) {
		BoneId id = skeleton->find_bone(p_bone);
		if (id >= 0) {
			set_hand_right_bone(id);
		}
	}
}
void RenIK::set_hip_bone_by_name(String p_bone) {
	hip_bone_name = p_bone;
	if (skeleton) {
		BoneId id = skeleton->find_bone(p_bone);
		if (id >= 0) {
			set_hip_bone(id);
		}
	}
}
void RenIK::set_foot_left_bone_by_name(String p_bone) {
	foot_left_bone_name = p_bone;
	if (skeleton) {
		BoneId id = skeleton->find_bone(p_bone);
		if (id >= 0) {
			set_foot_left_bone(id);
		}
	}
}
void RenIK::set_foot_right_bone_by_name(String p_bone) {
	foot_right_bone_name = p_bone;
	if (skeleton) {
		BoneId id = skeleton->find_bone(p_bone);
		if (id >= 0) {
			set_foot_right_bone(id);
		}
	}
}

void RenIK::set_head_bone(BoneId p_bone) {
	head = p_bone;
	spine_chain->set_leaf_bone(skeleton, p_bone);
	placement.spine_length = spine_chain->get_total_length();
	//calc rest offset of hips
	if (head >= 0 && head < skeleton->get_bone_count() && hip >= 0 && hip < skeleton->get_bone_count()) {
		placement.hip_offset = skeleton->get_bone_global_pose(hip).origin - skeleton->get_bone_global_pose(head).origin;
	}
}

void RenIK::set_hand_left_bone(BoneId p_bone) {
	limb_arm_left->set_leaf(skeleton, p_bone);
	left_shoulder_enabled = skeleton && limb_arm_left->is_valid() && !spine_chain->contains_bone(skeleton, skeleton->get_bone_parent(limb_arm_left->get_lower_bone()));
}

void RenIK::set_hand_right_bone(BoneId p_bone) {
	limb_arm_right->set_leaf(skeleton, p_bone);
	right_shoulder_enabled = skeleton && limb_arm_right->is_valid() && !spine_chain->contains_bone(skeleton, skeleton->get_bone_parent(limb_arm_right->get_lower_bone()));
}

void RenIK::set_hip_bone(BoneId p_bone) {
	hip = p_bone;
	spine_chain->set_root_bone(skeleton, p_bone);
	placement.spine_length = spine_chain->get_total_length();

	//calc rest offset of hips
	if (head >= 0 && head < skeleton->get_bone_count() && hip >= 0 && hip < skeleton->get_bone_count()) {
		placement.hip_offset = skeleton->get_bone_global_pose(hip).origin - skeleton->get_bone_global_pose(head).origin;
	}
}

void RenIK::set_foot_left_bone(BoneId p_bone) {
	limb_leg_left->set_leaf(skeleton, p_bone);
	placement.left_leg_length = limb_leg_left->is_valid() ? limb_leg_left->lower.origin.length() + limb_leg_left->leaf.origin.length() : 0;
	placement.left_hip_offset = limb_leg_left->is_valid() ? limb_leg_left->upper.origin : Vector3();
}

void RenIK::set_foot_right_bone(BoneId p_bone) {
	limb_leg_right->set_leaf(skeleton, p_bone);
	placement.right_leg_length = limb_leg_right->is_valid() ? limb_leg_right->lower.origin.length() + limb_leg_right->leaf.origin.length() : 0;
	placement.right_hip_offset = limb_leg_right->is_valid() ? limb_leg_right->upper.origin : Vector3();
}

int64_t RenIK::get_hip_bone() {
	return hip;
}
int64_t RenIK::get_head_bone() {
	return head;
}
int64_t RenIK::get_hand_left_bone() {
	return limb_arm_left->get_leaf_bone();
}
int64_t RenIK::get_hand_right_bone() {
	return limb_arm_right->get_leaf_bone();
}
int64_t RenIK::get_foot_left_bone() {
	return limb_leg_left->get_leaf_bone();
}
int64_t RenIK::get_foot_right_bone() {
	return limb_leg_right->get_leaf_bone();
}

String RenIK::get_hip_bone_name() {
	return hip_bone_name;
}
String RenIK::get_head_bone_name() {
	return head_bone_name;
}
String RenIK::get_hand_left_bone_name() {
	return hand_left_bone_name;
}
String RenIK::get_hand_right_bone_name() {
	return hand_right_bone_name;
}
String RenIK::get_foot_left_bone_name() {
	return foot_left_bone_name;
}
String RenIK::get_foot_right_bone_name() {
	return foot_right_bone_name;
}

NodePath RenIK::get_head_target_path() {
	return head_target_path;
}
void RenIK::set_head_target_path(NodePath p_path) {
	head_target_path = p_path;
	if (is_inside_tree()) {
		Spatial *new_node = Object::cast_to<Spatial>(get_node_or_null(p_path));
		if (new_node || p_path.is_empty()) {
			head_target_spatial = new_node;
		}
	}
}

NodePath RenIK::get_hand_left_target_path() {
	return hand_left_target_path;
}

void RenIK::set_hand_left_target_path(NodePath p_path) {
	hand_left_target_path = p_path;
	if (is_inside_tree()) {
		Spatial *new_node = Object::cast_to<Spatial>(get_node_or_null(p_path));
		if (new_node || p_path.is_empty()) {
			hand_left_target_spatial = new_node;
		}
	}
}

NodePath RenIK::get_hand_right_target_path() {
	return hand_right_target_path;
}

void RenIK::set_hand_right_target_path(NodePath p_path) {
	hand_right_target_path = p_path;
	if (is_inside_tree()) {
		Spatial *new_node = Object::cast_to<Spatial>(get_node_or_null(p_path));
		if (new_node || p_path.is_empty()) {
			hand_right_target_spatial = new_node;
		}
	}
}

NodePath RenIK::get_hip_target_path() {
	return hip_target_path;
}

void RenIK::set_hip_target_path(NodePath p_path) {
	hip_target_path = p_path;
	if (is_inside_tree()) {
		Spatial *new_node = Object::cast_to<Spatial>(get_node_or_null(p_path));
		if (new_node || p_path.is_empty()) {
			hip_target_spatial = new_node;
		}
	}
}

NodePath RenIK::get_foot_left_target_path() {
	return foot_left_target_path;
}

void RenIK::set_foot_left_target_path(NodePath p_path) {
	foot_left_target_path = p_path;
	if (is_inside_tree()) {
		Spatial *new_node = Object::cast_to<Spatial>(get_node_or_null(p_path));
		if (new_node || p_path.is_empty()) {
			foot_left_target_spatial = new_node;
		}
	}
}

NodePath RenIK::get_foot_right_target_path() {
	return foot_right_target_path;
}

void RenIK::set_foot_right_target_path(NodePath p_path) {
	foot_right_target_path = p_path;
	if (is_inside_tree()) {
		Spatial *new_node = Object::cast_to<Spatial>(get_node_or_null(p_path));
		if (new_node || p_path.is_empty()) {
			foot_right_target_spatial = new_node;
		}
	}
}
float RenIK::get_arm_upper_twist_offset() {
	return Math::rad2deg(limb_arm_left->upper_twist_offset);
}
void RenIK::set_arm_upper_twist_offset(float degrees) {
	limb_arm_left->upper_twist_offset = Math::deg2rad(degrees);
	limb_arm_right->upper_twist_offset = Math::deg2rad(-degrees);
}
float RenIK::get_arm_lower_twist_offset() {
	return Math::rad2deg(limb_arm_left->lower_twist_offset);
}
void RenIK::set_arm_lower_twist_offset(float degrees) {
	limb_arm_left->lower_twist_offset = Math::deg2rad(degrees);
	limb_arm_right->lower_twist_offset = Math::deg2rad(-degrees);
}
float RenIK::get_arm_roll_offset() {
	return Math::rad2deg(limb_arm_left->roll_offset);
}
void RenIK::set_arm_roll_offset(float degrees) {
	limb_arm_left->roll_offset = Math::deg2rad(degrees);
	limb_arm_right->roll_offset = Math::deg2rad(-degrees);
}
float RenIK::get_arm_upper_limb_twist() {
	return limb_arm_left->upper_limb_twist * 100;
}
void RenIK::set_arm_upper_limb_twist(float ratio) {
	limb_arm_left->upper_limb_twist = ratio / 100.0;
	limb_arm_right->upper_limb_twist = ratio / 100.0;
}
float RenIK::get_arm_lower_limb_twist() {
	return limb_arm_left->lower_limb_twist * 100;
}
void RenIK::set_arm_lower_limb_twist(float ratio) {
	limb_arm_left->lower_limb_twist = ratio / 100.0;
	limb_arm_right->lower_limb_twist = ratio / 100.0;
}
float RenIK::get_arm_twist_inflection_point_offset() {
	return Math::rad2deg(limb_arm_left->twist_inflection_point_offset);
}
void RenIK::set_arm_twist_inflection_point_offset(float degrees) {
	limb_arm_left->twist_inflection_point_offset = Math::deg2rad(degrees);
	limb_arm_right->twist_inflection_point_offset = Math::deg2rad(-degrees);
}
float RenIK::get_arm_twist_overflow() {
	return Math::rad2deg(limb_arm_left->twist_overflow);
}
void RenIK::set_arm_twist_overflow(float degrees) {
	limb_arm_left->twist_overflow = Math::deg2rad(degrees);
	limb_arm_right->twist_overflow = Math::deg2rad(degrees);
}

Vector3 RenIK::get_arm_pole_offset() {
	Vector3 v = limb_arm_left->pole_offset.get_euler();
	return Vector3(Math::rad2deg(v[0]), Math::rad2deg(v[1]), Math::rad2deg(v[2]));
}
void RenIK::set_arm_pole_offset(Vector3 euler) {
	Quat q = Quat(Vector3(Math::deg2rad(euler[0]), Math::deg2rad(euler[1]), Math::deg2rad(euler[2])));
	Quat q2 = Quat(Vector3(Math::deg2rad(euler[0]), Math::deg2rad(-euler[1]), Math::deg2rad(-euler[2])));
	limb_arm_left->pole_offset = q;
	limb_arm_right->pole_offset = q2;
}
Vector3 RenIK::get_arm_target_position_influence() {
	return limb_arm_left->target_position_influence * 10.0;
}
void RenIK::set_arm_target_position_influence(Vector3 xyz) {
	limb_arm_left->target_position_influence = xyz / 10.0;
	limb_arm_right->target_position_influence = Vector3(xyz[0], -xyz[1], -xyz[2]) / 10.0;
}
float RenIK::get_arm_target_rotation_influence() {
	return limb_arm_left->target_rotation_influence * 100.0;
}
void RenIK::set_arm_target_rotation_influence(float influence) {
	limb_arm_left->target_rotation_influence = influence / 100.0;
	limb_arm_right->target_rotation_influence = influence / 100.0;
}

float RenIK::get_leg_upper_twist_offset() {
	return Math::rad2deg(limb_leg_left->upper_twist_offset);
}
void RenIK::set_leg_upper_twist_offset(float degrees) {
	limb_leg_left->upper_twist_offset = Math::deg2rad(degrees);
	limb_leg_right->upper_twist_offset = Math::deg2rad(-degrees);
}
float RenIK::get_leg_lower_twist_offset() {
	return Math::rad2deg(limb_leg_left->lower_twist_offset);
}
void RenIK::set_leg_lower_twist_offset(float degrees) {
	limb_leg_left->lower_twist_offset = Math::deg2rad(degrees);
	limb_leg_right->lower_twist_offset = Math::deg2rad(-degrees);
}
float RenIK::get_leg_roll_offset() {
	return Math::rad2deg(limb_leg_left->roll_offset);
}
void RenIK::set_leg_roll_offset(float degrees) {
	limb_leg_left->roll_offset = Math::deg2rad(degrees);
	limb_leg_right->roll_offset = Math::deg2rad(-degrees);
}
float RenIK::get_leg_upper_limb_twist() {
	return limb_leg_left->upper_limb_twist * 100;
}
void RenIK::set_leg_upper_limb_twist(float ratio) {
	limb_leg_left->upper_limb_twist = ratio / 100.0;
	limb_leg_right->upper_limb_twist = ratio / 100.0;
}
float RenIK::get_leg_lower_limb_twist() {
	return limb_leg_left->lower_limb_twist * 100;
}
void RenIK::set_leg_lower_limb_twist(float ratio) {
	limb_leg_left->lower_limb_twist = ratio / 100.0;
	limb_leg_right->lower_limb_twist = ratio / 100.0;
}
float RenIK::get_leg_twist_inflection_point_offset() {
	return Math::rad2deg(limb_leg_left->twist_inflection_point_offset);
}
void RenIK::set_leg_twist_inflection_point_offset(float degrees) {
	limb_leg_left->twist_inflection_point_offset = Math::deg2rad(degrees);
	limb_leg_right->twist_inflection_point_offset = Math::deg2rad(-degrees);
}
float RenIK::get_leg_twist_overflow() {
	return Math::rad2deg(limb_leg_left->twist_overflow);
}
void RenIK::set_leg_twist_overflow(float degrees) {
	limb_leg_left->twist_overflow = Math::deg2rad(degrees);
	limb_leg_right->twist_overflow = Math::deg2rad(degrees);
}

Vector3 RenIK::get_leg_pole_offset() {
	Vector3 v = limb_leg_left->pole_offset.get_euler();
	return Vector3(Math::rad2deg(v[0]), Math::rad2deg(v[1]), Math::rad2deg(v[2]));
}
void RenIK::set_leg_pole_offset(Vector3 euler) {
	Quat q = Quat(Vector3(Math::deg2rad(euler[0]), Math::deg2rad(euler[1]), Math::deg2rad(euler[2])));
	Quat q2 = Quat(Vector3(Math::deg2rad(euler[0]), Math::deg2rad(-euler[1]), Math::deg2rad(-euler[2])));
	limb_leg_left->pole_offset = q;
	limb_leg_right->pole_offset = q2;
}
Vector3 RenIK::get_leg_target_position_influence() {
	return limb_leg_left->target_position_influence * 10.0;
}
void RenIK::set_leg_target_position_influence(Vector3 xyz) {
	limb_leg_left->target_position_influence = xyz / 10.0;
	limb_leg_right->target_position_influence = Vector3(xyz[0], -xyz[1], -xyz[2]) / 10.0;
}
float RenIK::get_leg_target_rotation_influence() {
	return limb_leg_left->target_rotation_influence * 100.0;
}
void RenIK::set_leg_target_rotation_influence(float influence) {
	limb_leg_left->target_rotation_influence = influence / 100.0;
	limb_leg_right->target_rotation_influence = influence / 100.0;
}

Vector3 RenIK::get_spine_curve() {
	return spine_chain->chain_curve_direction;
}
void RenIK::set_spine_curve(Vector3 direction) {
	spine_chain->chain_curve_direction = direction;
}
float RenIK::get_upper_spine_stiffness() {
	return spine_chain->get_leaf_stiffness() * 100.0;
}
void RenIK::set_upper_spine_stiffness(float influence) {
	spine_chain->set_leaf_stiffness(skeleton, influence / 100.0);
}
float RenIK::get_lower_spine_stiffness() {
	return spine_chain->get_root_stiffness() * 100.0;
}
void RenIK::set_lower_spine_stiffness(float influence) {
	spine_chain->set_root_stiffness(skeleton, influence / 100.0);
}
float RenIK::get_spine_twist() {
	return spine_chain->get_twist() * 100.0;
}
void RenIK::set_spine_twist(float influence) {
	spine_chain->set_twist(skeleton, influence / 100.0);
}
float RenIK::get_spine_twist_start() {
	return spine_chain->get_twist_start() * 100.0;
}
void RenIK::set_spine_twist_start(float influence) {
	spine_chain->set_twist_start(skeleton, influence / 100.0);
}

float RenIK::get_shoulder_influence() {
	return shoulder_influence * 100.0;
}
void RenIK::set_shoulder_influence(float influence) {
	shoulder_influence = influence / 100;
}

Vector3 RenIK::get_shoulder_offset() {
	return Vector3(Math::rad2deg(left_shoulder_offset[0]), Math::rad2deg(left_shoulder_offset[1]), Math::rad2deg(left_shoulder_offset[2]));
}
void RenIK::set_shoulder_offset(Vector3 euler) {
	left_shoulder_offset = Vector3(Math::deg2rad(euler[0]), Math::deg2rad(euler[1]), Math::deg2rad(euler[2]));
	right_shoulder_offset = Vector3(Math::deg2rad(euler[0]), -Math::deg2rad(euler[1]), -Math::deg2rad(euler[2]));
}

Vector3 RenIK::get_shoulder_pole_offset() {
	return Vector3(Math::rad2deg(left_shoulder_pole_offset[0]), Math::rad2deg(left_shoulder_pole_offset[1]), Math::rad2deg(left_shoulder_pole_offset[2]));
}
void RenIK::set_shoulder_pole_offset(Vector3 euler) {
	left_shoulder_pole_offset = Vector3(Math::deg2rad(euler[0]), Math::deg2rad(euler[1]), Math::deg2rad(euler[2]));
	right_shoulder_pole_offset = Vector3(Math::deg2rad(euler[0]), -Math::deg2rad(euler[1]), -Math::deg2rad(euler[2]));
}

//Placement
void RenIK::set_falling(bool p_value){
	placement.set_falling(p_value);
}

void RenIK::set_collision_mask(uint32_t p_mask) {
	placement.set_collision_mask(p_mask);
}

uint32_t RenIK::get_collision_mask() const {
	return placement.get_collision_mask();
}

void RenIK::set_collision_mask_bit(int p_bit, bool p_value) {
	placement.set_collision_mask_bit(p_bit, p_value);
}

bool RenIK::get_collision_mask_bit(int p_bit) const {
	return placement.get_collision_mask_bit(p_bit);
}

void RenIK::set_collide_with_areas(bool p_clip) {
	placement.set_collide_with_areas(p_clip);
}

bool RenIK::is_collide_with_areas_enabled() const {
	return placement.is_collide_with_areas_enabled();
}

void RenIK::set_collide_with_bodies(bool p_clip) {
	placement.set_collide_with_bodies(p_clip);
}

bool RenIK::is_collide_with_bodies_enabled() const {
	return placement.is_collide_with_bodies_enabled();
}

#endif // _3D_DISABLED
