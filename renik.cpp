#include "renik.h"
#ifndef _3D_DISABLED

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
		spine_chain(Vector3(0, 15, -15), 1, 1, 1, 0),
		left_shoulder_offset(Math::deg2rad(-20.0), Math::deg2rad(-10.0), Math::deg2rad(-10.0)),
		right_shoulder_offset(Math::deg2rad(-20.0), Math::deg2rad(10.0), Math::deg2rad(10.0)),
		left_shoulder_pole_offset(Math::deg2rad(78.0), Math::deg2rad(0.0), Math::deg2rad(0.0)),
		right_shoulder_pole_offset(Math::deg2rad(78.0), Math::deg2rad(0.0), Math::deg2rad(0.0)),
		limb_arm_left(0, 0, Math_PI, 0.5, 0.66666, Math::deg2rad(20.0), Math::deg2rad(45.0), 0.25, Vector3(Math::deg2rad(60.0), 0, 0), Vector3(2.0, -1, -2.0)),
		limb_arm_right(0, 0, -Math_PI, 0.5, 0.66666, Math::deg2rad(-20.0), Math::deg2rad(45.0), 0.25, Vector3(Math::deg2rad(60.0), 0, 0), Vector3(2.0, 1, 2.0)),
		limb_leg_left(0, 0, 0, 0.25, 0.25, 0, Math::deg2rad(45.0), 0.5, Vector3(0, 0, Math_PI), Vector3()),
		limb_leg_right(0, 0, 0, 0.25, 0.25, 0, Math::deg2rad(45.0), 0.5, Vector3(0, 0, -Math_PI), Vector3()){};

void RenIK::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_live_preview", "p_enable"), &RenIK::set_live_preview);
	ClassDB::bind_method(D_METHOD("get_live_preview"), &RenIK::get_live_preview);

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
	set_process_priority(1); //makes sure that ik is done last after all physics and movement have taken place
	enable_solve_ik_every_frame(true);
	enable_hip_placement(true);
	enable_foot_placement(true);
	set_physics_process_internal(true);
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
	perform_torso_ik();
	perform_hand_left_ik();
	perform_hand_right_ik();
	perform_foot_left_ik();
	perform_foot_right_ik();
}

void RenIK::update_placement(float delta) {
	//Based on head position and delta time, we calc our speed and distance from the ground and place the feet accordingly
	foot_place(delta, config, Set<RID>(), collision_mask, collide_with_bodies, collide_with_areas);
	//Once the feet are placed, we put the hips at a comfortable position between the two
	hip_place(delta, config);
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
	if (head_target_spatial && skeleton && spine_chain.is_valid()) {
		Transform headTransform = head_target_spatial->get_global_transform();
		Transform hipTransform = hip_target_spatial ? hip_target_spatial->get_global_transform() : placedHip;
		Vector3 delta = hipTransform.origin + hipTransform.basis.xform(spine_chain.get_joints()[0].relative_prev) - headTransform.origin;
		float fullLength = spine_chain.get_total_length();
		if (delta.length() > fullLength) {
			hipTransform.set_origin(headTransform.origin + (delta.normalized() * fullLength) - hipTransform.basis.xform(spine_chain.get_joints()[0].relative_prev));
		}
		skeleton->set_bone_global_pose_override(spine_chain.get_root_bone(), hipTransform, 1, true);
		apply_ik_map(solve_ifabrik(spine_chain, hipTransform, headTransform, 0.0005, 16));
		skeleton->set_bone_global_pose_override(spine_chain.get_leaf_bone(), headTransform, 1, true);
	}
}

void RenIK::perform_hand_left_ik() {
	if (hand_left_target_spatial && skeleton && limb_arm_left.is_valid()) {
		Transform root = skeleton->get_global_transform();
		BoneId rootBone = skeleton->get_bone_parent(limb_arm_left.get_upper_bone());
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
				Quat quatAlignToTarget = poleOffsetScaled * align_vectors(Vector3(0, 1, 0), poleOffset.inverse().xform(offsetQuat.inverse().xform(targetVector))).slerp(Quat(), 1 - shoulder_influence);
				Transform customPose = Transform(offsetQuat * quatAlignToTarget, Vector3());
				skeleton->set_bone_custom_pose(rootBone, customPose);
			}
			root = skeleton->get_global_transform() * skeleton->get_bone_global_pose(rootBone);
		}
		apply_ik_map(solve_trig_ik_redux(limb_arm_left, root, hand_left_target_spatial->get_global_transform()));
	}
}

void RenIK::perform_hand_right_ik() {
	if (hand_right_target_spatial && skeleton && limb_arm_right.is_valid()) {
		Transform root = skeleton->get_global_transform();
		BoneId rootBone = skeleton->get_bone_parent(limb_arm_right.get_upper_bone());
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
				Quat quatAlignToTarget = poleOffsetScaled * align_vectors(Vector3(0, 1, 0), poleOffset.inverse().xform(offsetQuat.inverse().xform(targetVector))).slerp(Quat(), 1 - shoulder_influence);
				Transform customPose = Transform(offsetQuat * quatAlignToTarget, Vector3());
				skeleton->set_bone_custom_pose(rootBone, customPose);
			}
			root = skeleton->get_global_transform() * skeleton->get_bone_global_pose(rootBone);
		}
		apply_ik_map(solve_trig_ik_redux(limb_arm_right, root, hand_right_target_spatial->get_global_transform()));
	}
}

void RenIK::perform_foot_left_ik() {
	if (skeleton && limb_leg_left.is_valid()) {
		if (foot_left_target_spatial) {
			Transform root = skeleton->get_global_transform();
			BoneId rootBone = skeleton->get_bone_parent(limb_leg_left.get_upper_bone());
			if (rootBone >= 0) {
				root = root * skeleton->get_bone_global_pose(rootBone);
			}
			apply_ik_map(solve_trig_ik_redux(limb_leg_left, root, foot_left_target_spatial->get_global_transform()));
		} else if (foot_placement) {
			Transform root = skeleton->get_global_transform();
			BoneId rootBone = skeleton->get_bone_parent(limb_leg_left.get_upper_bone());
			if (rootBone >= 0) {
				root = root * skeleton->get_bone_global_pose(rootBone);
			}
			apply_ik_map(solve_trig_ik_redux(limb_leg_left, root, placedLeft));
		}
	}
}

void RenIK::perform_foot_right_ik() {
	if (skeleton && limb_leg_right.is_valid()) {
		if (foot_right_target_spatial) {
			Transform root = skeleton->get_global_transform();
			BoneId rootBone = skeleton->get_bone_parent(limb_leg_right.get_upper_bone());
			if (rootBone >= 0) {
				root = root * skeleton->get_bone_global_pose(rootBone);
			}
			apply_ik_map(solve_trig_ik_redux(limb_leg_right, root, foot_right_target_spatial->get_global_transform()));
		} else if (foot_placement) {
			Transform root = skeleton->get_global_transform();
			BoneId rootBone = skeleton->get_bone_parent(limb_leg_right.get_upper_bone());
			if (rootBone >= 0) {
				root = root * skeleton->get_bone_global_pose(rootBone);
			}
			apply_ik_map(solve_trig_ik_redux(limb_leg_right, root, placedRight));
		}
	}
}

void RenIK::reset_chain(RenIKChain chain) {
	if (skeleton && chain.get_leaf_bone() < skeleton->get_bone_count() && chain.get_root_bone() < skeleton->get_bone_count()) {
		BoneId bone = chain.get_leaf_bone();
		while (bone >= 0 && bone != chain.get_root_bone()) {
			skeleton->set_bone_global_pose_override(bone, Transform(), 0, false);
			skeleton->set_bone_custom_pose(bone, Transform());
			bone = skeleton->get_bone_parent(bone);
		}
		if (bone >= 0) {
			skeleton->set_bone_global_pose_override(bone, Transform(), 0, false);
			skeleton->set_bone_custom_pose(bone, Transform());
		}
	}
}

void RenIK::reset_limb(RenIKLimb limb) {
	if (skeleton && limb.get_upper_bone() >= 0 && limb.get_lower_bone() >= 0 && limb.get_upper_bone() < skeleton->get_bone_count() && limb.get_lower_bone() < skeleton->get_bone_count()) {
		skeleton->set_bone_custom_pose(limb.get_upper_bone(), Transform());
		skeleton->set_bone_custom_pose(limb.get_lower_bone(), Transform());
		skeleton->set_bone_custom_pose(limb.get_leaf_bone(), Transform());
	}
}

void RenIK::hip_place(float delta, RenIKConfig config) {
}

/*foot_place requires raycasting unless a raycast result is provided.
	Raycasting needs to happen inside of a physics update
*/
void RenIK::foot_place(float delta, RenIKConfig config, Set<RID> exclude, uint32_t collision_mask, bool collide_with_bodies = true, bool collide_with_areas = false) {
	if (head_target_spatial) {
		Ref<World> w3d = head_target_spatial->get_world();
		ERR_FAIL_COND(w3d.is_null());

		PhysicsDirectSpaceState *dss = PhysicsServer::get_singleton()->space_get_direct_state(w3d->get_space());
		ERR_FAIL_COND(!dss);

		Transform headTransform = head_target_spatial->get_global_transform();

		PhysicsDirectSpaceState::RayResult left_raycast;
		PhysicsDirectSpaceState::RayResult right_raycast;
		PhysicsDirectSpaceState::RayResult laying_raycast;

		float startOffset = (config.spine_length * -config.center_of_balance_position) / sqrt(2);
		Vector3 leftStart = headTransform.translated(Vector3(0, startOffset, startOffset) + config.left_hip_offset).origin;
		Vector3 rightStart = headTransform.translated(Vector3(0, startOffset, startOffset) + config.right_hip_offset).origin;
		Vector3 leftStop = headTransform.origin + Vector3(0, (-config.spine_length - config.left_leg_length) * (1 + config.raycast_allowance) + config.left_hip_offset[1], 0) + headTransform.basis.xform(config.left_hip_offset);
		Vector3 rightStop = headTransform.origin + Vector3(0, (-config.spine_length - config.right_leg_length) * (1 + config.raycast_allowance) + config.right_hip_offset[1], 0) + headTransform.basis.xform(config.right_hip_offset);

		bool left_collided = dss->intersect_ray(leftStart, leftStop, left_raycast, exclude, collision_mask, collide_with_bodies, collide_with_areas);
		bool right_collided = dss->intersect_ray(rightStart, rightStop, right_raycast, exclude, collision_mask, collide_with_bodies, collide_with_areas);
		bool laying_down = dss->intersect_ray(headTransform.origin, headTransform.origin - Vector3(0, config.spine_length, 0), laying_raycast, exclude, collision_mask, collide_with_bodies, collide_with_areas);
		if (!left_collided) {
			left_raycast.collider = nullptr;
		}
		if (!right_collided) {
			right_raycast.collider = nullptr;
		}
		if (!laying_down) {
			laying_raycast.collider = nullptr;
		}
		foot_place(delta, config, left_raycast, right_raycast, laying_raycast);
	}
}

void RenIK::foot_place(float delta, RenIKConfig config, PhysicsDirectSpaceState::RayResult left_raycast, PhysicsDirectSpaceState::RayResult right_raycast, PhysicsDirectSpaceState::RayResult laying_raycast) {
	if (head_target_spatial) {
		Basis uprightFootBasis(Vector3(-1, 0, 0), Vector3(0, -1, 0), Vector3(0, 0, 1));
		const Spatial *newGroundLeftPointer = Object::cast_to<Spatial>(left_raycast.collider);
		const Spatial *newGroundRightPointer = Object::cast_to<Spatial>(right_raycast.collider);
		Transform centerGround = groundLeft.interpolate_with(groundRight, 0.5);
		Transform headTransform = head_target_spatial->get_global_transform();
		Vector3 newHead = centerGround.xform_inv(headTransform.origin);
		Vector3 velocity = (newHead - prevHead) / delta;
		float speed = velocity.length();
		Vector3 leftOffset = standLeft.origin - left_raycast.position;
		Vector3 rightOffset = standRight.origin - right_raycast.position;
		Vector3 axis;
		float leftRotation = 0;
		float rightRotation = 0;
		(standLeft.basis.inverse() * (head_target_spatial->get_global_transform().basis * uprightFootBasis)).get_rotation_axis_angle(axis, leftRotation);
		(standRight.basis.inverse() * (head_target_spatial->get_global_transform().basis * uprightFootBasis)).get_rotation_axis_angle(axis, rightRotation);
		//figure out if we're in the proper state
		switch (walk_state) {
			case FALLING:
				if ((newGroundLeftPointer || newGroundRightPointer) && //if at least one foot is on the ground
						!laying_raycast.collider //if we aren't too close to the ground
				) {
					walk_state = STANDING_TRANSITION;
					lastLeft = placedLeft;
					lastRight = placedRight;
				}
				break;
			case STANDING_TRANSITION:
				if (walk_transition_progress >= 1.0f) {
					walk_state = STANDING;
				} else {
				}
				break;
			case STANDING:
				//check balance & speed under threshold
				if ((!newGroundLeftPointer && !newGroundRightPointer) || //if both feet are off the ground
						laying_raycast.collider //if too close to the ground
				) {
					walk_state = FALLING; //start free falling
				} else if (leftOffset.length_squared() > config.balance_threshold * config.left_leg_length || //if the left foot is too far away from where it should be
						   rightOffset.length_squared() > config.balance_threshold * config.right_leg_length || //if the right foot is too far away from where it should be
						   (leftOffset + rightOffset).length_squared() > config.balance_threshold * config.spine_length || //if we're off balance
						   leftRotation > config.rotation_threshold || //if the left foot is twisted
						   rightRotation > config.rotation_threshold || //if the right foot is twisted
						   newGroundLeftPointer != groundLeftPointer || newGroundRightPointer != newGroundRightPointer // if the ground object disappears or changes suddenly
				) {
					walk_state = STANDING_TRANSITION;
					//check if the left foot was the one that caused the step
					if (leftOffset.length_squared() > config.balance_threshold * config.left_leg_length ||
							leftRotation > config.rotation_threshold ||
							newGroundLeftPointer != groundLeftPointer) {
						step_progress = 0;
					} else {
						step_progress = 0.5;
					};
					lastLeft = placedLeft;
					lastRight = placedRight;
				}
				break;
			case STEPPING_TRANSITION:
				if (walk_transition_progress >= 1.0f) {
					walk_state = STEPPING;
				} else {

				}
				break;
			case STEPPING:
				//check speed above threshold
				if ((!newGroundLeftPointer && !newGroundRightPointer) || //if both feet are off the ground
						laying_raycast.collider //if too close to the ground
				) {
					walk_state = LAYING_TRANSITION;
				} else if (speed < config.min_threshold * config.left_leg_length) { //moving too slow
					walk_state = STANDING_TRANSITION;
				}
				break;
			case LAYING_TRANSITION:
				if (walk_transition_progress >= 1.0f) {
					walk_state = LAYING;
				} else {
				}
				break;
			case LAYING:
				break;
			case OTHER_TRANSITION:
				if (walk_transition_progress >= 1.0f) {
					walk_state = OTHER;
				} else {
				}
				break;
			case OTHER:
				break;
		}
		if(walk_state < 0){//if we're in a transition
			walk_transition_progress += delta;
		} else {
			walk_transition_progress = 0;
		}
		prevHead = newHead;
		groundLeftPointer = newGroundLeftPointer;
		groundRightPointer = newGroundRightPointer;

		//for dangling
		Vector3 hipDangle(0, config.spine_length * (config.dangle_height - 1), 0);
		Vector3 legDangle;
		if (laying_raycast.collider) {
			Vector3 groundNormal = laying_raycast.normal;
			legDangle = headTransform.basis.xform_inv(vector_rejection(headTransform.basis.xform(Vector3(0, -1, 0)), groundNormal).normalized()) * config.left_leg_length;
		} else {
			Vector3 groundNormal = newGroundLeftPointer || newGroundRightPointer ? left_raycast.normal.linear_interpolate(right_raycast.normal, 0.5).normalized() : Vector3(0, 1, 0);
			if (headTransform.basis.xform(Vector3(0, 1, 0)).angle_to(groundNormal) > Math_PI / 3) { //if head is at an extreme angle, just have the feet follow the head
				legDangle = Vector3(0, -config.left_leg_length, 0);
			} else { //otherwise if the head is mostly vertical, try to point the feet at the ground
				legDangle = headTransform.basis.xform_inv(groundNormal * config.left_leg_length * (config.dangle_height - 1));
			}
		}
		Vector3 leftDangleVector = hipDangle + config.left_hip_offset + legDangle;
		Vector3 rightDangleVector = hipDangle + config.right_hip_offset + legDangle;
		Basis uprightRotatedFoot = headTransform.basis * uprightFootBasis;
		Quat pointFeetToHead = align_vectors(hipDangle + legDangle, Vector3(0, -1, 0));
		Transform dangleLeft = headTransform * Transform(uprightFootBasis * pointFeetToHead, leftDangleVector);
		Transform dangleRight = headTransform * Transform(uprightFootBasis * pointFeetToHead, rightDangleVector);

		switch (walk_state) {
			case STANDING: //standing state
			case STANDING_TRANSITION: //transitioning to standing state
				if (groundLeftPointer) {
					standLeft = groundLeft * groundedLeft;
					placedLeft = walk_state == STANDING ? standLeft : placedLeft.interpolate_with(standLeft, config.dangle_stiffness);
				} else {
					placedLeft = placedLeft.interpolate_with(dangleLeft, config.dangle_stiffness);
				}
				if (groundRightPointer) {
					standRight = groundRight * groundedRight;
					placedRight = walk_state == STANDING ? standRight : placedRight.interpolate_with(standRight, config.dangle_stiffness);
				} else {
					placedRight = placedRight.interpolate_with(dangleRight, config.dangle_stiffness);
				}
				break;
				// if (groundLeftPointer) {
				// 	standLeft = leftRotation > config.rotation_threshold ? Transform(uprightFootBasis * pointFeetToHead, left_raycast.position) : groundLeft * groundedLeft;
				// 	placedLeft = placedLeft.interpolate_with(standLeft, config.dangle_stiffness);
				// } else {
				// 	placedLeft = placedLeft.interpolate_with(dangleLeft, config.dangle_stiffness);
				// }
				// if (groundRightPointer) {
				// 	standRight = rightRotation > config.rotation_threshold ? Transform(uprightFootBasis * pointFeetToHead, right_raycast.position) : groundRight * groundedRight;
				// 	placedRight = placedRight.interpolate_with(standRight, config.dangle_stiffness);
				// } else {
				// 	placedRight = placedRight.interpolate_with(dangleRight, config.dangle_stiffness);
				// }
				// break;
			case STEPPING: //stepping state
			case STEPPING_TRANSITION:
				if (groundLeftPointer || groundRightPointer) {
					//if only one foot has a hold, pretend we're tightrope walking
					groundLeftPointer = groundLeftPointer ? groundLeftPointer : groundRightPointer;
					groundRightPointer = groundRightPointer ? groundRightPointer : groundLeftPointer;
					Vector3 leftPos = groundLeftPointer ? left_raycast.position : right_raycast.position;
					Vector3 leftNormal = groundLeftPointer ? left_raycast.normal .normalized(): right_raycast.normal.normalized();
					Vector3 rightPos = groundRightPointer ? right_raycast.position : left_raycast.position;
					Vector3 rightNormal = groundRightPointer ? right_raycast.normal.normalized() : left_raycast.normal.normalized();

					Quat rotationLeft = leftRotation > config.rotation_threshold ? pointFeetToHead : align_vectors(leftNormal, uprightRotatedFoot.xform_inv(Vector3(0, -1, 0)));
					Quat rotationRight = rightRotation > config.rotation_threshold ? pointFeetToHead : align_vectors(rightNormal, uprightRotatedFoot.xform_inv(Vector3(0, -1, 0)));

					// step_progress = step_progress + speed * speed > 1.0 ? 0.5 : step_progress + speed * speed;
					step_progress = fmod((step_progress + sqrtf(speed)) , 1.0); //0 is left foot at apex, 0.5 is right foot at apex
					float right_step_progress = fmod((step_progress + 0.5) , 1.0);
					float totalContact = (config.contact_length / 2) * MIN(0, 1 - config.contact_scale * sqrt(speed / config.max_threshold));
					float bottomSegmentSize = 0.5 - totalContact / 2; //size of followthru and buildup. starts at 0 for slow speeds
					float topSegmentSize = 0.25; //size of the top halves of the loop
					//
					//ideal is where we want the foot to land. At slow speeds its a little in front of where we're headed. At high speeds it's directly below the center of gravity.
					Transform idealLeft(uprightRotatedFoot * rotationLeft, leftPos + (vector_rejection(velocity, leftNormal).normalized() * totalContact));
					Transform idealRight(uprightRotatedFoot * rotationLeft, rightPos + (vector_rejection(velocity, rightNormal).normalized() * totalContact));
					placedLeft.set_basis(placedLeft.basis.slerp(idealLeft.basis, config.dangle_stiffness)); //always lags a little
					placedRight.set_basis(placedRight.basis.slerp(idealLeft.basis, config.dangle_stiffness)); //always lags a little
					Vector3 saddleLeft = (leftPos - headTransform.origin) * (1 - config.step_height - config.height_scale * speed * speed);
					Vector3 saddleRight = (rightPos - headTransform.origin) * (1 - config.step_height - config.height_scale * speed * speed);
					float followScale = config.loop_scale * speed * config.loop_ratio;
					float buildScale = config.loop_scale * speed * (1 - config.loop_ratio);
					Vector3 leftFollow = leftNormal * config.followthru_angle - velocity.normalized();
					Vector3 rightFollow = rightNormal * config.followthru_angle - velocity.normalized();
					Vector3 leftBuild = leftNormal * config.buildup_angle + velocity.normalized();
					Vector3 rightBuild = rightNormal * config.buildup_angle + velocity.normalized();
					//left foot
					if (step_progress < topSegmentSize) { //starts at end of the followthru, ends at saddle
						Vector3 startPos = headTransform.origin + lastLeft.origin + leftFollow * followScale;
						Vector3 endPos = saddleLeft;
						Quat startRot = lastLeft.basis.get_rotation_quat() * Quat(Vector3(1, 0, 0), followScale * Math_PI / 2); //rotated down 90 degrees
						Quat endRot = idealLeft.basis.get_rotation_quat() * Quat(Vector3(1, 0, 0), Math_PI / 4);//rotated down 45 degrees
						float segmentProgress = step_progress / topSegmentSize;
						Vector3 posDiff = endPos - startPos;
						Vector3 startCurve = startPos + leftNormal * (posDiff.length() / 4);
						Vector3 endCurve = endPos - vector_rejection(posDiff, leftNormal).normalized() * (posDiff.length() / 4);
						placedLeft.set_basis(startRot.slerp(endRot, segmentProgress));
						placedLeft.set_origin(startPos.cubic_interpolate(endPos, startCurve, endCurve, segmentProgress));
					} else if (step_progress < 2 * topSegmentSize) { //starts at saddle, ends at apex
						Vector3 startPos = saddleLeft;
						Vector3 endPos = headTransform.origin + idealLeft.origin + leftBuild * buildScale;
						Quat startRot = idealLeft.basis.get_rotation_quat() * Quat(Vector3(1, 0, 0), Math_PI / 4);
						Quat endRot = idealLeft.basis.get_rotation_quat() * Quat(Vector3(-1, 0, 0), buildScale * Math_PI / 4); //rotated up 45 degrees
						float segmentProgress = (step_progress - topSegmentSize) / topSegmentSize;
						Vector3 posDiff = endPos - startPos;
						Vector3 startCurve = startPos + vector_rejection(posDiff, leftNormal).normalized() * (posDiff.length() / 4);
						Vector3 endCurve = endPos - leftNormal * (posDiff.length() / 4);
						placedLeft.set_basis(startRot.slerp(endRot, segmentProgress));
						placedLeft.set_origin(startPos.cubic_interpolate(endPos, startCurve, endCurve, segmentProgress));
					} else if (step_progress < 2 * topSegmentSize + bottomSegmentSize) { //starts at apex, goes to landing point
						Vector3 startPos = headTransform.origin + idealLeft.origin + leftBuild * buildScale;
						Vector3 endPos = idealLeft.origin;
						Quat startRot = idealLeft.basis.get_rotation_quat() * Quat(Vector3(-1, 0, 0), buildScale * Math_PI / 4);
						Quat endRot = idealLeft.basis;
						float segmentProgress = (step_progress - topSegmentSize) / topSegmentSize;
						Vector3 posDiff = endPos - startPos;
						Vector3 startCurve = startPos - leftNormal * (posDiff.length() / 4);
						Vector3 endCurve = endPos;
						placedLeft.set_basis(startRot.slerp(endRot, segmentProgress));
						placedLeft.set_origin(startPos.cubic_interpolate(endPos, startCurve, endCurve, segmentProgress));
						//needed for the next step
						groundLeft = groundLeftPointer->get_global_transform();
						groundedLeft = groundLeft.affine_inverse() * idealLeft;
					} else if (step_progress < 2 * topSegmentSize + bottomSegmentSize + totalContact) { //lands and stays in contact with the ground
						groundLeft = groundLeftPointer->get_global_transform();
						placedLeft = groundLeft * groundedLeft;
					} else { //lifts back up during the follow thru
						float segmentProgress = (step_progress - 2 * topSegmentSize - bottomSegmentSize - totalContact) / bottomSegmentSize;
					}
				}
				break;
			case FALLING: //jumping / falling
				placedLeft = placedLeft.interpolate_with(dangleLeft, config.dangle_stiffness);
				placedRight = placedRight.interpolate_with(dangleRight, config.dangle_stiffness);
				break;
			case LAYING:
			case LAYING_TRANSITION:
				placedLeft = placedLeft.interpolate_with(dangleLeft, config.dangle_stiffness);
				placedRight = placedRight.interpolate_with(dangleRight, config.dangle_stiffness);
				break;
		}
	}
}

void RenIK::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
}

uint32_t RenIK::get_collision_mask() const {
	return collision_mask;
}

void RenIK::set_collision_mask_bit(int p_bit, bool p_value) {
	uint32_t mask = get_collision_mask();
	if (p_value)
		mask |= 1 << p_bit;
	else
		mask &= ~(1 << p_bit);
	set_collision_mask(mask);
}

bool RenIK::get_collision_mask_bit(int p_bit) const {
	return get_collision_mask() & (1 << p_bit);
}

void RenIK::set_collide_with_areas(bool p_clip) {

	collide_with_areas = p_clip;
}

bool RenIK::is_collide_with_areas_enabled() const {

	return collide_with_areas;
}

void RenIK::set_collide_with_bodies(bool p_clip) {

	collide_with_bodies = p_clip;
}

bool RenIK::is_collide_with_bodies_enabled() const {

	return collide_with_bodies;
}

//IK SOLVING

float RenIK::safe_acos(float f) {
	if (f > 1) {
		f = 1;
	} else if (f < -1) {
		f = -1;
	}
	return acos(f);
}
float RenIK::safe_asin(float f) {
	if (f > 1) {
		f = 1;
	} else if (f < -1) {
		f = -1;
	}
	return asin(f);
}

Vector3 RenIK::get_perpendicular_vector(Vector3 v) {
	Vector3 perpendicular;
	if (v[0] != 0 && v[1] != 0) {
		perpendicular = Vector3(0, 0, 1).cross(v).normalized();
	} else {
		perpendicular = Vector3(1, 0, 0);
	}
	return perpendicular;
}

Vector3 RenIK::vector_rejection(Vector3 v, Vector3 normal) {
	float normalLength = normal.length();
	Vector3 proj = (normal.dot(v) / normalLength) * (normal / normalLength);
	return v - proj;
}

Quat RenIK::align_vectors(Vector3 a, Vector3 b, float influence) {
	a.normalize();
	b.normalize();
	if (a.length_squared() != 0 && b.length_squared() != 0) {
		//Find the axis perpendicular to both vectors and rotate along it by the angular difference
		Vector3 perpendicular = a.cross(b).normalized();
		float angleDiff = a.angle_to(b) * influence;
		if (perpendicular.length_squared() == 0) {
			perpendicular = get_perpendicular_vector(a);
		}
		return Quat(perpendicular, angleDiff);
	} else {
		return Quat();
	}
}

Map<BoneId, Quat> RenIK::solve_trig_ik(RenIKLimb limb, Transform root, Transform target) {
	Map<BoneId, Quat> map;

	if (limb.is_valid()) { //There's no way to find a valid upperId if any of the other Id's are invalid, so we only check upperId
		Vector3 upperVector = limb.get_lower().get_origin();
		Vector3 lowerVector = limb.get_leaf().get_origin();
		Quat upperRest = limb.get_upper().get_basis().get_rotation_quat();
		Quat lowerRest = limb.get_lower().get_basis().get_rotation_quat();
		Quat upper = upperRest.inverse();
		Quat lower = lowerRest.inverse();
		//The true root of the limb is the point where the upper bone starts
		Transform trueRoot = root.translated(limb.get_upper().get_origin());
		Transform diff = root.affine_inverse() * trueRoot;
		Transform localTarget = trueRoot.affine_inverse() * target;

		//First we offset the pole
		upper = upper * limb.pole_offset.normalized(); //pole_offset is a euler because that's more human readable
		upper.normalize();
		lower.normalize();
		//Then we line up the limb with our target
		Vector3 targetVector = limb.pole_offset.inverse().xform(localTarget.get_origin());
		upper = upper * align_vectors(upperVector, targetVector);
		//Then we calculate how much we need to bend so we don't extend past the target
		//Law of Cosines
		float upperLength = upperVector.length();
		float lowerLength = lowerVector.length();
		float upperLength2 = upperVector.length_squared();
		float lowerLength2 = lowerVector.length_squared();
		float targetDistance = targetVector.length();
		float targetDistance2 = targetVector.length_squared();
		float upperAngle = safe_acos((upperLength2 + targetDistance2 - lowerLength2) / (2 * upperLength * targetDistance));
		float lowerAngle = safe_acos((upperLength2 + lowerLength2 - targetDistance2) / (2 * upperLength * lowerLength)) - Math_PI;
		Vector3 bendAxis = get_perpendicular_vector(upperVector); //TODO figure out how to set this automatically to the right axis
		Quat upperBend = Quat(bendAxis, upperAngle);
		Quat lowerBend = Quat(bendAxis, lowerAngle);
		upper = upper * upperBend;
		lower = lower * lowerBend;
		//Then we roll the limb based on the target position
		Vector3 targetRestPosition = upperVector.normalized() * (upperLength + lowerLength);
		Vector3 rollVector = upperBend.inverse().xform(upperVector).normalized();
		float positionalRollAmount = limb.target_position_influence.dot(targetRestPosition - targetVector);
		Quat positionRoll = Quat(rollVector, positionalRollAmount);
		upper = upper.normalized() * positionRoll;
		//And the target rotation

		Quat leafRest = limb.get_leaf().get_basis().get_rotation_quat();
		Quat armCombined = (upperRest * upper * lowerRest * lower).normalized();
		Quat targetQuat = localTarget.get_basis().get_rotation_quat() * leafRest;
		Quat leaf = ((armCombined * leafRest).inverse() * targetQuat).normalized();
		// if we had a plane along the roll vector we can project the leaf and lower limb on it to see which direction we need to roll to reduce the angle between the two
		Vector3 restVector = (armCombined).xform(lowerVector).normalized();
		Vector3 leafVector = leaf.xform(restVector).normalized();
		Vector3 restRejection = vector_rejection(restVector.normalized(), rollVector);
		Vector3 leafRejection = vector_rejection(leafVector.normalized(), rollVector);
		float directionalRollAmount = safe_acos(restRejection.normalized().dot(leafRejection.normalized())) * limb.target_rotation_influence;
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
		// old way: Quat lowerTwist = (align_vectors(lowerVector, leafRest.xform(leaf.xform(lowerVector))).inverse() * (leafRest * leaf)).slerp(Quat(), 1 - limb.lower_limb_twist).normalized();
		Vector3 twist = (leafRest * leaf).get_euler();
		Quat lowerTwist = Quat((leafRest * leaf).get_euler() * lowerVector.normalized() * (limb.lower_limb_twist));
		lower = lower * lowerTwist;
		leaf = (lowerTwist * leafRest).inverse() * leafRest * leaf;

		Quat upperTwist = Quat(twist * upperVector.normalized() * (limb.upper_limb_twist * limb.lower_limb_twist));
		upper = upper * upperTwist;
		lower = (upperTwist * lowerRest).inverse() * lowerRest * lower;

		//save data and return
		map.insert(limb.get_upper_bone(), upper);
		map.insert(limb.get_lower_bone(), lower);
		map.insert(limb.get_leaf_bone(), leaf);
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
	float angle1 = safe_acos((length1Squared + length3Squared - length2Squared) / (length1 * length3));
	float angle2 = Math_PI - safe_acos((length1Squared + length2Squared - length3Squared) / (length1 * length2));
	return std::make_pair(angle1, angle2);
}

//random helper function
float RenIK::smoothCurve(float number, float modifier) {
	return number / (abs(number) + modifier);
}

Map<BoneId, Basis> RenIK::solve_trig_ik_redux(RenIKLimb &limb, Transform root, Transform target) {
	Map<BoneId, Basis> map;
	if (limb.is_valid()) {
		//The true root of the limb is the point where the upper bone starts
		Transform trueRoot = root.translated(limb.get_upper().get_origin());
		Transform localTarget = trueRoot.affine_inverse() * target;

		//The Triangle
		Vector3 upperVector = limb.get_lower().get_origin();
		Vector3 lowerVector = limb.get_leaf().get_origin();
		Vector3 targetVector = localTarget.get_origin();
		Vector3 normalizedTargetVector = targetVector.normalized();
		float limbLength = upperVector.length() + lowerVector.length();
		if (targetVector.length() > upperVector.length() + lowerVector.length()) {
			targetVector = normalizedTargetVector * limbLength;
		}
		std::pair<float, float> angles = trig_angles(upperVector, lowerVector, targetVector);

		//The local x-axis of the upper limb is axis along which the limb will bend
		//We take into account how the pole offset and alignment with the target vector will affect this axis
		Vector3 startingPole = limb.pole_offset.xform(Vector3(0, 1, 0)); //the opposite of this vector is where the pole is
		Vector3 jointAxis = align_vectors(startingPole, targetVector).xform(limb.pole_offset.xform(Vector3(1, 0, 0)));

		// //We then find how far away from the rest position the leaf is and use that to change the rotational axis more.
		Vector3 leafRestVector = limb.get_upper().get_basis().xform(limb.get_lower().xform(limb.get_leaf().get_origin()));
		float positionalOffset = limb.target_position_influence.dot(targetVector - leafRestVector);
		jointAxis.rotate(normalizedTargetVector, positionalOffset + limb.roll_offset);

		//Leaf Rotations... here we go...
		//Let's always try to avoid having the leaf intersect the lowerlimb
		//First we find the a vector that corresponds with the direction the leaf and lower limbs are pointing local to the true root
		Vector3 localLeafVector = localTarget.get_basis().xform(Vector3(0, 1, 0)); //y axis of the target
		Vector3 localLowerVector = normalizedTargetVector.rotated(jointAxis, angles.first - angles.second).normalized();
		//We then take the vector rejections of the leaf and lower limb against the target vector
		//A rejection is the opposite of a projection. We use the target vector because that's our axis of rotation for the whole limb.
		//We then turn the whole arm along the target vector based on how close the rejections are
		//We scale the amount we rotate with the rotation influence setting and the angle between the leaf and lower vector so if the arm is mostly straight, we rotate less
		Vector3 leafRejection = vector_rejection(localLeafVector, normalizedTargetVector);
		Vector3 lowerRejection = vector_rejection(localLowerVector, normalizedTargetVector);
		float jointRollAmount = (leafRejection.angle_to(lowerRejection)) * limb.target_rotation_influence;
		jointRollAmount *= abs(localLeafVector.cross(localLowerVector).dot(normalizedTargetVector));
		if (leafRejection.cross(lowerRejection).dot(normalizedTargetVector) > 0) {
			jointRollAmount *= -1;
		}
		jointAxis.rotate(normalizedTargetVector, jointRollAmount);
		float totalRoll = jointRollAmount + positionalOffset + limb.roll_offset;

		//Add a little twist
		//We align the leaf's y axis with the lower limb's y-axis and see how far off the x-axis is from the joint axis to calculate the twist.
		Vector3 leafX = align_vectors(localLeafVector.rotated(normalizedTargetVector, jointRollAmount), localLowerVector.rotated(normalizedTargetVector, jointRollAmount)).xform(localTarget.get_basis().xform(Vector3(1, 0, 0)));
		Vector3 rolledJointAxis = jointAxis.rotated(localLowerVector, -totalRoll);
		Vector3 lowerZ = rolledJointAxis.cross(localLowerVector);
		float twistAngle = leafX.angle_to(rolledJointAxis);
		if (leafX.dot(lowerZ) > 0) {
			twistAngle *= -1;
		}

		float inflectionPoint = twistAngle > 0 ? Math_PI - limb.twist_inflection_point_offset : -Math_PI - limb.twist_inflection_point_offset;
		float overflowArea = limb.overflow_state * limb.twist_overflow;
		float inflectionDistance = twistAngle - inflectionPoint;

		if (Math::abs(inflectionDistance) < limb.twist_overflow) {
			if (limb.overflow_state == 0) {
				limb.overflow_state = inflectionDistance < 0 ? 1 : -1;
			}
		} else {
			limb.overflow_state = 0;
		}

		inflectionPoint += overflowArea;
		if (twistAngle > 0 && twistAngle > inflectionPoint) {
			twistAngle -= Math_TAU; //Change to complement angle
		} else if (twistAngle < 0 && twistAngle < inflectionPoint) {
			twistAngle += Math_TAU; //Change to complement angle
		}

		float lowerTwist = twistAngle * limb.lower_limb_twist;
		float upperTwist = lowerTwist * limb.upper_limb_twist + limb.upper_twist_offset - totalRoll;
		lowerTwist += limb.lower_twist_offset - 2 * limb.roll_offset - positionalOffset - jointRollAmount;

		jointAxis.rotate(normalizedTargetVector, twistAngle * limb.target_rotation_influence);

		//Rebuild the rotations
		Vector3 upperJointVector = normalizedTargetVector.rotated(jointAxis, angles.first);
		Vector3 rolledLowerJointAxis = Vector3(1, 0, 0).rotated(Vector3(0, 1, 0), -limb.roll_offset);
		Vector3 lowerJointVector = Vector3(0, 1, 0).rotated(rolledLowerJointAxis, angles.second);
		Vector3 twistedJointAxis = jointAxis.rotated(upperJointVector, upperTwist);
		Basis upperBasis = Basis(twistedJointAxis, upperJointVector, twistedJointAxis.cross(upperJointVector)).inverse();
		Basis lowerBasis = Basis(rolledLowerJointAxis, lowerJointVector, rolledLowerJointAxis.cross(lowerJointVector));
		lowerBasis.rotate_local(Vector3(0, 1, 0), lowerTwist);
		lowerBasis.rotate(Vector3(0, 1, 0), -upperTwist);

		map[limb.get_upper_bone()] = limb.get_upper().get_basis().inverse() * upperBasis;
		map[limb.get_lower_bone()] = limb.get_lower().get_basis().inverse() * lowerBasis;
		map[limb.get_leaf_bone()] = limb.get_leaf().get_basis().inverse() * (upperBasis * lowerBasis).inverse() * localTarget.get_basis() * limb.get_leaf().get_basis();
	}
	return map;
}

Map<BoneId, Quat> RenIK::solve_ifabrik(RenIKChain chain, Transform root, Transform target, float threshold, int loopLimit) {
	Map<BoneId, Quat> map;
	if (chain.is_valid()) { //if the chain is valid there's at least one joint in the chain and there's one bone between it and the root
		Vector<RenIKChain::Joint> joints = chain.get_joints(); //just so I don't have to call it all the time
		Transform trueRoot = root.translated(joints[0].relative_prev);
		Transform targetDelta = target * chain.get_relative_rest_leaf().affine_inverse(); //how the change in the target would affect the chain if the chain was parented to the target instead of the root
		Transform trueRelativeTarget = trueRoot.affine_inverse() * target;
		Quat alignToTarget = align_vectors(chain.get_relative_rest_leaf().origin - joints[0].relative_prev, trueRelativeTarget.origin);
		float heightDiff = (chain.get_relative_rest_leaf().origin - joints[0].relative_prev).length() - trueRelativeTarget.origin.length();
		heightDiff = heightDiff < 0 ? 0 : heightDiff;
		Transform prebentRoot = Transform(trueRoot.basis * alignToTarget, trueRoot.origin).translated((chain.chain_curve_direction * chain.get_total_length() * heightDiff) - joints[0].relative_prev); //The angle root is rotated to point at the target;

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
		Vector3 leafX = align_vectors(trueRelativeTarget.basis.xform(Vector3(0, 1, 0)), chain.get_relative_rest_leaf().basis.xform(Vector3(0, 1, 0))).normalized().xform(trueRelativeTarget.basis.xform(Vector3(1, 0, 0)));
		Vector3 restX = chain.get_relative_rest_leaf().basis.xform(Vector3(1, 0, 0));
		Vector3 restZ = chain.get_relative_rest_leaf().basis.xform(Vector3(0, 0, 1));
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
			Quat pose = align_vectors(Vector3(0, 1, 0), Transform(parentRot * joints[i].rotation, parentPos).affine_inverse().xform(globalJointPoints[i])); //offset by one because joints has one extra element
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
				skeleton->set_bone_custom_pose(skeleton->get_bone_parent(limb_arm_left.get_upper_bone()), Transform());
			}
		}
		if (hand_right_target_spatial) {
			reset_limb(limb_arm_right);
			if (skeleton && right_shoulder_enabled) {
				skeleton->set_bone_custom_pose(skeleton->get_bone_parent(limb_arm_right.get_upper_bone()), Transform());
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
	spine_chain.set_leaf_bone(skeleton, p_bone);
	config.spine_length = spine_chain.get_total_length();
}
void RenIK::set_hand_left_bone(BoneId p_bone) {
	limb_arm_left.set_leaf(skeleton, p_bone);
	left_shoulder_enabled = skeleton && limb_arm_left.is_valid() && !spine_chain.contains_bone(skeleton, skeleton->get_bone_parent(limb_arm_left.get_lower_bone()));
}

void RenIK::set_hand_right_bone(BoneId p_bone) {
	limb_arm_right.set_leaf(skeleton, p_bone);
	right_shoulder_enabled = skeleton && limb_arm_right.is_valid() && !spine_chain.contains_bone(skeleton, skeleton->get_bone_parent(limb_arm_right.get_lower_bone()));
}
void RenIK::set_hip_bone(BoneId p_bone) {
	hip = p_bone;
	spine_chain.set_root_bone(skeleton, p_bone);
	config.spine_length = spine_chain.get_total_length();
}
void RenIK::set_foot_left_bone(BoneId p_bone) {
	limb_leg_left.set_leaf(skeleton, p_bone);
	config.left_leg_length = limb_leg_left.is_valid() ? limb_leg_left.lower.origin.length() + limb_leg_left.leaf.origin.length() : 0;
	config.left_hip_offset = limb_leg_left.is_valid() ? limb_leg_left.upper.origin : Vector3();
}
void RenIK::set_foot_right_bone(BoneId p_bone) {
	limb_leg_right.set_leaf(skeleton, p_bone);
	config.right_leg_length = limb_leg_right.is_valid() ? limb_leg_right.lower.origin.length() + limb_leg_right.leaf.origin.length() : 0;
	config.right_hip_offset = limb_leg_right.is_valid() ? limb_leg_right.upper.origin : Vector3();
}

int64_t RenIK::get_hip_bone() {
	return hip;
}
int64_t RenIK::get_head_bone() {
	return head;
}
int64_t RenIK::get_hand_left_bone() {
	return limb_arm_left.get_leaf_bone();
}
int64_t RenIK::get_hand_right_bone() {
	return limb_arm_right.get_leaf_bone();
}
int64_t RenIK::get_foot_left_bone() {
	return limb_leg_left.get_leaf_bone();
}
int64_t RenIK::get_foot_right_bone() {
	return limb_leg_right.get_leaf_bone();
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
	return Math::rad2deg(limb_arm_left.upper_twist_offset);
}
void RenIK::set_arm_upper_twist_offset(float degrees) {
	limb_arm_left.upper_twist_offset = Math::deg2rad(degrees);
	limb_arm_right.upper_twist_offset = Math::deg2rad(-degrees);
}
float RenIK::get_arm_lower_twist_offset() {
	return Math::rad2deg(limb_arm_left.lower_twist_offset);
}
void RenIK::set_arm_lower_twist_offset(float degrees) {
	limb_arm_left.lower_twist_offset = Math::deg2rad(degrees);
	limb_arm_right.lower_twist_offset = Math::deg2rad(-degrees);
}
float RenIK::get_arm_roll_offset() {
	return Math::rad2deg(limb_arm_left.roll_offset);
}
void RenIK::set_arm_roll_offset(float degrees) {
	limb_arm_left.roll_offset = Math::deg2rad(degrees);
	limb_arm_right.roll_offset = Math::deg2rad(-degrees);
}
float RenIK::get_arm_upper_limb_twist() {
	return limb_arm_left.upper_limb_twist * 100;
}
void RenIK::set_arm_upper_limb_twist(float ratio) {
	limb_arm_left.upper_limb_twist = ratio / 100.0;
	limb_arm_right.upper_limb_twist = ratio / 100.0;
}
float RenIK::get_arm_lower_limb_twist() {
	return limb_arm_left.lower_limb_twist * 100;
}
void RenIK::set_arm_lower_limb_twist(float ratio) {
	limb_arm_left.lower_limb_twist = ratio / 100.0;
	limb_arm_right.lower_limb_twist = ratio / 100.0;
}
float RenIK::get_arm_twist_inflection_point_offset() {
	return Math::rad2deg(limb_arm_left.twist_inflection_point_offset);
}
void RenIK::set_arm_twist_inflection_point_offset(float degrees) {
	limb_arm_left.twist_inflection_point_offset = Math::deg2rad(degrees);
	limb_arm_right.twist_inflection_point_offset = Math::deg2rad(-degrees);
}
float RenIK::get_arm_twist_overflow() {
	return Math::rad2deg(limb_arm_left.twist_overflow);
}
void RenIK::set_arm_twist_overflow(float degrees) {
	limb_arm_left.twist_overflow = Math::deg2rad(degrees);
	limb_arm_right.twist_overflow = Math::deg2rad(degrees);
}

Vector3 RenIK::get_arm_pole_offset() {
	Vector3 v = limb_arm_left.pole_offset.get_euler();
	return Vector3(Math::rad2deg(v[0]), Math::rad2deg(v[1]), Math::rad2deg(v[2]));
}
void RenIK::set_arm_pole_offset(Vector3 euler) {
	Quat q = Quat(Vector3(Math::deg2rad(euler[0]), Math::deg2rad(euler[1]), Math::deg2rad(euler[2])));
	Quat q2 = Quat(Vector3(Math::deg2rad(euler[0]), Math::deg2rad(-euler[1]), Math::deg2rad(-euler[2])));
	limb_arm_left.pole_offset = q;
	limb_arm_right.pole_offset = q2;
}
Vector3 RenIK::get_arm_target_position_influence() {
	return limb_arm_left.target_position_influence * 10.0;
}
void RenIK::set_arm_target_position_influence(Vector3 xyz) {
	limb_arm_left.target_position_influence = xyz / 10.0;
	limb_arm_right.target_position_influence = Vector3(xyz[0], -xyz[1], -xyz[2]) / 10.0;
}
float RenIK::get_arm_target_rotation_influence() {
	return limb_arm_left.target_rotation_influence * 100.0;
}
void RenIK::set_arm_target_rotation_influence(float influence) {
	limb_arm_left.target_rotation_influence = influence / 100.0;
	limb_arm_right.target_rotation_influence = influence / 100.0;
}

float RenIK::get_leg_upper_twist_offset() {
	return Math::rad2deg(limb_leg_left.upper_twist_offset);
}
void RenIK::set_leg_upper_twist_offset(float degrees) {
	limb_leg_left.upper_twist_offset = Math::deg2rad(degrees);
	limb_leg_right.upper_twist_offset = Math::deg2rad(-degrees);
}
float RenIK::get_leg_lower_twist_offset() {
	return Math::rad2deg(limb_leg_left.lower_twist_offset);
}
void RenIK::set_leg_lower_twist_offset(float degrees) {
	limb_leg_left.lower_twist_offset = Math::deg2rad(degrees);
	limb_leg_right.lower_twist_offset = Math::deg2rad(-degrees);
}
float RenIK::get_leg_roll_offset() {
	return Math::rad2deg(limb_leg_left.roll_offset);
}
void RenIK::set_leg_roll_offset(float degrees) {
	limb_leg_left.roll_offset = Math::deg2rad(degrees);
	limb_leg_right.roll_offset = Math::deg2rad(-degrees);
}
float RenIK::get_leg_upper_limb_twist() {
	return limb_leg_left.upper_limb_twist * 100;
}
void RenIK::set_leg_upper_limb_twist(float ratio) {
	limb_leg_left.upper_limb_twist = ratio / 100.0;
	limb_leg_right.upper_limb_twist = ratio / 100.0;
}
float RenIK::get_leg_lower_limb_twist() {
	return limb_leg_left.lower_limb_twist * 100;
}
void RenIK::set_leg_lower_limb_twist(float ratio) {
	limb_leg_left.lower_limb_twist = ratio / 100.0;
	limb_leg_right.lower_limb_twist = ratio / 100.0;
}
float RenIK::get_leg_twist_inflection_point_offset() {
	return Math::rad2deg(limb_leg_left.twist_inflection_point_offset);
}
void RenIK::set_leg_twist_inflection_point_offset(float degrees) {
	limb_leg_left.twist_inflection_point_offset = Math::deg2rad(degrees);
	limb_leg_right.twist_inflection_point_offset = Math::deg2rad(-degrees);
}
float RenIK::get_leg_twist_overflow() {
	return Math::rad2deg(limb_leg_left.twist_overflow);
}
void RenIK::set_leg_twist_overflow(float degrees) {
	limb_leg_left.twist_overflow = Math::deg2rad(degrees);
	limb_leg_right.twist_overflow = Math::deg2rad(degrees);
}

Vector3 RenIK::get_leg_pole_offset() {
	Vector3 v = limb_leg_left.pole_offset.get_euler();
	return Vector3(Math::rad2deg(v[0]), Math::rad2deg(v[1]), Math::rad2deg(v[2]));
}
void RenIK::set_leg_pole_offset(Vector3 euler) {
	Quat q = Quat(Vector3(Math::deg2rad(euler[0]), Math::deg2rad(euler[1]), Math::deg2rad(euler[2])));
	Quat q2 = Quat(Vector3(Math::deg2rad(euler[0]), Math::deg2rad(-euler[1]), Math::deg2rad(-euler[2])));
	limb_leg_left.pole_offset = q;
	limb_leg_right.pole_offset = q2;
}
Vector3 RenIK::get_leg_target_position_influence() {
	return limb_leg_left.target_position_influence * 10.0;
}
void RenIK::set_leg_target_position_influence(Vector3 xyz) {
	limb_leg_left.target_position_influence = xyz / 10.0;
	limb_leg_right.target_position_influence = Vector3(xyz[0], -xyz[1], -xyz[2]) / 10.0;
}
float RenIK::get_leg_target_rotation_influence() {
	return limb_leg_left.target_rotation_influence * 100.0;
}
void RenIK::set_leg_target_rotation_influence(float influence) {
	limb_leg_left.target_rotation_influence = influence / 100.0;
	limb_leg_right.target_rotation_influence = influence / 100.0;
}

Vector3 RenIK::get_spine_curve() {
	return spine_chain.chain_curve_direction;
}
void RenIK::set_spine_curve(Vector3 direction) {
	spine_chain.chain_curve_direction = direction;
}
float RenIK::get_upper_spine_stiffness() {
	return spine_chain.get_leaf_stiffness() * 100.0;
}
void RenIK::set_upper_spine_stiffness(float influence) {
	spine_chain.set_leaf_stiffness(skeleton, influence / 100.0);
}
float RenIK::get_lower_spine_stiffness() {
	return spine_chain.get_root_stiffness() * 100.0;
}
void RenIK::set_lower_spine_stiffness(float influence) {
	spine_chain.set_root_stiffness(skeleton, influence / 100.0);
}
float RenIK::get_spine_twist() {
	return spine_chain.get_twist() * 100.0;
}
void RenIK::set_spine_twist(float influence) {
	spine_chain.set_twist(skeleton, influence / 100.0);
}
float RenIK::get_spine_twist_start() {
	return spine_chain.get_twist_start() * 100.0;
}
void RenIK::set_spine_twist_start(float influence) {
	spine_chain.set_twist_start(skeleton, influence / 100.0);
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

#endif // _3D_DISABLED