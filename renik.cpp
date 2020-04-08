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
		limb_arm_left(0, 0, Math_PI, 0.25, 0.25, Math::deg2rad(20.0), Math::deg2rad(45.0), 0.25, Vector3(Math::deg2rad(60.0), 0, 0), Vector3()),
		limb_arm_right(0, 0, -Math_PI, 0.25, 0.25, Math::deg2rad(-20.0), Math::deg2rad(45.0), 0.25, Vector3(Math::deg2rad(-60.0), 0, 0), Vector3()),
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

	ADD_GROUP("Arm IK Settings (Advanced)", "arm_");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "arm_pole_offset"), "set_arm_pole_offset", "get_arm_pole_offset");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "arm_target_position_influence"), "set_arm_target_position_influence", "get_arm_target_position_influence");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "arm_target_rotation_influence", PROPERTY_HINT_RANGE, "0,100,0.1"), "set_arm_target_rotation_influence", "get_arm_target_rotation_influence");

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

	ADD_GROUP("Walk Settings (Advanced)", "walk_");

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
				if (!manual_update) {
					update_ik(1);
				}
			}
			break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS:
			if (!Engine::get_singleton()->is_editor_hint() || live_preview) {
				if (!manual_update) {
					update_placement(get_physics_process_delta_time());
				}
				break;
			}
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
	set_process_internal(true);
	set_physics_process_internal(true);
}

void RenIK::update_ik(float influence) {
	perform_torso_ik(influence);
	perform_hand_left_ik(influence);
	perform_hand_right_ik(influence);
	perform_foot_left_ik(influence);
	perform_foot_right_ik(influence);
}

void RenIK::update_placement(float delta) {
	//TODO raytrace for floor here
	//in the meantime, let's assume we're standing on the floor (aka its 0 away)

	hip_place(delta, config);
	foot_place(delta, config);
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

void RenIK::perform_torso_ik(float influence) {
	if (head_target_spatial && skeleton && spine_chain.is_valid()) {
		Transform headTransform = head_target_spatial->get_global_transform();
		Transform hipTransform = hip_target_spatial ? hip_target_spatial->get_global_transform(): placedHip;
		apply_ik_map(solve_ifabrik(spine_chain, hipTransform, headTransform, 0.001, 12));
	}
}

void RenIK::perform_hand_left_ik(float influence) {
	if (hand_left_target_spatial && skeleton && limb_arm_left.is_valid()) {
		Transform root = skeleton->get_global_transform();
		BoneId rootBone = skeleton->get_bone_parent(limb_arm_left.get_upper_bone());
		if (rootBone >= 0) {
			root = root * skeleton->get_bone_global_pose(rootBone);
		}
		apply_ik_map(solve_trig_ik_redux(limb_arm_left, root, hand_left_target_spatial->get_global_transform()));
	}
}

void RenIK::perform_hand_right_ik(float influence) {
	if (hand_right_target_spatial && skeleton && limb_arm_right.is_valid()) {
		Transform root = skeleton->get_global_transform();
		BoneId rootBone = skeleton->get_bone_parent(limb_arm_right.get_upper_bone());
		if (rootBone >= 0) {
			root = root * skeleton->get_bone_global_pose(rootBone);
		}
		apply_ik_map(solve_trig_ik_redux(limb_arm_right, root, hand_right_target_spatial->get_global_transform()));
	}
}

void RenIK::perform_foot_left_ik(float influence) {
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

void RenIK::perform_foot_right_ik(float influence) {
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
	if (skeleton && chain.get_start_bone() < skeleton->get_bone_count() && chain.get_end_bone() < skeleton->get_bone_count()) {
		BoneId bone = chain.get_start_bone();
		while (bone >= 0 && bone != chain.get_end_bone()) {
			skeleton->set_bone_custom_pose(bone, Transform());
			bone = skeleton->get_bone_parent(bone);
		}
		if (bone >= 0) {
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

void RenIK::foot_place(float delta, RenIKConfig config) {
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
		Quat positionRoll = Quat(rollVector, limb.target_position_influence.dot(targetRestPosition - targetVector));
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

		//Add a little twist
		//We align the leaf's y axis with the lower limb's y-axis and see how far off the x-axis is from the joint axis to calculate the twist.
		Vector3 leafX = align_vectors(localLeafVector.rotated(normalizedTargetVector, jointRollAmount), localLowerVector.rotated(normalizedTargetVector, jointRollAmount)).xform(localTarget.get_basis().xform(Vector3(1, 0, 0)));
		Vector3 lowerZ = jointAxis.rotated(localLowerVector, -limb.roll_offset).cross(localLowerVector);
		float twistAngle = leafX.angle_to(jointAxis.rotated(localLowerVector, -limb.roll_offset));
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
		float upperTwist = lowerTwist * limb.upper_limb_twist + limb.upper_twist_offset - limb.roll_offset;
		lowerTwist += limb.lower_twist_offset - 2 * limb.roll_offset;

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
	if (chain.is_valid()) {
		//The true root of the limb is the point where the upper bone starts
		Transform trueRoot = chain.get_rest_start();
		Transform localTarget = trueRoot.affine_inverse() * target;
		Transform localTargetDelta = chain.get_rest_start().affine_inverse() * localTarget;

		Vector<RenIKChain::Joint> joints = chain.get_joints(); //will have n-1 joints. The last is going to be at the root. The first is going to be at the target

		//We generate the starting points
		//Here is where we take into account root and target influences and the prebend vector
		jointPoints.set(jointPoints.size() - 1, Vector3()); //The root joint
		float progress = 0;
		for (int i = jointPoints.size() - 2; i > 0; i--) {
			progress += chain.get_bone_length(i);
			Vector3 rootVector = chain.get_rest_joints()[i]; //tries to match the rest transform
			Vector3 targetVector = localTargetDelta.xform(chain.get_rest_joints()[i]);
			Vector3 prebendVector = align_vectors(Vector3(0, 1, 0), localTarget.origin).xform(chain.chain_curve_direction);
			float rootProgress = progress / chain.get_total_length();
			float targetProgress = (chain.get_total_length() - progress) / chain.get_total_length();
			float rootInfluence = (1 - rootProgress);
			float targetInfluence = (1 - targetProgress);

			jointPoints.set(i, jointPoints[i] + prebendVector);
			jointPoints[i].linear_interpolate(rootVector, rootInfluence);
			jointPoints[i].linear_interpolate(targetVector, targetInfluence);
		}
		jointPoints.set(0, localTarget.origin); //The target joint

		//We then do regular FABRIK
		for (int i = 0; i < loopLimit; i++) {
			Vector3 lastJoint = Vector3(); //the root joint
			//Forwards
			for (int j = jointPoints.size() - 2; j > 0; j--) {
				Vector3 delta = jointPoints[j] - lastJoint;
				delta.normalize();
				delta = delta * chain.get_bone_length(j + 1); //set to length of parent
				jointPoints.set(j, lastJoint + delta);
			}

			lastJoint = jointPoints[0];
			//Backwards
			for (int j = 1; j < jointPoints.size() - 1; j++) {
				Vector3 delta = jointPoints[j] - lastJoint;
				delta.normalize();
				delta = delta * chain.get_bone_length(j);
				jointPoints.set(j, lastJoint + delta);
			}

			float error = lastJoint.length() - chain.get_bone_length(chain.get_bone_count() - 1);
			if ((Math::abs(error)) < threshold) {
				break;
			}
		}

		//Convert everything to quaternions and store it in the map
		Quat prev = root.get_basis().get_quat();
		map.insert(chain.get_bone_id(chain.get_bone_count() - 1), prev); //set the root
		for (int i = chain.get_bone_count() - 2; i > 0; i--) {
			BoneId bone = chain.get_bone_id(i);
			Vector3 nextBone = jointPoints[i + 1];
			map.insert(bone, root.get_basis().get_quat());
		}
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
		}
		if (hand_right_target_spatial) {
			reset_limb(limb_arm_right);
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
	spine_chain.set_chain(skeleton, spine_chain.get_start_bone(), p_bone);
}
void RenIK::set_hand_left_bone(BoneId p_bone) {
	limb_arm_left.set_leaf(skeleton, p_bone);
}
void RenIK::set_hand_right_bone(BoneId p_bone) {
	limb_arm_right.set_leaf(skeleton, p_bone);
}
void RenIK::set_hip_bone(BoneId p_bone) {
	hip = p_bone;
	spine_chain.set_chain(skeleton, p_bone, spine_chain.get_end_bone());
	if (skeleton) {
		hip_height = skeleton->get_bone_rest(hip).get_origin()[1]; //y component of the hip origin
	}
}
void RenIK::set_foot_left_bone(BoneId p_bone) {
	limb_leg_left.set_leaf(skeleton, p_bone);
}
void RenIK::set_foot_right_bone(BoneId p_bone) {
	limb_leg_right.set_leaf(skeleton, p_bone);
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
	Quat q2 = Quat(Vector3(Math::deg2rad(-euler[0]), Math::deg2rad(-euler[1]), Math::deg2rad(euler[2])));
	limb_arm_left.pole_offset = q;
	limb_arm_right.pole_offset = q2;
}
Vector3 RenIK::get_arm_target_position_influence() {
	return limb_arm_left.target_position_influence * 10.0;
}
void RenIK::set_arm_target_position_influence(Vector3 xyz) {
	limb_arm_left.target_position_influence = xyz / 10.0;
	limb_arm_right.target_position_influence = xyz / 10.0;
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
	Quat q2 = Quat(Vector3(Math::deg2rad(-euler[0]), Math::deg2rad(-euler[1]), Math::deg2rad(euler[2])));
	limb_leg_left.pole_offset = q;
	limb_leg_right.pole_offset = q2;
}
Vector3 RenIK::get_leg_target_position_influence() {
	return limb_leg_left.target_position_influence * 10.0;
}
void RenIK::set_leg_target_position_influence(Vector3 xyz) {
	limb_leg_left.target_position_influence = xyz / 10.0;
	limb_leg_right.target_position_influence = xyz / 10.0;
}
float RenIK::get_leg_target_rotation_influence() {
	return limb_leg_left.target_rotation_influence * 100.0;
}
void RenIK::set_leg_target_rotation_influence(float influence) {
	limb_leg_left.target_rotation_influence = influence / 100.0;
	limb_leg_right.target_rotation_influence = influence / 100.0;
}

#endif // _3D_DISABLED