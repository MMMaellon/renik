#include "renik.h"

RenIK::RenIK(){};

void RenIK::_bind_methods() {

	ClassDB::bind_method(D_METHOD("set_skeleton_path", "skeleton_path"), &RenIK::set_skeleton_path);
	ClassDB::bind_method(D_METHOD("get_skeleton_path"), &RenIK::get_skeleton_path);

	ClassDB::bind_method(D_METHOD("set_head_bone_by_name", "head_bone_name"), &RenIK::set_head_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_head_bone_name"), &RenIK::get_head_bone_name);
	ClassDB::bind_method(D_METHOD("set_hand_left_bone_by_name", "hand_left_bone_name"), &RenIK::set_hand_left_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_hand_left_bone_name"), &RenIK::get_hand_left_bone_name);
	ClassDB::bind_method(D_METHOD("set_hand_right_bone_by_name", "hand_right_bone_name"), &RenIK::set_hand_right_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_hand_right_bone_name"), &RenIK::get_hand_right_bone_name);
	ClassDB::bind_method(D_METHOD("set_hip_bone_by_name", "hip_bone_name"), &RenIK::set_hip_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_hip_bone_name"), &RenIK::get_hip_bone_name);
	ClassDB::bind_method(D_METHOD("set_foot_left_bone_by_name", "foot_left_bone_name"), &RenIK::set_foot_left_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_foot_left_bone_name"), &RenIK::get_foot_left_bone_name);
	ClassDB::bind_method(D_METHOD("set_foot_right_bone_by_name", "foot_right_bone_name"), &RenIK::set_foot_right_bone_by_name);
	ClassDB::bind_method(D_METHOD("get_foot_right_bone_name"), &RenIK::get_foot_right_bone_name);

	// ClassDB::bind_method(D_METHOD("set_head_target_path", "head_target_path"), &RenIK::set_head_target_path);
	// ClassDB::bind_method(D_METHOD("get_head_target_path"), &RenIK::get_head_target_path);
	// ClassDB::bind_method(D_METHOD("set_hand_left_target_path", "hand_left_target_path"), &RenIK::set_hand_left_target_path);
	// ClassDB::bind_method(D_METHOD("get_hand_left_target_path"), &RenIK::get_hand_left_target_path);
	// ClassDB::bind_method(D_METHOD("set_hand_right_target_path", "hand_right_target_path"), &RenIK::set_hand_right_target_path);
	// ClassDB::bind_method(D_METHOD("get_hand_right_target_path"), &RenIK::get_hand_right_target_path);
	// ClassDB::bind_method(D_METHOD("set_hip_target_path", "hip_target_path"), &RenIK::set_hip_target_path);
	// ClassDB::bind_method(D_METHOD("get_hip_target_path"), &RenIK::get_hip_target_path);
	// ClassDB::bind_method(D_METHOD("set_foot_left_target_path", "foot_left_target_path"), &RenIK::set_foot_left_target_path);
	// ClassDB::bind_method(D_METHOD("get_foot_left_target_path"), &RenIK::get_foot_left_target_path);
	// ClassDB::bind_method(D_METHOD("set_foot_right_target_path", "foot_right_target_path"), &RenIK::set_foot_right_target_path);
	// ClassDB::bind_method(D_METHOD("get_foot_right_target_path"), &RenIK::get_foot_right_target_path);

	ADD_GROUP("Armature", "armature_");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "skeleton_path"), "set_skeleton_path", "get_skeleton_path");

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "head_bone_name"), "set_head_bone_by_name", "get_head_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "hand_left_bone_name"), "set_hand_left_bone_by_name", "get_hand_left_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "hand_right_bone_name"), "set_hand_right_bone_by_name", "get_hand_right_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "hip_bone_name"), "set_hip_bone_by_name", "get_hip_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "foot_left_bone_name"), "set_foot_left_bone_by_name", "get_foot_left_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "foot_right_bone_name"), "set_foot_right_bone_by_name", "get_foot_right_bone_name");

	// ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "head_target_path"), "set_head_target_path", "get_head_target_path");
	// ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "hand_left_target_path"), "set_hand_left_target_path", "get_hand_left_target_path");
	// ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "hand_right_target_path"), "set_hand_right_target_path", "get_hand_right_target_path");
	// ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "hip_target_path"), "set_hip_target_path", "get_hip_target_path");
	// ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "foot_left_target_path"), "set_foot_left_target_path", "get_foot_left_target_path");
	// ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "foot_right_target_path"), "set_foot_right_target_path", "get_foot_right_target_path");

	ADD_GROUP("Body IK", "body_ik_");
	ADD_GROUP("Arm IK", "arm_ik_");
	ADD_GROUP("Leg IK", "leg_ik_");
	ADD_GROUP("Hip Placement", "hip_placement_");
	ADD_GROUP("Foot Placement", "foot_placement_");
	// register_property<RenIK, float>("Floor Height", &RenIK::floorHeight, 0.0);
	// register_property<RenIK, float>("Dangle Looseness", &RenIK::relaxAmount, 30.0);
	// register_property<RenIK, float>("Crouch Bend Factor", &RenIK::crouchBendFactor, 50.0);
	// register_property<RenIK, float>("Hunch Factor", &RenIK::hunchFactor, 0.0);
	// register_property<RenIK, float>("Shoulder Flex Multiplier", &RenIK::shoulderFlex, 10.0);
	// register_property<RenIK, float>("Max Leg Stretch", &RenIK::legStretch, 0.1);
	// register_property<RenIK, float>("Max Arm Stretch", &RenIK::armStretch, 0.1);

	// register_property<RenIK, float>("Elbow Angle Offset", &RenIK::elbowAngleOffset, -90);
	// register_property<RenIK, float>("Elbow Rest Offset", &RenIK::elbowRestOffset, -45.0);
	// register_property<RenIK, float>("Shoulder Twisting", &RenIK::shoulderTwist, 50);
	// register_property<RenIK, float>("Wrist Twisting", &RenIK::wristTwist, 66.666);
	// register_property<RenIK, Vector3>("Elbow Pole Offset", &RenIK::elbowPoleOffset, Vector3(0, 0, -60));
	// register_property<RenIK, Vector3>("Hand Position Elbow Influence", &RenIK::elbowPositionBias, Vector3(-100, 0, -50.0));
	// register_property<RenIK, float>("Hand Rotation Elbow Influence", &RenIK::elbowRotationBias, 50.0);
	// register_property<RenIK, float>("Knee Angle Offset", &RenIK::kneeAngleOffset, 180.0);
	// register_property<RenIK, float>("Knee Rest Offset", &RenIK::kneeRestOffset, 0.0);
	// register_property<RenIK, float>("Hip Twisting", &RenIK::hipTwist, 0);
	// register_property<RenIK, float>("Ankle Twisting", &RenIK::ankleTwist, 20);
	// register_property<RenIK, Vector3>("Knee Pole Offset", &RenIK::kneePoleOffset, Vector3(-30, -20, 0));
	// register_property<RenIK, Vector3>("Foot Position Knee Influence", &RenIK::kneePositionBias, Vector3(0, 0, 30));
	// register_property<RenIK, float>("Foot Rotation Knee Influence", &RenIK::kneeRotationBias, 10.0);

	// register_property<RenIK, Vector2>("Spine Curve Axis", &RenIK::curveAxis, Vector2(-1, 0));
	// register_property<RenIK, float>("Hip Turn Speed", &RenIK::hipTurnSpeed, 5.0);
	// register_property<RenIK, float>("Spine Twist", &RenIK::spineTwist, 1.0);
	// register_property<RenIK, float>("Spine Twist Offset", &RenIK::spineTwistOffset, 0);

	// register_property<RenIK, float>("Max Stair Height", &RenIK::rayCastHeight, 0.5);

	// register_property<RenIK, bool>("Enable Head Tracker", &RenIK::headTrackerEnabled, false);
	// register_property<RenIK, bool>("Enable Left Hand Tracker", &RenIK::leftHandTrackerEnabled, false);
	// register_property<RenIK, bool>("Enable Right Hand Tracker", &RenIK::rightHandTrackerEnabled, false);
	// register_property<RenIK, bool>("Enable Hip Tracker", &RenIK::hipTrackerEnabled, true);
	// register_property<RenIK, bool>("Enable Left Foot Tracker", &RenIK::leftFootTrackerEnabled, true);
	// register_property<RenIK, bool>("Enable Right Foot Tracker", &RenIK::rightFootTrackerEnabled, true);

	// register_property<RenIK, bool>("Manual Update", &RenIK::manualUpdate, false);

	ClassDB::bind_method(D_METHOD("update_ik"), &RenIK::update_ik);
	ClassDB::bind_method(D_METHOD("update_placement"), &RenIK::update_placement);
}

void RenIK::_validate_property(PropertyInfo &property) const {
	if (property.name == "head_bone_name" || property.name == "hip_bone_name" || property.name == "hand_left_bone_name" || property.name == "hand_right_bone_name" || property.name == "foot_left_bone_name" || property.name == "foot_right_bone_name") {
		if (skeleton != nullptr) {
			String names("--,");
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
		case NOTIFICATION_ENTER_TREE: {
			Node *parent = get_parent();
			if (parent) {
				set_skeleton_path(parent->get_path());
			}
			set_process_priority(1);
			break;
		}
		case NOTIFICATION_POSTINITIALIZE:
			default_settings();
			break;
		case NOTIFICATION_READY:
			validate_settings();
			break;
		case NOTIFICATION_PROCESS:
			if (!manualUpdate) {
				update_ik(get_process_delta_time());
			}
			break;
		case NOTIFICATION_PHYSICS_PROCESS:
			if (!manualUpdate) {
				update_placement(get_physics_process_delta_time());
			}
			break;
	}
}

void RenIK::default_settings() {
}

void RenIK::validate_settings() {
}

void RenIK::update_ik(float influence) {
	perform_torso_ik(influence);
	perform_left_hand_ik(influence);
	perform_right_hand_ik(influence);
	perform_left_foot_ik(influence);
	perform_right_foot_ik(influence);
}

void RenIK::update_placement(float delta) {
	hip_place(delta);
	foot_place(delta);
}

void RenIK::perform_torso_ik(float influence) {
}

void RenIK::perform_left_hand_ik(float influence) {
}

void RenIK::perform_right_hand_ik(float influence) {
}

void RenIK::perform_left_foot_ik(float influence) {
}

void RenIK::perform_right_foot_ik(float influence) {
}

void RenIK::hip_place(float delta) {
}

void RenIK::foot_place(float delta) {
}

Map<BoneId, Transform> RenIK::solve_trig_ik(TrigLimb limb, Transform global_target) {
	Map<BoneId, Transform> map;
	BoneId leafId = limb.leaf;
	BoneId lowerId = limb.skeleton.get_bone_parent(leafId);
	BoneId upperId = limb.skeleton.get_bone_parent(lowerId);
	BoneId parentId = limb.skeleton.get_bone_parent(upperId);

	if (upperId >= 0) { //There's no way to find a valid upperId if any of the other Id's are invalid, so we only check parent
		Transform leafRest = limb.skeleton.get_bone_rest(leafId);
		Transform lowerRest = limb.skeleton.get_bone_rest(lowerId);
		Transform upperRest = limb.skeleton.get_bone_rest(upperId);
		Transform parent = limb.skeleton.get_global_transform();
		if (parentId >= 0) {
			parent = parent * limb.skeleton.get_bone_global_pose(parentId);
		}
		real_t upperLength = lowerRest.get_origin().length();
		real_t lowerLength = leafRest.get_origin().length();
		Quat upper = upperRest.get_basis().get_quat();
		Quat lower = lowerRest.get_basis().get_quat();
	}
	return map;
}

Map<BoneId, Transform> RenIK::solve_isfabrik(FABRIKChain chain, Transform target, float threshold, int loopLimit) {
	Map<BoneId, Transform> map;
	return map;
}

Map<BoneId, Transform> RenIK::solve_fabrik(FABRIKChain chain, Transform target, float threshold, int loopLimit) {
	Map<BoneId, Transform> map;
	return map;
}

Vector<BoneId> RenIK::calculate_bone_chain(BoneId root, BoneId leaf) {
	Vector<BoneId> chain;
	BoneId b = leaf;
	chain.push_back(b);
	if (skeleton != nullptr) {
		while (b >= 0 && b != root) {
			b = skeleton->get_bone_parent(b);
			chain.push_back(b);
		}
		if (b < 0) {
			chain.clear();
			chain.push_back(leaf);
		} else {
			chain.push_back(root);
		}
	}
	chain.invert();
	return chain;
}

NodePath RenIK::get_skeleton_path() {
	return skeleton_path;
}
void RenIK::set_skeleton_path(NodePath new_skeleton_path) {
	skeleton_path = new_skeleton_path;
	Node *skeletonNode = get_node(skeleton_path);
	skeleton = Object::cast_to<Skeleton>(skeletonNode);
}

void RenIK::set_head_bone_by_name(String bone) {
	head_bone_name = bone;
	if (skeleton != nullptr) {
		BoneId id = skeleton->find_bone(bone);
		if (id >= 0) {
			set_head_bone(id);
		}
	}
}
void RenIK::set_hand_left_bone_by_name(String bone) {
	hand_left_bone_name = bone;
	if (skeleton != nullptr) {
		BoneId id = skeleton->find_bone(bone);
		if (id >= 0) {
			set_hand_left_bone(id);
		}
	}
}
void RenIK::set_hand_right_bone_by_name(String bone) {
	hand_right_bone_name = bone;
	if (skeleton != nullptr) {
		BoneId id = skeleton->find_bone(bone);
		if (id >= 0) {
			set_hand_right_bone(id);
		}
	}
}
void RenIK::set_hip_bone_by_name(String bone) {
	hip_bone_name = bone;
	if (skeleton != nullptr) {
		BoneId id = skeleton->find_bone(bone);
		if (id >= 0) {
			set_hip_bone(id);
		}
	}
}
void RenIK::set_foot_left_bone_by_name(String bone) {
	foot_left_bone_name = bone;
	if (skeleton != nullptr) {
		BoneId id = skeleton->find_bone(bone);
		if (id >= 0) {
			set_foot_left_bone(id);
		}
	}
}
void RenIK::set_foot_right_bone_by_name(String bone) {
	foot_right_bone_name = bone;
	if (skeleton != nullptr) {
		BoneId id = skeleton->find_bone(bone);
		if (id >= 0) {
			set_foot_right_bone(id);
		}
	}
}

void RenIK::set_head_bone(BoneId bone) {
	head = bone;
	spine_chain.bones = calculate_bone_chain(hip, head);
}
void RenIK::set_hand_left_bone(BoneId bone) {
	limb_arm_left.leaf = bone;
}
void RenIK::set_hand_right_bone(BoneId bone) {
	limb_arm_right.leaf = bone;
}
void RenIK::set_hip_bone(BoneId bone) {
	hip = bone;
	spine_chain.bones = calculate_bone_chain(hip, head);
}
void RenIK::set_foot_left_bone(BoneId bone) {
	limb_leg_left.leaf = bone;
}
void RenIK::set_foot_right_bone(BoneId bone) {
	limb_leg_right.leaf = bone;
}

int64_t RenIK::get_hip_bone() {
	return hip;
}
int64_t RenIK::get_head_bone() {
	return head;
}
int64_t RenIK::get_hand_left_bone() {
	return limb_arm_left.leaf;
}
int64_t RenIK::get_hand_right_bone() {
	return limb_arm_right.leaf;
}
int64_t RenIK::get_foot_left_bone() {
	return limb_leg_left.leaf;
}
int64_t RenIK::get_foot_right_bone() {
	return limb_leg_right.leaf;
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