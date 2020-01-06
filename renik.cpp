#include "renik.h"

RenIK::RenIK(){};

void RenIK::_bind_methods() {
	ADD_GROUP("Armature", "armature_");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "Skeleton"), "set_skeleton_path", "get_skeleton_path");

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Head Bone"), "set_head_bone", "get_head_bone");
	ADD_PROPERTY(PropertyInfo(Variant::POOL_STRING_ARRAY, "Neck Bones"), "set_neck_bones", "get_neck_bones");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Shoulder Left Bone"), "set_shoulder_left_bone", "get_shoulder_left_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Upper Arm Left Bone"), "set_upper_arm_left_bone", "get_upper_arm_left_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Forearm Left Bone"), "set_forearm_left_bone", "get_forearm_left_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Hand Left Bone"), "set_hand_left_bone", "get_hand_left_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Shoulder Right Bone"), "set_shoulder_right_bone", "get_shoulder_right_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Upper Arm Right Bone"), "set_upper_arm_right_bone", "get_upper_arm_right_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Forearm Right Bone"), "set_forearm_right_bone", "get_forearm_right_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Hand Right Bone"), "set_hand_right_bone", "get_hand_right_bone");
	ADD_PROPERTY(PropertyInfo(Variant::POOL_STRING_ARRAY, "Spine Bones"), "set_spine_bones", "get_spine_bones");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Hip Bone"), "set_hip_bone", "get_hip_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Thigh Left Bone"), "set_thigh_left_bone", "get_thigh_left_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Shin Left Bone"), "set_shin_left_bone", "get_shin_left_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Foot Left Bone"), "set_foot_left_bone", "get_foot_left_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Thigh Right Bone"), "set_thigh_right_bone", "get_thigh_right_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Shin Right Bone"), "set_shin_right_bone", "get_shin_right_bone");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "Foot Right Bone"), "set_foot_right_bone", "get_foot_right_bone");

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "Head Target"), "set_head_target_path", "get_head_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "Left Hand Target"), "set_hand_left_target_path", "get_hand_left_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "Right Hand Target"), "set_hand_right_target_path", "get_hand_right_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "Hip Target"), "set_hip_target_path", "get_hip_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "Left Foot Target"), "set_foot_left_target_path", "get_foot_left_target_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "Right Foot Target"), "set_foot_right_target_path", "get_foot_right_target_path");

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

	// ClassDB::bind_method(D_METHOD("_physics_process"), &RenIK::_physics_process);
	// ClassDB::bind_method(D_METHOD("_process"), &RenIK::_process);
	// ClassDB::bind_method(D_METHOD("_ready"), &RenIK::_ready);

	// ClassDB::bind_method(D_METHOD("performBodyIK"), &RenIK::performBodyIK);
	// ClassDB::bind_method(D_METHOD("performLeftHandIK"), &RenIK::performLeftHandIK);
	// ClassDB::bind_method(D_METHOD("performRightHandIK"), &RenIK::performRightHandIK);
	// ClassDB::bind_method(D_METHOD("performLeftFootIK"), &RenIK::performLeftFootIK);
	// ClassDB::bind_method(D_METHOD("performRightFootIK"), &RenIK::performRightFootIK);

	// ClassDB::bind_method(D_METHOD("setHeadTargetNode"), &RenIK::setHeadTargetNode);
	// ClassDB::bind_method(D_METHOD("setHandLeftTargetNode"), &RenIK::setHandLeftTargetNode);
	// ClassDB::bind_method(D_METHOD("setHandRightTargetNode"), &RenIK::setHandRightTargetNode);
	// ClassDB::bind_method(D_METHOD("setHipTargetNode"), &RenIK::setHipTargetNode);
	// ClassDB::bind_method(D_METHOD("setFootLeftTargetNode"), &RenIK::setFootLeftTargetNode);
	// ClassDB::bind_method(D_METHOD("setFootRightTargetNode"), &RenIK::setFootRightTargetNode);

	// ClassDB::bind_method(D_METHOD("setFalling"), &RenIK::setFalling);
	// ClassDB::bind_method(D_METHOD("setManualUpdate"), &RenIK::setManualUpdate);
	// ClassDB::bind_method(D_METHOD("update"), &RenIK::update);
}

void RenIK::_validate_property(PropertyInfo &property) const {
}
void RenIK::_notification(int p_what) {

	switch (p_what) {

		case NOTIFICATION_ENTER_TREE: {

			_enter_tree();
		} break;
		case NOTIFICATION_EXIT_TREE: {

			_exit_tree();
		} break;
	}
}

void RenIK::_enter_tree() {
}

void RenIK::_exit_tree() {
}

void RenIK::_physics_process(float delta) {
}

void RenIK::_process(float delta) {
}
