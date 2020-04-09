#ifndef RENIK_H
#define RENIK_H
#ifndef _3D_DISABLED

#include "renik/renik_chain.h"
#include "renik/renik_limb.h"
#include <core/engine.h>
#include <core/variant.h>
#include <scene/3d/skeleton.h>
#include <scene/main/node.h>
#include <memory>
#include <vector>

class RenIK : public Node {
	GDCLASS(RenIK, Node);

public:
	struct RenIKConfig : public Reference {
		GDCLASS(RenIKConfig, Reference);

	public:
		//Hip Placement Adjustments
		float crouch_amount; //Crouching means bending over at the hip while keeping the spine straight
		float hunch_amount; //Hunching means bending over by arching the spine
		float hip_turn_speed; //How fast the hips turn to follow the head
		float hip_turn_limit; //How far the hips are allowed to stray from the direction the head is facing
		float hip_follow_head_influence;

		//Foot Placement Adjustments - Only takes effect when there are no foot targets
		//These are values when at the slowest walk speed
		bool calculate_speed;
		float floor_offset;
		float raycast_allowance; //how far past the max length of the limb we'll still consider close enough
		float contact_length;
		float followthru_angle; //local to the LOOP y is up, x is forward
		float buildup_angle;
		float loop_ratio; //ratio of buildup / followthru
		float step_height;
		float min_threshold;
		float max_threshold; //when all scaling stops and the legs just move faster
		float raycast_threshold;
		float balance_threshold;
		float center_of_balance_position; //distance between hips and head that we'll call the center of balance. Usually at 25%, near the belly button

		float dangle_height;
		float dangle_stiffness;

		//Everything scales logarithmically
		float contact_scale;
		float height_scale;
		float loop_scale;
		float time_scale;
		RenIKConfig() {
			crouch_amount = 0;
			hunch_amount = 0;
			hip_turn_speed = 0;
			hip_turn_limit = 0;
			hip_follow_head_influence = 0;
			calculate_speed = false;
			floor_offset = 0;
			raycast_allowance = 0;
			contact_length = 0;
			followthru_angle = 0;
			buildup_angle = 0;
			loop_ratio = 0;
			step_height = 0;
			min_threshold = 0;
			max_threshold = 0;
			raycast_threshold = 0;
			balance_threshold = 0;
			center_of_balance_position = 0;
			dangle_height = 0;
			dangle_stiffness = 0;
			contact_scale = 0;
			height_scale = 0;
			loop_scale = 0;
			time_scale = 0;
		}
	};

	struct FeetTransforms : public Reference {
		GDCLASS(FeetTransforms, Reference);

	public:
		Transform leftFoot;
		Transform rightFoot;
		FeetTransforms() :
				leftFoot(),
				rightFoot() {}
	};

	RenIK();

	void _initialize();

	virtual void _validate_property(PropertyInfo &property) const;
	void _notification(int p_what);
	static void _bind_methods();

	void update_ik();
	void update_placement(float delta);

	void apply_ik_map(Map<BoneId, Quat> ikMap);
	void apply_ik_map(Map<BoneId, Basis> ikMap);
	void perform_torso_ik();
	void perform_hand_left_ik();
	void perform_hand_right_ik();
	void perform_foot_left_ik();
	void perform_foot_right_ik();
	void reset_chain(RenIKChain chain);
	void reset_limb(RenIKLimb limb);

	bool get_live_preview();
	void set_live_preview(bool p_enable);

	NodePath get_skeleton_path();
	void set_skeleton_path(NodePath p_path);
	void set_skeleton(Node *p_path);

	void set_head_bone_by_name(String p_bone);
	void set_hand_left_bone_by_name(String p_bone);
	void set_hand_right_bone_by_name(String p_bone);
	void set_hip_bone_by_name(String p_bone);
	void set_foot_left_bone_by_name(String p_bone);
	void set_foot_right_bone_by_name(String p_bone);

	void set_head_bone(BoneId p_bone);
	void set_hand_left_bone(BoneId p_bone);
	void set_hand_right_bone(BoneId p_bone);
	void set_hip_bone(BoneId p_bone);
	void set_foot_left_bone(BoneId p_bone);
	void set_foot_right_bone(BoneId p_bone);

	int64_t get_hip_bone();
	int64_t get_head_bone();
	int64_t get_hand_left_bone();
	int64_t get_hand_right_bone();
	int64_t get_foot_left_bone();
	int64_t get_foot_right_bone();

	String get_hip_bone_name();
	String get_head_bone_name();
	String get_hand_left_bone_name();
	String get_hand_right_bone_name();
	String get_foot_left_bone_name();
	String get_foot_right_bone_name();

	void set_head_target_path(NodePath p_path);
	void set_hand_left_target_path(NodePath p_path);
	void set_hand_right_target_path(NodePath p_path);
	void set_hip_target_path(NodePath p_path);
	void set_foot_left_target_path(NodePath p_path);
	void set_foot_right_target_path(NodePath p_path);
	NodePath get_head_target_path();
	NodePath get_hand_left_target_path();
	NodePath get_hand_right_target_path();
	NodePath get_hip_target_path();
	NodePath get_foot_left_target_path();
	NodePath get_foot_right_target_path();

	//IK Settings
	float get_arm_upper_twist_offset();
	void set_arm_upper_twist_offset(float degrees);
	float get_arm_lower_twist_offset();
	void set_arm_lower_twist_offset(float degrees);
	float get_arm_roll_offset();
	void set_arm_roll_offset(float degrees);
	float get_arm_upper_limb_twist();
	void set_arm_upper_limb_twist(float ratio);
	float get_arm_lower_limb_twist();
	void set_arm_lower_limb_twist(float ratio);
	float get_arm_twist_inflection_point_offset();
	void set_arm_twist_inflection_point_offset(float degrees);
	float get_arm_twist_overflow();
	void set_arm_twist_overflow(float degrees);

	Vector3 get_arm_pole_offset();
	void set_arm_pole_offset(Vector3 euler);
	Vector3 get_arm_target_position_influence();
	void set_arm_target_position_influence(Vector3 xyz);
	float get_arm_target_rotation_influence();
	void set_arm_target_rotation_influence(float influence);
	float get_arm_target_twist_influence();
	void set_arm_target_twist_influence(float influence);

	float get_leg_upper_twist_offset();
	void set_leg_upper_twist_offset(float degrees);
	float get_leg_lower_twist_offset();
	void set_leg_lower_twist_offset(float degrees);
	float get_leg_roll_offset();
	void set_leg_roll_offset(float degrees);
	float get_leg_upper_limb_twist();
	void set_leg_upper_limb_twist(float ratio);
	float get_leg_lower_limb_twist();
	void set_leg_lower_limb_twist(float ratio);
	float get_leg_twist_inflection_point_offset();
	void set_leg_twist_inflection_point_offset(float degrees);
	float get_leg_twist_overflow();
	void set_leg_twist_overflow(float degrees);

	Vector3 get_leg_pole_offset();
	void set_leg_pole_offset(Vector3 euler);
	Vector3 get_leg_target_position_influence();
	void set_leg_target_position_influence(Vector3 xyz);
	float get_leg_target_rotation_influence();
	void set_leg_target_rotation_influence(float influence);
	float get_leg_target_twist_influence();
	void set_leg_target_twist_influence(float influence);

	Vector3 get_spine_curve();
	void set_spine_curve(Vector3 influence);
	float get_upper_spine_stiffness();
	void set_upper_spine_stiffness(float influence);
	float get_lower_spine_stiffness();
	void set_lower_spine_stiffness(float influence);
	float get_spine_twist();
	void set_spine_twist(float influence);
	float get_spine_twist_start();
	void set_spine_twist_start(float influence);

	float get_shoulder_influence();
	void set_shoulder_influence(float influence);

	Vector3 get_shoulder_offset();
	void set_shoulder_offset(Vector3 euler);

	bool get_use_editor_speed();
	void set_use_editor_speed(bool enable);

	static Quat align_vectors(Vector3 a, Vector3 b, float influence = 1);

	static std::pair<float, float> trig_angles(Vector3 const &length1, Vector3 const &length2, Vector3 const &length3);
	static Map<BoneId, Quat> solve_trig_ik(RenIKLimb limb, Transform limb_parent_transform, Transform target);

	static Map<BoneId, Basis> solve_trig_ik_redux(RenIKLimb &limb, Transform limb_parent_transform, Transform target);

	static Map<BoneId, Quat> solve_ifabrik(RenIKChain chain, Transform chain_parent_transform, Transform target, float threshold, int loopLimit);

	void hip_place(float delta, RenIKConfig config);
	void foot_place(float delta, RenIKConfig config);
	//All used in leg trace
	void set_falling(bool falling);
	void set_manual_update(bool update_manually);
	void update();

private:
	//Setup -------------------------
	bool live_preview = false;
	//The Skeleton
	NodePath skeleton_path = NodePath("..");
	Skeleton *skeleton = nullptr;

	//IK Targets
	NodePath head_target_path;
	NodePath hand_left_target_path;
	NodePath hand_right_target_path;
	NodePath hip_target_path;
	NodePath foot_left_target_path;
	NodePath foot_right_target_path;
	Spatial *head_target_spatial = nullptr;
	Spatial *hand_left_target_spatial = nullptr;
	Spatial *hand_right_target_spatial = nullptr;
	Spatial *hip_target_spatial = nullptr;
	Spatial *foot_left_target_spatial = nullptr;
	Spatial *foot_right_target_spatial = nullptr;

	//IK ADJUSTMENTS --------------------
	RenIKConfig config;
	String head_bone_name;
	String hip_bone_name;
	String hand_left_bone_name;
	String hand_right_bone_name;
	String foot_left_bone_name;
	String foot_right_bone_name;

	BoneId hip = -1;
	BoneId head = -1;

	RenIKChain spine_chain;
	RenIKLimb limb_arm_left;
	RenIKLimb limb_arm_right;
	RenIKLimb limb_leg_left;
	RenIKLimb limb_leg_right;
	float shoulder_influence = 0.3;
	bool left_shoulder_enabled = false;
	bool right_shoulder_enabled = false;
	Vector3 left_shoulder_offset;
	Vector3 right_shoulder_offset;

	//General Settings ------------------
	bool manual_update = false;
	bool hip_placement = true;
	bool foot_placement = true;
	bool headTrackerEnabled = true;
	bool leftHandTrackerEnabled = true;
	bool rightHandTrackerEnabled = true;
	bool hipTrackerEnabled = true;
	bool leftFootTrackerEnabled = true;
	bool rightFootTrackerEnabled = true;

	// //Internal Variables --------------------
	// //IK Cache
	// //If the transform didn't change, we skip recalculating IK
	// Transform headTargetCached;
	// Transform handLeftTargetCached;
	// Transform handRightTargetCached;
	// Transform hipTargetCached;
	// Transform footLeftTargetCached;
	// Transform footRightTargetCached;

	// //tPose Cache
	// float tPoseHeight;
	// float tPoseHipsDist;
	// Vector3 tPoseHipVector;
	// Transform tPoseHeadLocal;
	// Transform tPoseHipTransformLocalToHead;
	// Transform tPoseHead;
	// Transform tPoseHips;
	// Transform tPoseHandLeft;
	// Transform tPoseHandRight;
	// Transform tPoseFootLeft;
	// Transform tPoseFootRight;

	int walk_state = 0; //0 is stand state, 1 is step state, -1 is jump state
	float ground_speed = 0;
	Transform placedHip; //relative to skeleton
	Transform placedLeft; //relative to world
	Transform placedRight; //relative to world

	//cache
	float hip_height = 0;

	Vector<BoneId> calculate_bone_chain(BoneId root, BoneId leaf);

	//SIFABRIK
	float spineLength = 0;
	struct BonePoint {
		BonePoint(float l, Transform t, Vector3 p) {
			length = l;
			transform = t;
			point = p;
		}
		BonePoint() {
			length = 0;
			transform = Transform();
			point = Vector3();
		}
		float length = 0;
		Transform transform; //local
		Vector3 point; //global
	};

	static float smoothCurve(float number, float modifier = 0.5);
	static float sinusoidalInterpolation(float number);
	static Vector3 vector_rejection(Vector3 v, Vector3 normal);
	static float safe_acos(float f);
	static float safe_asin(float f);
	static Vector3 get_perpendicular_vector(Vector3 v);

	//for SIFABRIK
	static void solveFABRIKPoints(std::vector<RenIK::BonePoint> &bonePoints, Vector3 rootPoint, Vector3 goal, float threshold, int loopLimit);
	static Vector3 fitPointToLine(Vector3 point, Vector3 goal, float length);
};

#endif
#endif