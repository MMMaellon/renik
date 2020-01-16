#ifndef RENIK_H
#define RENIK_H

#include <scene/3d/skeleton.h>
#include <scene/main/node.h>
#include <memory>
#include <vector>

class RenIK : public Node {
	GDCLASS(RenIK, Node);

public:
	struct TrigLimb : public Reference {
		GDCLASS(TrigLimb, Reference);

	public:
		Skeleton skeleton;
		BoneId leaf = -1;

		float angle_offset; //Defines which way is bending forward and which way is backward
		float rest_roll_offset; //Rolls the entire limb so it bends a certain direction at rest
		float upper_limb_twist; //How much the upper limb follows the lower limb
		float lower_limb_twist; //How much the lower limb follows the leaf limb

		Vector3 pole_offset; /*ADVANCED - Moving the limb 180 degrees from rest tends to be a bit unpredictable
		as there is a pole in the forward vector sphere at that spot.
		This offsets the rest position so that the pole is in a place where the limb is unlikely to go*/

		Vector3 target_position_influence; //ADVANCED - How much each of the leaf's axis of translation from rest affects the ik
		float target_twist_influence; //ADVANCED - How much the leaf's rotation affects the ik
		float target_flex_influence; //ADVANCED - How much the leaf's rotation affects the ik
	};

	struct FABRIKChain : public Reference {
		GDCLASS(FABRIKChain, Reference);

	public:
		Skeleton skeleton;
		Vector<BoneId> bones;
		float twist_influence;
		float twist_start;
	};

	struct Posture : public Reference {
		GDCLASS(Posture, Reference);

	public:
		Transform head;
		Transform hip;
		Transform hand_left;
		Transform hand_right;
		Transform foot_left;
		Transform foot_right;
		Transform skeleton;
		//Body Adjustments
		float spineCurve; //To prevent spines from bending the wrong way, we precurve it before solving the IK. This controls how much to precurve it
		float spineCurveAngle; //The angle to precurve it
		float spineTwist; //How much the spine tries to twist to follow the head when the hips are facing a differen direction
		float spineTwistOffset; //Where along the spine the twisting starts

		//Hip Placement Adjustments
		float crouchAmount; //Crouching means bending over at the hip while keeping the spine straight
		float hunchAmount; //Hunching means bending over by arching the spine
		float hipTurnSpeed; //How fast the hips turn to follow the head
		float hipTurnLimit; //How far the hips are allowed to stray from the direction the head is facing

		//Foot Placement Adjustments - Only takes effect when there are no foot targets
		float stepHeight;
		float stepSpeed;
		float stepAnticipation;
		float footAngle;
		float footLooseness;
		float footDangleHeight;
		float distanceThreshold;
		float twistThreshold;
		float walkThreshold;
		float runThreshold;
	};

	RenIK();

	void validate_settings();
	void default_settings(); // our initializer called by Godot

	virtual void _validate_property(PropertyInfo &property) const;
	void _notification(int p_what);
	static void _bind_methods();

	void update_ik(float influence);
	void update_placement(float delta);

	void perform_torso_ik(float influence = 1);
	void perform_left_hand_ik(float influence = 1);
	void perform_right_hand_ik(float influence = 1);
	void perform_left_foot_ik(float influence = 1);
	void perform_right_foot_ik(float influence = 1);

	NodePath get_skeleton_path();
	void set_skeleton_path(NodePath new_skeleton_path);

	void set_head_bone_by_name(String bone);
	void set_hand_left_bone_by_name(String bone);
	void set_hand_right_bone_by_name(String bone);
	void set_hip_bone_by_name(String bone);
	void set_foot_left_bone_by_name(String bone);
	void set_foot_right_bone_by_name(String bone);

	void set_head_bone(BoneId bone);
	void set_hand_left_bone(BoneId bone);
	void set_hand_right_bone(BoneId bone);
	void set_hip_bone(BoneId bone);
	void set_foot_left_bone(BoneId bone);
	void set_foot_right_bone(BoneId bone);

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

	void set_head_target_path(NodePath bone);
	void set_hand_left_target_path(NodePath bone);
	void set_hand_right_target_path(NodePath bone);
	void set_hip_target_path(NodePath bone);
	void set_foot_left_target_path(NodePath bone);
	void set_foot_right_target_path(NodePath bone);
	NodePath get_head_target_path();
	NodePath get_hand_left_target_path();
	NodePath get_hand_right_target_path();
	NodePath get_hip_target_path();
	NodePath get_foot_left_target_path();
	NodePath get_foot_right_target_path();

	static Map<BoneId, Transform> solve_trig_ik(TrigLimb limb, Transform global_target);

	static Map<BoneId, Transform> solve_isfabrik(FABRIKChain chain, Transform target, float threshold, int loopLimit);

	static Map<BoneId, Transform> solve_fabrik(FABRIKChain chain, Transform target, float threshold, int loopLimit);

	static Transform solve_hip_placement(Transform head, Transform hip, float floor_dist, float max_torso_height);
	static Map<int, Transform> solve_foot_placement(Map<int, Transform> trackers);

	void hip_place(float delta);
	void foot_place(float delta);
	//All used in leg trace
	void set_falling(bool falling);
	void set_manual_update(bool update_manually);
	void update();

private:
	//Setup -------------------------
	//The Skeleton
	NodePath skeleton_path;
	Skeleton *skeleton = nullptr;

	//IK Targets
	NodePath headTargetNode;
	NodePath handLeftTargetNode;
	NodePath handRightTargetNode;
	NodePath hipTargetNode;
	NodePath footLeftTargetNode;
	NodePath footRightTargetNode;
	Spatial *headTargetSpatial;
	Spatial *handLeftTargetSpatial;
	Spatial *handRightTargetSpatial;
	Spatial *hipTargetSpatial;
	Spatial *footLeftTargetSpatial;
	Spatial *footRightTargetSpatial;
	Transform headTarget;
	Transform handLeftTarget;
	Transform handRightTarget;
	Transform hipTarget;
	Transform footLeftTarget;
	Transform footRightTarget;

	//IK ADJUSTMENTS --------------------
	Posture posture;
	String head_bone_name;
	String hip_bone_name;
	String hand_left_bone_name;
	String hand_right_bone_name;
	String foot_left_bone_name;
	String foot_right_bone_name;

	BoneId hip = -1;
	BoneId head = -1;

	FABRIKChain spine_chain;
	TrigLimb limb_arm_left;
	TrigLimb limb_arm_right;
	TrigLimb limb_leg_left;
	TrigLimb limb_leg_right;

	//General Settings ------------------
	bool manualUpdate;
	bool headTrackerEnabled;
	bool leftHandTrackerEnabled;
	bool rightHandTrackerEnabled;
	bool hipTrackerEnabled;
	bool leftFootTrackerEnabled;
	bool rightFootTrackerEnabled;

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

	Vector<BoneId> calculate_bone_chain(BoneId root, BoneId leaf);

	//SIFABRIK
	float spineLength;
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
		float length;
		Transform transform; //local
		Vector3 point; //global
	};

	static float smoothCurve(float number, float modifier = 0.5);
	static float sinusoidalInterpolation(float number);
	static Vector3 vectorRejection(Vector3 v, Vector3 normal);
	static void crossDot(Vector3 &cross, float &dot, Vector3 srcVector1, Vector3 srcVector2);
	static float safeACOS(float f);
	static float safeASIN(float f);

	//for SIFABRIK
	static void solveFABRIKPoints(std::vector<RenIK::BonePoint> &bonePoints, Vector3 rootPoint, Vector3 goal, float threshold, int loopLimit);
	static Vector3 fitPointToLine(Vector3 point, Vector3 goal, float length);
};

#endif