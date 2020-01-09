#ifndef RENIK_H
#define RENIK_H

#include <scene/3d/skeleton.h>
#include <scene/main/node.h>
#include <vector>

class RenIK : public Node {
	GDCLASS(RenIK, Node);

private:
	//Setup -------------------------
	//The Skeleton
	NodePath skeletonPath;
	Skeleton *skeleton;
	int64_t headBone;
	PoolIntArray neckBones; //array of neck bones
	int64_t shoulderLeftBone;
	int64_t upperArmLeftBone;
	int64_t forearmLeftBone;
	int64_t handLeftBone;
	int64_t shoulderRightBone;
	int64_t upperArmRightBone;
	int64_t forearmRightBone;
	int64_t handRightBone;
	PoolIntArray spineBones; //array of spine bones
	int64_t hipBone;
	int64_t thighLeftBone;
	int64_t shinLeftBone;
	int64_t footLeftBone;
	int64_t thighRightBone;
	int64_t shinRightBone;
	int64_t footRightBone;

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
	//Body Adjustments
	float spineCurve; //To prevent spines from bending the wrong way, we precurve it before solving the IK. This controls how much to precurve it
	float spineCurveAngle; //The angle to precurve it
	float spineTwist; //How much the spine tries to twist to follow the head when the hips are facing a differen direction
	float spineTwistOffset; //Where along the spine the twisting starts

	//Arm Adjustments
	float elbowAngleOffset; //Defines which way the elbows bend
	float elbowOffset; //Raises and lowers the elbows by twisting the whole arm
	float upperArmTwist; //The amount the shoulder twists with the forearm
	float forearmTwist; //The amount the forearm twists with the hand rotation
	Vector3 elbowPoleOffset; /*ADVANCED - Using IK to move a limb close to 180 degrees creates problems.
	This variable offets the arm in the T-pose such that the problem area is out of the way.*/
	Vector3 elbowPositionBias; /*ADVANCED - Raises and lowers the elbows based on the target's location's distance from the hand's T-pose position.
	First element in the vector controls the effect of differences in position along the x axis, the second is the y axis, and the third is the z axis*/
	float wristTwistInfluence; //ADVANCED - Controls how much twisting the wrist affects the elbow position
	float wristFlexInfluence; //ADVANCED - Controls how much flexing the wrist affects the elbow position

	//Leg Adjustments
	float kneeAngleOffset; //Defines which way the knees bend
	float kneeOffset; //Bows and unbows the knees by twisting the whole leg
	float thighTwist; //The amount the thigh twists with the lower leg
	float lowerLegTwist; //The amount the lower leg twists with the foot
	Vector3 kneePoleOffset; /*ADVANCED - Using IK to move a limb close to 180 degrees creates problems.
	This variable offets the leg in the T-pose such that the problem area is out of the way.*/
	Vector3 kneePositionBias; /*ADVANCED - Bows and unbows the knees based on the target's location's distance from the foot T-pose position.
	First element in the vector controls the effect of differences in position along the x axis, the second is the y axis, and the third is the z axis.*/
	float footTwistInfluence; //ADVANCED - Controls how much twisting the ankle affects the knee position
	float footFlexInfluence; //ADVANCED - Controls how much flexing the ankle affects the knee position

	//Hip Placement Adjustments - Only takes effect when there are no hip targets
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

	//General Settings ------------------
	bool manualUpdate;
	bool headTrackerEnabled;
	bool leftHandTrackerEnabled;
	bool rightHandTrackerEnabled;
	bool hipTrackerEnabled;
	bool leftFootTrackerEnabled;
	bool rightFootTrackerEnabled;

	//Internal Variables --------------------
	//IK Cache
	//If the transform didn't change, we skip recalculating IK
	Transform headTargetCached;
	Transform handLeftTargetCached;
	Transform handRightTargetCached;
	Transform hipTargetCached;
	Transform footLeftTargetCached;
	Transform footRightTargetCached;

	//tPose Cache
	float tPoseHeight;
	float tPoseHipsDist;
	Vector3 tPoseHipVector;
	Transform tPoseHeadLocal;
	Transform tPoseHipTransformLocalToHead;
	Transform tPoseHead;
	Transform tPoseHips;
	Transform tPoseHandLeft;
	Transform tPoseHandRight;
	Transform tPoseFootLeft;
	Transform tPoseFootRight;

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

	//Foot Placement Variables
	Transform centerFloor; //where the center of gravity is over the floor
	Transform leftPlant; //The planted foot position
	Transform rightPlant;
	Transform leftStepTarget;
	Transform rightStepTarget;
	Transform oldCenterFloor;
	bool leftStepping;
	bool rightStepping;
	bool falling;
	float leftProgress; //How far along the leg is in taking the step. Measured in time. Goes from 0 to leftStepTime.
	float rightProgress;
	float leftFootLength;
	float rightFootLength;
	float footScoreThreshold;
	float centerScoreThreshold;
	int leftStepType;
	int rightStepType;

	void dangleLeftFoot(Transform target);
	void dangleRightFoot(Transform target);
	void progressLeftStep(Transform balancedLeftStep, float delta, float centerVelocity, float heightOffsetRatio);
	void progressRightStep(Transform balancedRightStep, float delta, float centerVelocity, float heightOffsetRatio);

	static float smoothCurve(float number, float modifier = 0.5);
	static float sinusoidalInterpolation(float number);
	static Vector3 vectorRejection(Vector3 v, Vector3 normal);
	static void crossDot(Vector3 &cross, float &dot, Vector3 srcVector1, Vector3 srcVector2);
	static float safeACOS(float f);
	static float safeASIN(float f);

	//for SIFABRIK
	void calcSpineCurve();
	static void solveFABRIKPoints(std::vector<RenIK::BonePoint> &bonePoints, Vector3 rootPoint, Vector3 goal, float threshold, int loopLimit);
	static Vector3 fitPointToLine(Vector3 point, Vector3 goal, float length);

public:
	RenIK();

	void _ready();
	void _init(); // our initializer called by Godot

	virtual void _validate_property(PropertyInfo &property) const;
	void _notification(int p_what);
	void _enter_tree();
	void _exit_tree();
	static void _bind_methods();

	void _physics_process(float delta);
	void _process(float delta);

	void perform_torso_ik(float influence = 1);
	void perform_left_hand_ik(float influence = 1);
	void perform_right_hand_ik(float influence = 1);
	void perform_left_foot_ik(float influence = 1);
	void perform_right_foot_ik(float influence = 1);

	NodePath get_skeleton_path();
	void set_skeleton_path(NodePath newValue);

	void set_head_bone(String bone);
	void set_neck_bones(PoolStringArray bone);
	void set_shoulder_left_bone(String bone);
	void set_upper_arm_left_bone(String bone);
	void set_forearm_left_bone(String bone);
	void set_hand_left_bone(String bone);
	void set_shoulder_right_bone(String bone);
	void set_upper_arm_right_bone(String bone);
	void set_forearm_right_bone(String bone);
	void set_hand_right_bone(String bone);
	void set_spine_bones(PoolStringArray bone);
	void set_hip_bone(String bone);
	void set_thigh_left_bone(String bone);
	void set_shin_left_bone(String bone);
	void set_foot_left_bone(String bone);
	void set_thigh_right_bone(String bone);
	void set_shin_right_bone(String bone);
	void set_foot_right_bone(String bone);

	void set_head_bone(BoneId bone);
	void set_neck_bones(PoolIntArray bone);
	void set_shoulder_left_bone(BoneId bone);
	void set_upper_arm_left_bone(BoneId bone);
	void set_forearm_left_bone(BoneId bone);
	void set_hand_left_bone(BoneId bone);
	void set_shoulder_right_bone(BoneId bone);
	void set_upper_arm_right_bone(BoneId bone);
	void set_forearm_right_bone(BoneId bone);
	void set_hand_right_bone(BoneId bone);
	void set_spine_bones(PoolIntArray bone);
	void set_hip_bone(BoneId bone);
	void set_thigh_left_bone(BoneId bone);
	void set_shin_left_bone(BoneId bone);
	void set_foot_left_bone(BoneId bone);
	void set_thigh_right_bone(BoneId bone);
	void set_shin_right_bone(BoneId bone);
	void set_foot_right_bone(BoneId bone);

	int64_t get_hip_bone();
	int64_t get_head_bone();
	PoolIntArray get_neck_bones();
	int64_t get_shoulder_left_bone();
	int64_t get_upper_arm_left_bone();
	int64_t get_forearm_left_bone();
	int64_t get_hand_left_bone();
	int64_t get_shoulder_right_bone();
	int64_t get_upper_arm_right_bone();
	int64_t get_forearm_right_bone();
	int64_t get_hand_right_bone();
	PoolIntArray get_spine_bones();
	int64_t get_thigh_left_bone();
	int64_t get_shin_left_bone();
	int64_t get_foot_left_bone();
	int64_t get_thigh_right_bone();
	int64_t get_shin_right_bone();
	int64_t get_foot_right_bone();

	String get_hip_bone_name();
	String get_head_bone_name();
	PoolStringArray get_neck_bones_names();
	String get_shoulder_left_bone_name();
	String get_upper_arm_left_bone_name();
	String get_forearm_left_bone_name();
	String get_hand_left_bone_name();
	String get_shoulder_right_bone_name();
	String get_upper_arm_right_bone_name();
	String get_forearm_right_bone_name();
	String get_hand_right_bone_name();
	PoolStringArray get_spine_bones_names();
	String get_thigh_left_bone_name();
	String get_shin_left_bone_name();
	String get_foot_left_bone_name();
	String get_thigh_right_bone_name();
	String get_shin_right_bone_name();
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

	struct TrigSolution {
		Transform limb1;
		Transform limb2;
	};

	static TrigSolution solve_trig_ik(Transform parent, Transform limb1, Transform limb2, Transform leaf, Transform target, float angleOffset, float restOffset, float l1Twist, float l2Twist, Vector3 poleOffset, Vector3 targetPositionInfluence, float targetRotationInfluence);

	static std::vector<Transform> solve_isfabrik(Transform parent, std::vector<Transform> restTransforms, Transform leaf, Transform target, std::vector<Transform> curveTransforms, float curveDist, float maxDist, float threshold, int loopLimit, float twist, float twistOffset);

	static std::vector<Transform> solve_fabrik(Transform parent, std::vector<Transform> restTransforms, Transform leaf, Transform target, float threshold, int loopLimit, float twist, float twistOffset);

	void hip_place();
	void foot_place(float delta);
	//All used in leg trace
	void set_falling(bool falling);
	void set_manual_update(bool update_manually);
	void update();
};

#endif