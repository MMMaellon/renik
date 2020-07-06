#ifndef RENIK_H
#define RENIK_H
#ifndef _3D_DISABLED

#include "renik/renik_chain.h"
#include "renik/renik_helper.h"
#include "renik/renik_limb.h"
#include "renik/renik_placement.h"
#include "servers/physics_server.h"
#include <core/engine.h>
#include <core/variant.h>
#include <scene/3d/skeleton.h>
#include <scene/main/node.h>
#include <memory>
#include <vector>

class RenIK : public Node {
	GDCLASS(RenIK, Node);

public:

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
	void reset_chain(Ref<RenIKChain> chain);
	void reset_limb(Ref<RenIKLimb> limb);

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
	Vector3 get_shoulder_pole_offset();
	void set_shoulder_pole_offset(Vector3 euler);

	bool get_use_editor_speed();
	void set_use_editor_speed(bool enable);

	//placement
	void set_falling(bool falling);
	void enable_solve_ik_every_frame(bool automatically_update_ik);
	void enable_foot_placement(bool enabled);
	void enable_hip_placement(bool enabled);
	void set_collision_mask_bit(int p_bit, bool p_value);
	bool get_collision_mask_bit(int p_bit) const;
	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const;
	void set_collide_with_areas(bool p_clip);
	bool is_collide_with_areas_enabled() const;
	void set_collide_with_bodies(bool p_clip);
	bool is_collide_with_bodies_enabled() const;

	static std::pair<float, float> trig_angles(Vector3 const &length1, Vector3 const &length2, Vector3 const &length3);
	static Map<BoneId, Quat> solve_trig_ik(Ref<RenIKLimb> limb, Transform limb_parent_transform, Transform target);

	static Map<BoneId, Basis> solve_trig_ik_redux(Ref<RenIKLimb> limb, Transform limb_parent_transform, Transform target);

	static Map<BoneId, Quat> solve_ifabrik(Ref<RenIKChain> chain, Transform chain_parent_transform, Transform target, float threshold, int loopLimit);

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
	RenIKPlacement placement;
	String head_bone_name;
	String hip_bone_name;
	String hand_left_bone_name;
	String hand_right_bone_name;
	String foot_left_bone_name;
	String foot_right_bone_name;

	BoneId hip = -1;
	BoneId head = -1;

	Ref<RenIKChain> spine_chain;
	Ref<RenIKLimb> limb_arm_left;
	Ref<RenIKLimb> limb_arm_right;
	Ref<RenIKLimb> limb_leg_left;
	Ref<RenIKLimb> limb_leg_right;
	float shoulder_influence = 0.15;
	bool left_shoulder_enabled = false;
	bool right_shoulder_enabled = false;
	Vector3 left_shoulder_offset;
	Vector3 right_shoulder_offset;
	Vector3 left_shoulder_pole_offset;
	Vector3 right_shoulder_pole_offset;

	//General Settings ------------------
	bool hip_placement = true;
	bool foot_placement = true;
	bool headTrackerEnabled = true;
	bool leftHandTrackerEnabled = true;
	bool rightHandTrackerEnabled = true;
	bool hipTrackerEnabled = true;
	bool leftFootTrackerEnabled = true;
	bool rightFootTrackerEnabled = true;

	Vector<BoneId> calculate_bone_chain(BoneId root, BoneId leaf);
};

#endif
#endif