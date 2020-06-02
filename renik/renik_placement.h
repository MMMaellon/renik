#ifndef RENIK_PLACEMENT_H
#define RENIK_PLACEMENT_H

#include "servers/physics_server.h"
#include <core/engine.h>
#include <scene/main/node.h>
#include <scene/3d/spatial.h>

struct RenIKPlacement : public Reference {
	GDCLASS(RenIKPlacement, Reference);

public:
#define FALLING 0
#define STANDING_TRANSITION -1
#define STANDING 1
#define STEPPING_TRANSITION -2
#define STEPPING 2
#define BACKSTEPPING_TRANSITION -3
#define BACKSTEPPING 3
#define LAYING_TRANSITION -4
#define LAYING 4
#define STRAFING_TRANSITION -5
#define STRAFING 5
#define OTHER_TRANSITION -6
#define OTHER 6

	struct Gait{

	};

	Gait forward_gait;
	Gait backward_gait;
	Gait sideways_gait;

	float spine_length = 1;
	float left_leg_length = 1;
	float right_leg_length = 1;
	float left_foot_length = 0.125;
	float right_foot_length = 0.125;
	//Hip Placement Adjustments
	float crouch_ratio = 0.5; //Crouching means bending over at the hip while keeping the spine straight
	float hunch_ratio = 0.5; //Hunching means bending over by arching the spine
	Vector3 hip_offset;
	float hip_follow_head_influence = 0;

	//Foot Placement Adjustments - Only takes effect when there are no foot targets
	//These are values when at the slowest walk speed
	float floor_offset = 0.05;
	float raycast_allowance = 0.05; //how far past the max length of the limb we'll still consider close enough
	float min_threshold = 0.025;
	float max_threshold = 0.05; //when all scaling stops and the legs just move faster
	float min_transition_speed = 0.04;
	float rotation_threshold = Math_PI / 4.0;
	float balance_threshold = 0.03;
	float center_of_balance_position = 0.75; //distance between hips and head that we'll call the center of balance. Usually at 25%, near the belly button

	float dangle_ratio = 0.9;
	float dangle_stiffness = 5;
	float dangle_angle = Math_PI / 8;
	float dangle_follow_head = 0.5;
	Vector3 left_hip_offset;
	Vector3 right_hip_offset;

	//Everything scales logarithmically
	float strafe_angle_limit = 0.75;
	float step_pace = 0.015;

	Transform hip; //relative to world
	Transform left_foot; //relative to world
	Transform right_foot; //relative to world


	const Basis foot_basis_offset = Basis(Vector3(-1, 0, 0), Vector3(0, -1, 0), Vector3(0, 0, 1));

	RenIKPlacement() { }

	void hip_place(float delta, Transform head, Transform left_foot, Transform right_foot, float twist);
	void foot_place(float delta, Transform head, Ref<World> w3d);
	void foot_place(float delta, Transform head, PhysicsDirectSpaceState::RayResult left_raycast, PhysicsDirectSpaceState::RayResult right_raycast, PhysicsDirectSpaceState::RayResult laying_raycast);
	void foot_place_redux(float delta, Transform head, PhysicsDirectSpaceState::RayResult left_raycast, PhysicsDirectSpaceState::RayResult right_raycast, PhysicsDirectSpaceState::RayResult laying_raycast);
	//All used in leg trace
	void set_falling(bool falling);
	void set_collision_mask_bit(int p_bit, bool p_value);
	bool get_collision_mask_bit(int p_bit) const;
	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const;
	void set_collide_with_areas(bool p_clip);
	bool is_collide_with_areas_enabled() const;
	void set_collide_with_bodies(bool p_clip);
	bool is_collide_with_bodies_enabled() const;

private:
#define LOOP_EXIT_GROUNDED 0
#define LOOP_LIFT 1
#define LOOP_ENTER_SADDLE 2
#define LOOP_EXIT_SADDLE 3
#define LOOP_LOWER 4
#define LOOP_ENTER_GROUNDED 5
	bool fall_override = false;
	bool prone_override = false;
	int walk_state = 0; // -2 is falling, -1 is transitioning to standing, 0 is stand state, 1 is transitioning to stepping, 2 is stepping
    float walk_transition_progress = 0;
    float step_progress = 0;
    Vector3 prevHead;
    uint32_t collision_mask = 1; //the first bit is on but all others are off
    bool collide_with_areas = false;
    bool collide_with_bodies = true;

	//Standing
	Transform left_stand;
	Transform right_stand;
	Transform left_stand_local;//local to ground
	Transform right_stand_local;//local to ground
	Spatial* left_ground = nullptr;
	Spatial *right_ground = nullptr;
	//Stepping
	Transform left_step;
	Transform right_step;
	Vector3 left_grounded_stop;
	Vector3 right_grounded_stop;
	const float standing_transition_duration = 0.25;
	const float stepping_transition_duration = 0.2;
	const float laying_transition_duration = 0.25;
	int left_loop_state = 0;
	int right_loop_state = 0;
	float loop_scaling = 0;

	//helpers
	bool
	is_balanced(Vector3 center, Vector3 forward, Vector3 ground_normal, Vector3 left, Vector3 right);
	void stand_foot(Transform &stand, Transform &stand_local, Spatial *ground);
	Transform dangle_foot(Transform head, float distance, float leg_length, Vector3 hip_offset);
	void initialize_loop(Vector3 velocity, Vector3 left_ground, Vector3 right_ground, bool left_grounded, bool right_grounded);
	void loop(Transform head, Vector3 velocity, Vector3 left_ground_pos, Vector3 left_normal, Vector3 right_ground_pos, Vector3 right_normal, bool left_grounded, bool right_grounded);
	void loop_foot(Transform &step, Transform &stand, Transform &stand_local, Spatial *ground, int &loop_state, Vector3 &grounded_stop, Transform head, float leg_length, float foot_length, Vector3 velocity, float loop_scaling, float step_progress, Vector3 ground_pos, Vector3 ground_normal);
	void step_direction(Vector3 forward, Vector3 velocity, Vector3 left_ground, Vector3 right_ground, bool left_grounded, bool right_grounded);
	int get_loop_state(float loop_state_scaling, float loop_progress, float &loop_state_progress);
	};

#endif //RENIK_PLACEMENT_H