#ifndef RENIK_PLACEMENT_H
#define RENIK_PLACEMENT_H

#include "servers/physics_server.h"
#include <core/engine.h>
#include <scene/main/node.h>

struct RenIKPlacement : public Reference {
	GDCLASS(RenIKPlacement, Reference);

public:
	float spine_length;
	float left_leg_length;
	float right_leg_length;
	Vector3 left_hip_offset;
	Vector3 right_hip_offset;
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
	float rotation_threshold;
	float balance_threshold;
	float center_of_balance_position; //distance between hips and head that we'll call the center of balance. Usually at 25%, near the belly button

	float dangle_height;
	float dangle_stiffness;

	//Everything scales logarithmically
	float contact_scale;
	float height_scale;
	float loop_scale;
	float time_scale;

	Transform hip; //relative to world
	Transform left_foot; //relative to world
	Transform right_foot; //relative to world

	RenIKPlacement() {
		crouch_amount = 0;
		hunch_amount = 0;
		hip_turn_speed = 0;
		hip_turn_limit = 0;
		hip_follow_head_influence = 0;
		calculate_speed = false;
		floor_offset = 0;
		raycast_allowance = 0.05;
		contact_length = 0;
		followthru_angle = 0;
		buildup_angle = 0;
		loop_ratio = 0.5;
		step_height = 0;
		min_threshold = 0.1;
		max_threshold = 1.0;
		raycast_threshold = 0;
		rotation_threshold = Math_PI / 2.0;
		balance_threshold = 0.25;
		center_of_balance_position = 0.75;
		dangle_height = 0.2;
		dangle_stiffness = 0.2;
		contact_scale = 0;
		height_scale = 0;
		loop_scale = 0;
		time_scale = 0;
	}

	void hip_place(float delta, Transform head, Transform left_foot, Transform right_foot, float twist);
	void foot_place(float delta, Transform head, Ref<World> w3d);
	void foot_place(float delta, Transform head, PhysicsDirectSpaceState::RayResult left_raycast, PhysicsDirectSpaceState::RayResult right_raycast, PhysicsDirectSpaceState::RayResult laying_raycast);
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
	bool fall_override;
	int walk_state = 0; // -2 is falling, -1 is transitioning to standing, 0 is stand state, 1 is transitioning to stepping, 2 is stepping
#define FALLING 0
#define STANDING_TRANSITION -1
#define STANDING 1
#define STEPPING_TRANSITION -2
#define STEPPING 2
#define LAYING_TRANSITION -3
#define LAYING 3
#define OTHER_TRANSITION -4
#define OTHER 4
    float walk_transition_duration = 100;
    float walk_transition_progress = 0;

    float ground_speed = 0;
    float step_progress = 0;
    Vector3 prevHead; //local to midpoint between grounds
	Transform standLeft; //relative to world
    Transform standRight; //relative to world
    Transform groundedLeft; //relative to ground
    Transform groundedRight; //relative to ground
    Transform lastLeft; //relative to head (no rotation)
    Transform lastRight; //relative to head (no rotation)
    Transform groundLeft;
    Transform groundRight;
    // Spatial *groundLeftPointer;
    // Spatial *groundRightPointer;
    uint32_t collision_mask = 1; //the first bit is on but all others are off
    bool collide_with_areas = false;
    bool collide_with_bodies = true;
};

#endif //RENIK_PLACEMENT_H