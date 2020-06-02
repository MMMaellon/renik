#include "renik_placement.h"
#include "renik_helper.h"

#ifndef _3D_DISABLED

void RenIKPlacement::hip_place(float delta, Transform head, Transform left_foot, Transform right_foot, float twist) {
	Vector3 left_middle = (left_foot.translated(Vector3(0, 0, left_foot_length / 2))).origin;
	Vector3 right_middle = (right_foot.translated(Vector3(0, 0, right_foot_length / 2))).origin;
	float left_distance = left_middle.distance_squared_to(head.origin);
	float right_distance = right_middle.distance_squared_to(head.origin);
	Vector3 foot_median = left_middle.linear_interpolate(right_middle, 0.5);
	Vector3 foot = left_distance > right_distance ? left_middle : right_middle;
	Vector3 foot_direction = (foot - head.origin).project(foot_median - head.origin);

	hip.basis = RenIKHelper::align_vectors(Vector3(0, -1, 0), foot_direction);
	Vector3 head_forward = head.basis.inverse()[2];
	Vector3 feet_forward = (left_foot.interpolate_with(right_foot, 0.5)).basis[2];
	Vector3 hip_forward = feet_forward.linear_interpolate(head_forward, 0.5);

	Vector3 hip_y = -foot_direction.normalized();
	Vector3 hip_z = RenIKHelper::vector_rejection(hip_forward.normalized(), hip_y).normalized();
	Vector3 hip_x = hip_y.cross(hip_z).normalized();
	hip.basis = Basis(hip_x, hip_y, hip_z).inverse().orthonormalized();

	float crouch_distance = head.origin.distance_to(foot) * crouch_ratio;
	float extra_hip_distance = hip_offset.length() - crouch_distance;

	hip.origin = head.origin;
	hip.translate(crouch_distance * hip_offset.normalized());
	if (extra_hip_distance > 0) {
		hip.translate(Vector3(0, 0, -extra_hip_distance * (1 / hunch_ratio)));
	}
}

/*foot_place requires raycasting unless a raycast result is provided.
	Raycasting needs to happen inside of a physics update
*/
void RenIKPlacement::foot_place(float delta, Transform head, Ref<World> w3d) {
	ERR_FAIL_COND(w3d.is_null());

	PhysicsDirectSpaceState *dss = PhysicsServer::get_singleton()->space_get_direct_state(w3d->get_space());
	ERR_FAIL_COND(!dss);

	PhysicsDirectSpaceState::RayResult left_raycast;
	PhysicsDirectSpaceState::RayResult right_raycast;
	PhysicsDirectSpaceState::RayResult laying_raycast;

	float startOffset = (spine_length * -center_of_balance_position) / sqrt(2);
	Vector3 leftStart = head.translated(Vector3(0, startOffset, startOffset) + left_hip_offset).origin;
	Vector3 rightStart = head.translated(Vector3(0, startOffset, startOffset) + right_hip_offset).origin;
	Vector3 leftStop = head.origin + Vector3(0, (-spine_length - left_leg_length - floor_offset) * (1 + raycast_allowance) + left_hip_offset[1], 0) + head.basis.xform(left_hip_offset);
	Vector3 rightStop = head.origin + Vector3(0, (-spine_length - right_leg_length - floor_offset) * (1 + raycast_allowance) + right_hip_offset[1], 0) + head.basis.xform(right_hip_offset);

	bool left_collided = dss->intersect_ray(leftStart, leftStop, left_raycast, Set<RID>(), collision_mask, collide_with_bodies, collide_with_areas);
	bool right_collided = dss->intersect_ray(rightStart, rightStop, right_raycast, Set<RID>(), collision_mask, collide_with_bodies, collide_with_areas);
	bool laying_down = dss->intersect_ray(head.origin, head.origin - Vector3(0, spine_length + floor_offset, 0), laying_raycast, Set<RID>(), collision_mask, collide_with_bodies, collide_with_areas);
	if (!left_collided) {
		left_raycast.collider = nullptr;
	}
	if (!right_collided) {
		right_raycast.collider = nullptr;
	}
	if (!laying_down) {
		laying_raycast.collider = nullptr;
	}
	Vector3 left_offset = (leftStart - leftStop).normalized() * floor_offset * left_leg_length;
	Vector3 right_offset = (rightStart - rightStop).normalized() * floor_offset * right_leg_length;
	Vector3 laying_offset = Vector3(0,floor_offset * (left_leg_length + right_leg_length) / 2,0);
	left_raycast.position += left_offset;
	right_raycast.position += right_offset;
	laying_raycast.position += laying_offset;
	// foot_place(delta, head, left_raycast, right_raycast, laying_raycast);
	foot_place_redux(delta, head, left_raycast, right_raycast, laying_raycast);
}

Transform RenIKPlacement::dangle_foot(Transform head, float distance, float leg_length, Vector3 hip_offset) {
	Transform foot;
	Basis upright_head = RenIKHelper::align_vectors(Vector3(0, 1, 0), head.basis[1]).slerp(Quat(), 1 - dangle_follow_head);
	Vector3 dangle_vector = Vector3(0, spine_length + leg_length, 0) - hip_offset;
	Basis dangle_basis = head.basis * upright_head;
	foot.basis = dangle_basis * Basis(Vector3(1, 0, 0), dangle_angle);
	foot.origin = head.origin + dangle_basis.xform(-dangle_vector);
	return foot;
}

void RenIKPlacement::initialize_loop(Vector3 velocity, Vector3 left_ground, Vector3 right_ground, bool left_grounded, bool right_grounded){
	if(left_grounded && right_grounded){
		Vector3 foot_diff = left_foot.origin - right_foot.origin;
		float dot = foot_diff.dot(velocity);
		if (dot == 0) {
			float left_dist = left_foot.origin.distance_squared_to(left_ground);
			float right_dist = right_foot.origin.distance_squared_to(right_ground);
			if(left_dist < right_dist){
				//left foot more off balance
				step_progress = 0;
			} else {
				//right foot more off balance
				step_progress = 0.5;
			}
		} else if (dot > 0) {
			//left foot in front
			step_progress = 0;
		} else {
			//right foot in front
			step_progress = 0.5;
		}
	} else if (left_grounded){
		step_progress = 0;
	} else {
		step_progress = 0.5;
	}
}

int RenIKPlacement::get_loop_state(float loop_state_scaling, float loop_progress, float &loop_state_progress) {
	int state = -1;

	// loop_state_scaling = sqrtf(loop_state_scaling);
	float ground_time = 20 - 20 * loop_state_scaling;
	float lift_time = 10 + 5 * loop_state_scaling;
	float enter_saddle_time = 10 + 5 * loop_state_scaling;
	float exit_saddle_time = 10 + 5 * loop_state_scaling;
	float lower_time = 10 + 5 * loop_state_scaling;
	float total_time = ground_time + lift_time + enter_saddle_time + exit_saddle_time + lower_time + ground_time;

	float progress_time = loop_progress * total_time;

	if (progress_time < ground_time) {
		state = LOOP_EXIT_GROUNDED;
		loop_state_progress = (progress_time) / ground_time;
	} else if (progress_time < ground_time + lift_time) {
		state = LOOP_LIFT;
		loop_state_progress = (progress_time - ground_time) / lift_time;
	} else if (progress_time < ground_time + lift_time + enter_saddle_time) {
		state = LOOP_ENTER_SADDLE;
		loop_state_progress = (progress_time - ground_time - lift_time) / enter_saddle_time;
	} else if (progress_time < ground_time + lift_time + enter_saddle_time + exit_saddle_time) {
		state = LOOP_EXIT_SADDLE;
		loop_state_progress = (progress_time - ground_time - lift_time - enter_saddle_time) / exit_saddle_time;
	} else if (progress_time < ground_time + lift_time + enter_saddle_time + exit_saddle_time + lower_time) {
		state = LOOP_LOWER;
		loop_state_progress = (progress_time - ground_time - lift_time - enter_saddle_time - exit_saddle_time) / lower_time;
	} else {
		state = LOOP_ENTER_GROUNDED;
		loop_state_progress = (progress_time - ground_time - lift_time - enter_saddle_time - exit_saddle_time - lower_time) / ground_time;
	}

	return state;
}

void RenIKPlacement::loop_foot(Transform &step, Transform &stand, Transform &stand_local, Spatial *ground, int &loop_state, Vector3 &grounded_stop, Transform head, float leg_length, float foot_length, Vector3 velocity, float loop_scaling, float step_progress, Vector3 ground_pos, Vector3 ground_normal) {
	Quat upright_foot = RenIKHelper::align_vectors(ground_normal, head.basis[1]);
	if(ground_normal.dot(head.basis[1]) > cos(rotation_threshold) && ground_normal.dot(Vector3(0,1,0)) > cos(rotation_threshold)){
		upright_foot = Quat();
	}
	Vector3 ground_velocity = RenIKHelper::vector_rejection(velocity, ground_normal);
	if(ground_velocity.length() > max_threshold * step_pace){
		ground_velocity = ground_velocity.normalized() * max_threshold * step_pace;
	}
	float loop_state_progress = 0;
	loop_state = get_loop_state(loop_scaling, step_progress, loop_state_progress);
	float head_distance = head.origin.distance_to(ground_pos);
	float ease_scaling = loop_scaling * loop_scaling * loop_scaling * loop_scaling;//ease the growth a little
	float vertical_scaling = head_distance * ease_scaling;
	float horizontal_scaling = leg_length * ease_scaling;
	Transform grounded_foot = Transform(head.basis * upright_foot, ground_pos);
	Transform saddle_foot = Transform(grounded_foot.basis.rotated_local(Vector3(1, 0, 0), ease_scaling * dangle_angle), ground_pos + ground_normal * vertical_scaling * 0.1 + ground_normal * head_distance * 0.01);
	Transform lifted_foot = Transform(grounded_foot.basis.rotated_local(Vector3(1, 0, 0), ease_scaling * Math_PI / 2), ground_pos + ground_normal * vertical_scaling * 0.25 + ground_normal * head_distance * 0.025 - ground_velocity.normalized() * horizontal_scaling * 0.5);
	Transform lowering_foot = Transform(grounded_foot.basis.rotated_local(Vector3(1, 0, 0), ease_scaling * Math_PI / -4), ground_pos + ground_normal * vertical_scaling * 0.15 + ground_normal * head_distance * 0.0 + ground_velocity.normalized() * horizontal_scaling * 0.25);

	// loop_state = LOOP_EXIT_GROUNDED;
	switch (loop_state) {
		case LOOP_EXIT_GROUNDED: {
			//stick to where it landed
			if (ground != nullptr) {
				step = ground->get_global_transform() * stand_local;
				step.basis = step.basis.get_rotation_quat();
				stand = step;
			} else {
				step = stand;
			}

			float step_distance = step.origin.distance_to(ground_pos);
			Transform lean_offset;
			float lean_angle = step_distance * 2.5;
			lean_angle = lean_angle > Math_PI / 4 ? Math_PI / 4 : lean_angle;
			lean_offset.origin = Vector3(0, foot_length * sin(lean_angle), 0);
			lean_offset.rotate_basis(Vector3(1, 0, 0), lean_angle);
			step *= lean_offset;

			grounded_stop = step.origin;

			break;
		}
		case LOOP_ENTER_GROUNDED:{
			//stick to where it landed
			if (ground != nullptr) {
				step = ground->get_global_transform() * stand_local;
				step.basis = step.basis.get_rotation_quat();
				stand = step;
			} else {
				step = stand;
			}

			float step_distance = step.origin.distance_to(ground_pos);
			Transform lean_offset;
			float lean_angle = step_distance * 2.5;
			lean_angle = lean_angle > Math_PI / 4 ? Math_PI / 4 : lean_angle;
			lean_offset.origin = Vector3(0, foot_length * sin(lean_angle), 0);
			lean_offset.rotate_basis(Vector3(1, 0, 0), lean_angle);
			step *= lean_offset;

			grounded_stop = step.origin;

			break;
		}
		case LOOP_LIFT:{
			float step_distance = step.origin.distance_to(ground_pos);
			Transform lean_offset;
			float lean_angle = step_distance * 2.5;
			lean_angle = lean_angle > Math_PI / 4 ? Math_PI / 4 : lean_angle;

			step.basis = grounded_foot.basis.rotated_local(Vector3(1,0,0), lean_angle).slerp(lifted_foot.basis, loop_state_progress);
			step.origin = grounded_stop.cubic_interpolate(lifted_foot.origin, grounded_stop - ground_velocity * horizontal_scaling, lifted_foot.origin + ground_normal * vertical_scaling, loop_state_progress);
			break;
		}
		case LOOP_ENTER_SADDLE:
			step.basis = lifted_foot.basis.slerp(saddle_foot.basis, loop_state_progress);
			step.origin = lifted_foot.origin.cubic_interpolate(saddle_foot.origin, lifted_foot.origin - ground_normal * vertical_scaling, saddle_foot.origin + ground_velocity * leg_length, loop_state_progress);
			break;
		case LOOP_EXIT_SADDLE:
			step.basis = saddle_foot.basis.slerp(lowering_foot.basis, loop_state_progress);
			step.origin = saddle_foot.origin.cubic_interpolate(lowering_foot.origin, saddle_foot.origin - ground_velocity * horizontal_scaling, lowering_foot.origin - ground_normal * vertical_scaling, loop_state_progress);
			break;
		case LOOP_LOWER:
			step.basis = lowering_foot.basis.slerp(grounded_foot.basis, loop_state_progress);
			step.origin = lowering_foot.origin.cubic_interpolate(grounded_foot.origin, lowering_foot.origin + ground_normal * vertical_scaling, grounded_foot.origin - ground_velocity * horizontal_scaling, loop_state_progress);
			break;
	}

	if (loop_state != LOOP_EXIT_GROUNDED && loop_state != LOOP_ENTER_GROUNDED) {
		//update standing positions to ensure a smooth transition to standing
		stand.origin = ground_pos;
		stand.basis = grounded_foot.basis;
		if (ground != nullptr) {
			stand_local = ground->get_global_transform().affine_inverse() * stand;
			stand_local.basis = stand_local.basis.get_rotation_quat();
		} else {
			stand_local = stand;
		}
		if(walk_state != LOOP_LIFT){
			grounded_stop = step.origin;
		} else {
			grounded_stop = grounded_stop.linear_interpolate(ground_pos, 0.1 + 0.4 * loop_scaling);
		}
	}
}

void RenIKPlacement::loop(Transform head, Vector3 velocity, Vector3 left_ground_pos, Vector3 left_normal, Vector3 right_ground_pos, Vector3 right_normal, bool left_grounded, bool right_grounded) {
	float stride_speed = step_pace * velocity.length() / ((left_leg_length + right_leg_length) / 2);
	stride_speed = log( 1 + stride_speed);
	stride_speed = stride_speed > max_threshold ? max_threshold : stride_speed;
	stride_speed = stride_speed < min_threshold ? min_threshold : stride_speed;
	step_progress = Math::fmod((step_progress + stride_speed), 1.0f) ;
	float new_loop_scaling = max_threshold > min_threshold ? (stride_speed - min_threshold) / (max_threshold - min_threshold) : 0;
	loop_scaling = (loop_scaling * 0.9 + new_loop_scaling * 0.1);

	Quat left_upright_foot = RenIKHelper::align_vectors(left_normal, head.basis[1]);
	Quat right_upright_foot = RenIKHelper::align_vectors(right_normal, head.basis[1]);
	Transform left_grounded_foot = Transform(head.basis * left_upright_foot, left_ground_pos);
	Transform right_grounded_foot = Transform(head.basis * right_upright_foot, right_ground_pos);

	if (left_grounded) {
		loop_foot(left_step, left_stand, left_stand_local, left_ground, left_loop_state, left_grounded_stop, head, left_leg_length, left_foot_length, velocity, loop_scaling, step_progress, left_ground_pos, left_normal);
	} else {
		Transform left_dangle = dangle_foot(head, (spine_length + left_leg_length) * dangle_ratio, left_leg_length, left_hip_offset);
		left_foot.basis = left_foot.basis.slerp(left_dangle.basis, 1.0f - (1.0f / dangle_stiffness));
		left_foot.origin = RenIKHelper::log_clamp(left_foot.origin, left_dangle.origin, 1.0 / dangle_stiffness);
	}

	if (right_grounded) {
		loop_foot(right_step, right_stand, right_stand_local, right_ground, right_loop_state, right_grounded_stop, head, right_leg_length, right_foot_length, velocity, loop_scaling, Math::fmod((step_progress + 0.5f), 1.0f), right_ground_pos, right_normal);
	} else {
		Transform right_dangle = dangle_foot(head, (spine_length + right_leg_length) * dangle_ratio, right_leg_length, right_hip_offset);
		right_foot.basis = right_foot.basis.slerp(right_dangle.basis, 1.0f - (1.0f / dangle_stiffness));
		right_foot.origin = RenIKHelper::log_clamp(right_foot.origin, right_dangle.origin, 1.0 / dangle_stiffness);
	}
}

void RenIKPlacement::step_direction(Vector3 forward, Vector3 velocity, Vector3 left_ground, Vector3 right_ground, bool left_grounded, bool right_grounded) {
	float dot = velocity.dot(forward);
	if (Math::abs(dot) > strafe_angle_limit) {
		if (walk_state != STRAFING && walk_state != STRAFING_TRANSITION) {
			walk_state = STRAFING_TRANSITION;
			walk_transition_progress = stepping_transition_duration; //In units of loop progression
			initialize_loop(velocity, left_ground, right_ground, left_grounded, right_grounded);
		}
	} else if (dot < 0) {
		if (walk_state != BACKSTEPPING && walk_state != BACKSTEPPING_TRANSITION) {
			walk_state = BACKSTEPPING_TRANSITION;
			walk_transition_progress = stepping_transition_duration; //In units of loop progression
			initialize_loop(velocity, left_ground, right_ground, left_grounded, right_grounded);
		}
	} else {
		if (walk_state != STEPPING && walk_state != STEPPING_TRANSITION) {
			walk_state = STEPPING_TRANSITION;
			walk_transition_progress = stepping_transition_duration; //In units of loop progression
			initialize_loop(velocity, left_ground, right_ground, left_grounded, right_grounded);
		}
	}
}

void RenIKPlacement::stand_foot(Transform &stand, Transform &stand_local, Spatial *ground) {
		if (ground == ground) {
			stand = ground->get_global_transform() * stand_local;
			stand.basis = stand.basis.get_rotation_quat();
		} else {
			ground = ground;
			stand_local = ground->get_global_transform().affine_inverse() * stand;
			stand_local.basis = stand_local.basis.get_rotation_quat();
		}
}

/*
Step 1: Figure out what state we're in.
If we're far from the ground, we're FALLING
If we're too close to the ground, we're LAYING
If we're moving too fast forward or off-balance, we're STEPPING
If we're moving too fast backward, we're BACKSTEPPING
Else we're just STANDING

There are transition states between all these base states

Step 2: Based on the state we place the feet.
FALLING: Dangle the feet down.
LAYING: Align feet with the rejection of our head's -z axis on the ground normal.
STEPPING: DO THE LOOP
STANDING: If any foot is in the air we lerp it to where the raycast hit the ground.
If any foot was already on the ground, we leave it there.
Transitions to the STANDING state is only possible from the stepping state, so we'll know
if a foot is already on the ground based on where it was in the stepping loop.

THE LOOP: Made up of 6 parts
1. The push - From when foot is on the ground directly below the center of gravity until it lifts off the ground.
2. The kick - Foot kicks up to the furthest point backward of the loop.
3. Enter saddle - Foot swings down to point directly below center of gravity. It's still above the ground.
4. Exit saddle - Foot continues swing up to the furthest point forward of the loop.
5. The buildup - Foot gains speed as it comes in contact with the ground.
6. The landing - Foot touches down and sticks the ground until it's under the center of gravity

Parts 1 and 6 are made by keeping the foot in place in world space.
Parts 2-5 are animated with bezier curves with continuous tangents between parts.
Parts 5 and 2 have vertical tangents, 2 and 3 have horizontal tangents, 3 and 4 have vertical tangents

At high speeds the durations of parts 1 and 6 will be 0 which makes the loop an uninterrupted loop of bezier curves
At low speeds the durations of 2 and 5 will be almost 0 (though I don't plan to go all the way to 0)

Progress through loop will be represented with a float that goes from 0.0 to 1.0 where 0.0 is the beginning of part 1
and 1.0 is the end of part 6.
The progress from 0.0 to 1.0 happens smoothly and linearly with movement speed.
What range of numbers represents each part of the loop changes dynamically with movement speed.
*/
void RenIKPlacement::foot_place_redux(float delta, Transform head, PhysicsDirectSpaceState::RayResult left_raycast, PhysicsDirectSpaceState::RayResult right_raycast, PhysicsDirectSpaceState::RayResult laying_raycast) {
	//Step 1: Find the proper state
	//Note we always enter transition states when possible
	Vector3 velocity = (head.origin - prevHead) / delta;
	Vector3 left_velocity;
	Vector3 right_velocity;
	if (left_raycast.collider != nullptr) {
		left_velocity = RenIKHelper::vector_rejection(velocity, left_raycast.normal);
	} else {
		left_velocity = RenIKHelper::vector_rejection(velocity, Vector3(0, 1, 0));
	}
	if (right_raycast.collider != nullptr) {
		right_velocity = RenIKHelper::vector_rejection(velocity, right_raycast.normal);
	} else {
		right_velocity = RenIKHelper::vector_rejection(velocity, Vector3(0, 1, 0));
	}

	float effective_min_threshold = min_threshold * ((left_leg_length + right_leg_length) / 2) / step_pace;
	if (!left_raycast.collider && !right_raycast.collider && !laying_raycast.collider || fall_override) {
		//If none of the raycasts hit anything then there isn't any ground to stand on
		walk_state = FALLING;
		walk_transition_progress = 0;
	} else if (laying_raycast.collider || prone_override) {
		//If we're close enough for the laying raycast to trigger and we aren't already laying down
		//transition to laying down
		if (walk_state != LAYING && walk_state != LAYING_TRANSITION) {
			walk_state = LAYING_TRANSITION;
			walk_transition_progress = laying_transition_duration; //In units of loop progression
		}
	} else {
		Vector3 ground = left_raycast.collider != nullptr ? left_raycast.position : right_raycast.position;
		Vector3 ground_normal = left_raycast.collider != nullptr ? left_raycast.normal : right_raycast.normal;
		switch(walk_state){
			case STANDING:{//brackets to stop declarations from breaking case labels
				//test that the feet aren't twisted in weird ways
				Vector3 forward = RenIKHelper::vector_rejection(head.basis[2], ground_normal).normalized();
				Vector3 left_forward = left_stand.basis[2];
				Vector3 right_forward = right_stand.basis[2];
				Vector3 upward = ground_normal;
				Vector3 left_upward = left_stand.basis[1];
				Vector3 right_upward = right_stand.basis[1];
				
				if (left_velocity.length() > effective_min_threshold
				|| right_velocity.length() > effective_min_threshold
				|| !is_balanced(head.origin, head.basis.xform(Vector3(0, 0, 1)), ground_normal, left_foot.origin, right_foot.origin)
				|| (left_raycast.collider != nullptr && left_foot.origin.distance_squared_to(left_raycast.position) > balance_threshold * (left_leg_length + right_leg_length) / 2)
				|| (right_raycast.collider != nullptr && right_foot.origin.distance_squared_to(right_raycast.position) > balance_threshold * (left_leg_length + right_leg_length) / 2)
				|| (left_raycast.collider != nullptr && (Spatial *)left_raycast.collider != left_ground)
				|| (right_raycast.collider != nullptr && (Spatial *)right_raycast.collider != right_ground)
				|| forward.dot(left_forward) < cos(rotation_threshold)
				|| forward.dot(right_forward) < cos(rotation_threshold)
				|| upward.dot(left_upward) < cos(rotation_threshold)
				|| upward.dot(right_upward) < cos(rotation_threshold)
				) {
					step_direction(head.basis[0], velocity, left_raycast.position, right_raycast.position, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
				}
				break;
			}
			case STANDING_TRANSITION:
				if (left_velocity.length() > effective_min_threshold
				|| right_velocity.length() > effective_min_threshold
				|| (left_raycast.collider != nullptr && (Spatial *)left_raycast.collider != left_ground)
				|| (right_raycast.collider != nullptr && (Spatial *)right_raycast.collider != right_ground)
				) {
					step_direction(head.basis[0], velocity, left_raycast.position, right_raycast.position, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
				}
				break;
			case STEPPING:
			case STEPPING_TRANSITION:
			case BACKSTEPPING:
			case BACKSTEPPING_TRANSITION:
			case STRAFING:
			case STRAFING_TRANSITION:
				if (left_velocity.length() < effective_min_threshold
				&& right_velocity.length() < effective_min_threshold
				&& walk_transition_progress == 0
				&& (left_raycast.collider == nullptr || left_foot.origin.distance_squared_to(left_raycast.position) < balance_threshold * (left_leg_length + right_leg_length) / 2)
				&& (right_raycast.collider == nullptr || right_foot.origin.distance_squared_to(right_raycast.position) < balance_threshold * (left_leg_length + right_leg_length) / 2)
				) {
					walk_state = STANDING_TRANSITION;
					walk_transition_progress = standing_transition_duration; //In units of loop progression
				} else {
					step_direction(head.basis[0], velocity, left_raycast.position, right_raycast.position, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
				}
				break;
			default:
				step_direction(head.basis[0], velocity, left_raycast.position, right_raycast.position, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
				break;
		}
	}

	float stride_speed = step_pace * velocity.length() / ((left_leg_length + right_leg_length) / 2);
	walk_transition_progress -= stride_speed < min_transition_speed ? min_transition_speed : stride_speed;
	walk_transition_progress = walk_transition_progress > 0 ? walk_transition_progress : 0;
	if (walk_transition_progress == 0 && walk_state < 0) {
		walk_state *= -1;
	}

	//Step 2: Place foot based on state
	switch(walk_state){
		case FALLING:{
			Transform left_dangle = dangle_foot(head, (spine_length + left_leg_length) * dangle_ratio, left_leg_length, left_hip_offset);
			Transform right_dangle = dangle_foot(head, (spine_length + right_leg_length) * dangle_ratio, right_leg_length, right_hip_offset);

			left_foot.basis = left_foot.basis.slerp(left_dangle.basis * foot_basis_offset, 1.0f - (1.0f / dangle_stiffness));
			left_foot.origin = RenIKHelper::log_clamp(left_foot.origin, left_dangle.origin, 1.0 / dangle_stiffness);

			right_foot.basis = right_foot.basis.slerp(right_dangle.basis * foot_basis_offset, 1.0f - (1.0f / dangle_stiffness));
			right_foot.origin = RenIKHelper::log_clamp(right_foot.origin, right_dangle.origin, 1.0 / dangle_stiffness);
			break;
		}
		case STANDING_TRANSITION:
		case STANDING :{
			left_ground = (Spatial *)left_raycast.collider;
			right_ground = (Spatial *)right_raycast.collider;
			float left_distance = head.origin.distance_to(right_raycast.position);//dangle foot according to where the other foot is
			float right_distance = head.origin.distance_to(left_raycast.position);

			if (left_ground != nullptr) {
				stand_foot(left_stand, left_stand_local, left_ground);
				left_foot = Transform(left_stand.basis * foot_basis_offset, left_stand.origin).interpolate_with(left_foot, walk_transition_progress / standing_transition_duration);
				left_grounded_stop = left_stand.origin;
			} else {
				Transform left_dangle = dangle_foot(head, (spine_length + left_leg_length) * dangle_ratio, left_leg_length, left_hip_offset);
				left_foot.basis = left_foot.basis.slerp(left_dangle.basis * foot_basis_offset, 1.0f - (1.0f / dangle_stiffness));
				left_foot.origin = RenIKHelper::log_clamp(left_foot.origin, left_dangle.origin, 1.0 / dangle_stiffness);
			}

			if (right_ground != nullptr) {
				stand_foot(right_stand, right_stand_local, right_ground);
				right_foot = Transform(right_stand.basis * foot_basis_offset, right_stand.origin).interpolate_with(right_foot, walk_transition_progress / standing_transition_duration);
				right_grounded_stop = right_stand.origin;
			} else {
				Transform right_dangle = dangle_foot(head, (spine_length + right_leg_length) * dangle_ratio, right_leg_length, right_hip_offset);
				right_foot.basis = right_foot.basis.slerp(right_dangle.basis * foot_basis_offset, 1.0f - (1.0f / dangle_stiffness));
				right_foot.origin = RenIKHelper::log_clamp(right_foot.origin, right_dangle.origin, 1.0 / dangle_stiffness);
			}

			break;
		}
		case STEPPING_TRANSITION:
		case STEPPING:
		case BACKSTEPPING_TRANSITION:
		case BACKSTEPPING:
		case STRAFING_TRANSITION:
		case STRAFING:
			loop(head, velocity, left_raycast.position, left_raycast.normal, right_raycast.position, right_raycast.normal, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
			left_foot = Transform(left_step.basis * foot_basis_offset, left_step.origin).interpolate_with(left_foot, walk_transition_progress / stepping_transition_duration);
			right_foot = Transform(right_step.basis * foot_basis_offset, right_step.origin).interpolate_with(right_foot, walk_transition_progress / stepping_transition_duration);
			break;
		case LAYING_TRANSITION:
			break;
		case LAYING:
			break;
		case OTHER_TRANSITION:
			break;
		case OTHER:
			break;
	}

	prevHead = head.origin;
}

bool RenIKPlacement::is_balanced(Vector3 center, Vector3 forward, Vector3 ground_normal, Vector3 left, Vector3 right) {
	Vector3 local_left = left - center;
	Vector3 local_right = right - center;

	Vector3 side_vector = forward.cross(ground_normal);

	Vector3 left_projection = local_left.project(side_vector);
	Vector3 right_projection = local_right.project(side_vector);

	return left_projection.dot(right_projection) < 0;//when these point in the same direction, then both the left and the right are on the same side of the center
}

void RenIKPlacement::foot_place(float delta, Transform head, PhysicsDirectSpaceState::RayResult left_raycast, PhysicsDirectSpaceState::RayResult right_raycast, PhysicsDirectSpaceState::RayResult laying_raycast) {
	// Basis uprightFootBasis(Vector3(-1, 0, 0), Vector3(0, -1, 0), Vector3(0, 0, 1));
	// const Spatial *newGroundLeftPointer = Object::cast_to<Spatial>(left_raycast.collider);
	// const Spatial *newGroundRightPointer = Object::cast_to<Spatial>(right_raycast.collider);
	// Transform centerGround = groundLeft.interpolate_with(groundRight, 0.5);
	// Transform headTransform = head_target_spatial->get_global_transform();
	// Vector3 newHead = centerGround.xform_inv(headTransform.origin);
	// Vector3 velocity = (newHead - prevHead) / delta;
	// float speed = velocity.length();
	// Vector3 leftOffset = standLeft.origin - left_raycast.position;
	// Vector3 rightOffset = standRight.origin - right_raycast.position;
	// Vector3 axis;
	// float leftRotation = 0;
	// float rightRotation = 0;
	// (standLeft.basis.inverse() * (head_target_spatial->get_global_transform().basis * uprightFootBasis)).get_rotation_axis_angle(axis, leftRotation);
	// (standRight.basis.inverse() * (head_target_spatial->get_global_transform().basis * uprightFootBasis)).get_rotation_axis_angle(axis, rightRotation);
	// //figure out if we're in the proper state
	// switch (walk_state) {
	// 	case FALLING:
	// 		if ((newGroundLeftPointer || newGroundRightPointer) && //if at least one foot is on the ground
	// 				!laying_raycast.collider //if we aren't too close to the ground
	// 		) {
	// 			walk_state = STANDING_TRANSITION;
	// 			lastLeft = placedLeft;
	// 			lastRight = placedRight;
	// 		}
	// 		break;
	// 	case STANDING_TRANSITION:
	// 		if (walk_transition_progress >= 1.0f) {
	// 			walk_state = STANDING;
	// 		} else {
	// 		}
	// 		break;
	// 	case STANDING:
	// 		//check balance & speed under threshold
	// 		if ((!newGroundLeftPointer && !newGroundRightPointer) || //if both feet are off the ground
	// 				laying_raycast.collider //if too close to the ground
	// 		) {
	// 			walk_state = FALLING; //start free falling
	// 		} else if (leftOffset.length_squared() > balance_threshold * left_leg_length || //if the left foot is too far away from where it should be
	// 					rightOffset.length_squared() > balance_threshold * right_leg_length || //if the right foot is too far away from where it should be
	// 					(leftOffset + rightOffset).length_squared() > balance_threshold * spine_length || //if we're off balance
	// 					leftRotation > rotation_threshold || //if the left foot is twisted
	// 					rightRotation > rotation_threshold || //if the right foot is twisted
	// 					newGroundLeftPointer != groundLeftPointer || newGroundRightPointer != newGroundRightPointer // if the ground object disappears or changes suddenly
	// 		) {
	// 			walk_state = STANDING_TRANSITION;
	// 			//check if the left foot was the one that caused the step
	// 			if (leftOffset.length_squared() > balance_threshold * left_leg_length ||
	// 					leftRotation > rotation_threshold ||
	// 					newGroundLeftPointer != groundLeftPointer) {
	// 				step_progress = 0;
	// 			} else {
	// 				step_progress = 0.5;
	// 			};
	// 			lastLeft = placedLeft;
	// 			lastRight = placedRight;
	// 		}
	// 		break;
	// 	case STEPPING_TRANSITION:
	// 		if (walk_transition_progress >= 1.0f) {
	// 			walk_state = STEPPING;
	// 		} else {
	// 		}
	// 		break;
	// 	case STEPPING:
	// 		//check speed above threshold
	// 		if ((!newGroundLeftPointer && !newGroundRightPointer) || //if both feet are off the ground
	// 				laying_raycast.collider //if too close to the ground
	// 		) {
	// 			walk_state = LAYING_TRANSITION;
	// 		} else if (speed < min_threshold * left_leg_length) { //moving too slow
	// 			walk_state = STANDING_TRANSITION;
	// 		}
	// 		break;
	// 	case LAYING_TRANSITION:
	// 		if (walk_transition_progress >= 1.0f) {
	// 			walk_state = LAYING;
	// 		} else {
	// 		}
	// 		break;
	// 	case LAYING:
	// 		break;
	// 	case OTHER_TRANSITION:
	// 		if (walk_transition_progress >= 1.0f) {
	// 			walk_state = OTHER;
	// 		} else {
	// 		}
	// 		break;
	// 	case OTHER:
	// 		break;
	// }
	// if (walk_state < 0) { //if we're in a transition
	// 	walk_transition_progress += delta;
	// } else {
	// 	walk_transition_progress = 0;
	// }
	// prevHead = newHead;
	// groundLeftPointer = newGroundLeftPointer;
	// groundRightPointer = newGroundRightPointer;

	// //for dangling
	// Vector3 hipDangle(0, spine_length * (dangle_height - 1), 0);
	// Vector3 legDangle;
	// if (laying_raycast.collider) {
	// 	Vector3 groundNormal = laying_raycast.normal;
	// 	legDangle = headTransform.basis.xform_inv(vector_rejection(headTransform.basis.xform(Vector3(0, -1, 0)), groundNormal).normalized()) * left_leg_length;
	// } else {
	// 	Vector3 groundNormal = newGroundLeftPointer || newGroundRightPointer ? left_raycast.normal.linear_interpolate(right_raycast.normal, 0.5).normalized() : Vector3(0, 1, 0);
	// 	if (headTransform.basis.xform(Vector3(0, 1, 0)).angle_to(groundNormal) > Math_PI / 3) { //if head is at an extreme angle, just have the feet follow the head
	// 		legDangle = Vector3(0, -left_leg_length, 0);
	// 	} else { //otherwise if the head is mostly vertical, try to point the feet at the ground
	// 		legDangle = headTransform.basis.xform_inv(groundNormal * left_leg_length * (dangle_height - 1));
	// 	}
	// }
	// Vector3 leftDangleVector = hipDangle + left_hip_offset + legDangle;
	// Vector3 rightDangleVector = hipDangle + right_hip_offset + legDangle;
	// Basis uprightRotatedFoot = headTransform.basis * uprightFootBasis;
	// Quat pointFeetToHead = align_vectors(hipDangle + legDangle, Vector3(0, -1, 0));
	// Transform dangleLeft = headTransform * Transform(uprightFootBasis * pointFeetToHead, leftDangleVector);
	// Transform dangleRight = headTransform * Transform(uprightFootBasis * pointFeetToHead, rightDangleVector);

	// switch (walk_state) {
	// 	case STANDING: //standing state
	// 	case STANDING_TRANSITION: //transitioning to standing state
	// 		if (groundLeftPointer) {
	// 			standLeft = groundLeft * groundedLeft;
	// 			placedLeft = walk_state == STANDING ? standLeft : placedLeft.interpolate_with(standLeft, dangle_stiffness);
	// 		} else {
	// 			placedLeft = placedLeft.interpolate_with(dangleLeft, dangle_stiffness);
	// 		}
	// 		if (groundRightPointer) {
	// 			standRight = groundRight * groundedRight;
	// 			placedRight = walk_state == STANDING ? standRight : placedRight.interpolate_with(standRight, dangle_stiffness);
	// 		} else {
	// 			placedRight = placedRight.interpolate_with(dangleRight, dangle_stiffness);
	// 		}
	// 		break;
	// 		// if (groundLeftPointer) {
	// 		// 	standLeft = leftRotation > rotation_threshold ? Transform(uprightFootBasis * pointFeetToHead, left_raycast.position) : groundLeft * groundedLeft;
	// 		// 	placedLeft = placedLeft.interpolate_with(standLeft, dangle_stiffness);
	// 		// } else {
	// 		// 	placedLeft = placedLeft.interpolate_with(dangleLeft, dangle_stiffness);
	// 		// }
	// 		// if (groundRightPointer) {
	// 		// 	standRight = rightRotation > rotation_threshold ? Transform(uprightFootBasis * pointFeetToHead, right_raycast.position) : groundRight * groundedRight;
	// 		// 	placedRight = placedRight.interpolate_with(standRight, dangle_stiffness);
	// 		// } else {
	// 		// 	placedRight = placedRight.interpolate_with(dangleRight, dangle_stiffness);
	// 		// }
	// 		// break;
	// 	case STEPPING: //stepping state
	// 	case STEPPING_TRANSITION:
	// 		if (groundLeftPointer || groundRightPointer) {
	// 			//if only one foot has a hold, pretend we're tightrope walking
	// 			groundLeftPointer = groundLeftPointer ? groundLeftPointer : groundRightPointer;
	// 			groundRightPointer = groundRightPointer ? groundRightPointer : groundLeftPointer;
	// 			Vector3 leftPos = groundLeftPointer ? left_raycast.position : right_raycast.position;
	// 			Vector3 leftNormal = groundLeftPointer ? left_raycast.normal.normalized() : right_raycast.normal.normalized();
	// 			Vector3 rightPos = groundRightPointer ? right_raycast.position : left_raycast.position;
	// 			Vector3 rightNormal = groundRightPointer ? right_raycast.normal.normalized() : left_raycast.normal.normalized();

	// 			Quat rotationLeft = leftRotation > rotation_threshold ? pointFeetToHead : align_vectors(leftNormal, uprightRotatedFoot.xform_inv(Vector3(0, -1, 0)));
	// 			Quat rotationRight = rightRotation > rotation_threshold ? pointFeetToHead : align_vectors(rightNormal, uprightRotatedFoot.xform_inv(Vector3(0, -1, 0)));

	// 			// step_progress = step_progress + speed * speed > 1.0 ? 0.5 : step_progress + speed * speed;
	// 			step_progress = fmod((step_progress + sqrtf(speed)), 1.0); //0 is left foot at apex, 0.5 is right foot at apex
	// 			float right_step_progress = fmod((step_progress + 0.5), 1.0);
	// 			float totalContact = (contact_length / 2) * MIN(0, 1 - contact_scale * sqrt(speed / max_threshold));
	// 			float bottomSegmentSize = 0.5 - totalContact / 2; //size of followthru and buildup. starts at 0 for slow speeds
	// 			float topSegmentSize = 0.25; //size of the top halves of the loop
	// 			//
	// 			//ideal is where we want the foot to land. At slow speeds its a little in front of where we're headed. At high speeds it's directly below the center of gravity.
	// 			Transform idealLeft(uprightRotatedFoot * rotationLeft, leftPos + (vector_rejection(velocity, leftNormal).normalized() * totalContact));
	// 			Transform idealRight(uprightRotatedFoot * rotationLeft, rightPos + (vector_rejection(velocity, rightNormal).normalized() * totalContact));
	// 			placedLeft.set_basis(placedLeft.basis.slerp(idealLeft.basis, dangle_stiffness)); //always lags a little
	// 			placedRight.set_basis(placedRight.basis.slerp(idealLeft.basis, dangle_stiffness)); //always lags a little
	// 			Vector3 saddleLeft = (leftPos - headTransform.origin) * (1 - step_height - height_scale * speed * speed);
	// 			Vector3 saddleRight = (rightPos - headTransform.origin) * (1 - step_height - height_scale * speed * speed);
	// 			float followScale = loop_scale * speed * loop_ratio;
	// 			float buildScale = loop_scale * speed * (1 - loop_ratio);
	// 			Vector3 leftFollow = leftNormal * followthru_angle - velocity.normalized();
	// 			Vector3 rightFollow = rightNormal * followthru_angle - velocity.normalized();
	// 			Vector3 leftBuild = leftNormal * buildup_angle + velocity.normalized();
	// 			Vector3 rightBuild = rightNormal * buildup_angle + velocity.normalized();
	// 			//left foot
	// 			if (step_progress < topSegmentSize) { //starts at end of the followthru, ends at saddle
	// 				Vector3 startPos = headTransform.origin + lastLeft.origin + leftFollow * followScale;
	// 				Vector3 endPos = saddleLeft;
	// 				Quat startRot = lastLeft.basis.get_rotation_quat() * Quat(Vector3(1, 0, 0), followScale * Math_PI / 2); //rotated down 90 degrees
	// 				Quat endRot = idealLeft.basis.get_rotation_quat() * Quat(Vector3(1, 0, 0), Math_PI / 4); //rotated down 45 degrees
	// 				float segmentProgress = step_progress / topSegmentSize;
	// 				Vector3 posDiff = endPos - startPos;
	// 				Vector3 startCurve = startPos + leftNormal * (posDiff.length() / 4);
	// 				Vector3 endCurve = endPos - vector_rejection(posDiff, leftNormal).normalized() * (posDiff.length() / 4);
	// 				placedLeft.set_basis(startRot.slerp(endRot, segmentProgress));
	// 				placedLeft.set_origin(startPos.cubic_interpolate(endPos, startCurve, endCurve, segmentProgress));
	// 			} else if (step_progress < 2 * topSegmentSize) { //starts at saddle, ends at apex
	// 				Vector3 startPos = saddleLeft;
	// 				Vector3 endPos = headTransform.origin + idealLeft.origin + leftBuild * buildScale;
	// 				Quat startRot = idealLeft.basis.get_rotation_quat() * Quat(Vector3(1, 0, 0), Math_PI / 4);
	// 				Quat endRot = idealLeft.basis.get_rotation_quat() * Quat(Vector3(-1, 0, 0), buildScale * Math_PI / 4); //rotated up 45 degrees
	// 				float segmentProgress = (step_progress - topSegmentSize) / topSegmentSize;
	// 				Vector3 posDiff = endPos - startPos;
	// 				Vector3 startCurve = startPos + vector_rejection(posDiff, leftNormal).normalized() * (posDiff.length() / 4);
	// 				Vector3 endCurve = endPos - leftNormal * (posDiff.length() / 4);
	// 				placedLeft.set_basis(startRot.slerp(endRot, segmentProgress));
	// 				placedLeft.set_origin(startPos.cubic_interpolate(endPos, startCurve, endCurve, segmentProgress));
	// 			} else if (step_progress < 2 * topSegmentSize + bottomSegmentSize) { //starts at apex, goes to landing point
	// 				Vector3 startPos = headTransform.origin + idealLeft.origin + leftBuild * buildScale;
	// 				Vector3 endPos = idealLeft.origin;
	// 				Quat startRot = idealLeft.basis.get_rotation_quat() * Quat(Vector3(-1, 0, 0), buildScale * Math_PI / 4);
	// 				Quat endRot = idealLeft.basis;
	// 				float segmentProgress = (step_progress - topSegmentSize) / topSegmentSize;
	// 				Vector3 posDiff = endPos - startPos;
	// 				Vector3 startCurve = startPos - leftNormal * (posDiff.length() / 4);
	// 				Vector3 endCurve = endPos;
	// 				placedLeft.set_basis(startRot.slerp(endRot, segmentProgress));
	// 				placedLeft.set_origin(startPos.cubic_interpolate(endPos, startCurve, endCurve, segmentProgress));
	// 				//needed for the next step
	// 				groundLeft = groundLeftPointer->get_global_transform();
	// 				groundedLeft = groundLeft.affine_inverse() * idealLeft;
	// 			} else if (step_progress < 2 * topSegmentSize + bottomSegmentSize + totalContact) { //lands and stays in contact with the ground
	// 				groundLeft = groundLeftPointer->get_global_transform();
	// 				placedLeft = groundLeft * groundedLeft;
	// 			} else { //lifts back up during the follow thru
	// 				float segmentProgress = (step_progress - 2 * topSegmentSize - bottomSegmentSize - totalContact) / bottomSegmentSize;
	// 			}
	// 		}
	// 		break;
	// 	case FALLING: //jumping / falling
	// 		placedLeft = placedLeft.interpolate_with(dangleLeft, dangle_stiffness);
	// 		placedRight = placedRight.interpolate_with(dangleRight, dangle_stiffness);
	// 		break;
	// 	case LAYING:
	// 	case LAYING_TRANSITION:
	// 		placedLeft = placedLeft.interpolate_with(dangleLeft, dangle_stiffness);
	// 		placedRight = placedRight.interpolate_with(dangleRight, dangle_stiffness);
	// 		break;
	// }
}

void RenIKPlacement::set_falling(bool falling) {
	fall_override = falling;
}

void RenIKPlacement::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
}

uint32_t RenIKPlacement::get_collision_mask() const {
	return collision_mask;
}

void RenIKPlacement::set_collision_mask_bit(int p_bit, bool p_value) {
	uint32_t mask = get_collision_mask();
	if (p_value)
		mask |= 1 << p_bit;
	else
		mask &= ~(1 << p_bit);
	set_collision_mask(mask);
}

bool RenIKPlacement::get_collision_mask_bit(int p_bit) const {
	return get_collision_mask() & (1 << p_bit);
}

void RenIKPlacement::set_collide_with_areas(bool p_clip) {

	collide_with_areas = p_clip;
}

bool RenIKPlacement::is_collide_with_areas_enabled() const {

	return collide_with_areas;
}

void RenIKPlacement::set_collide_with_bodies(bool p_clip) {

	collide_with_bodies = p_clip;
}

bool RenIKPlacement::is_collide_with_bodies_enabled() const {

	return collide_with_bodies;
}

#endif // _3D_DISABLED