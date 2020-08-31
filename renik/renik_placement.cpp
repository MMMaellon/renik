#include "renik_placement.h"
#include "renik_helper.h"

#ifndef _3D_DISABLED

void RenIKPlacement::save_previous_transforms() {
	prev_hip = target_hip;
	prev_left_foot = target_left_foot;
	prev_right_foot = target_right_foot;
}

void RenIKPlacement::interpolate_transforms(float fraction, bool update_hips, bool update_feet) {
	if (update_hips) {
		interpolated_hip = prev_hip.interpolate_with(target_hip, fraction);
	}
	if (update_feet) {
		interpolated_left_foot = prev_left_foot.interpolate_with(target_left_foot, fraction);
		interpolated_right_foot = prev_right_foot.interpolate_with(target_right_foot, fraction);
	}
}

void RenIKPlacement::hip_place(float delta, Transform head, Transform left_foot, Transform right_foot, float twist, bool instant) {
	Vector3 left_middle = (left_foot.translated(Vector3(0, 0, left_foot_length / 2))).origin;
	Vector3 right_middle = (right_foot.translated(Vector3(0, 0, right_foot_length / 2))).origin;
	float left_distance = left_middle.distance_squared_to(head.origin);
	float right_distance = right_middle.distance_squared_to(head.origin);
	Vector3 foot_median = left_middle.linear_interpolate(right_middle, 0.5);
	Vector3 foot = left_distance > right_distance ? left_middle : right_middle;
	Vector3 foot_direction = (foot - head.origin).project(foot_median - head.origin);

	target_hip.basis = RenIKHelper::align_vectors(Vector3(0, -1, 0), foot_direction);
	Vector3 head_forward = head.basis.inverse()[2];
	Vector3 feet_forward = (left_foot.interpolate_with(right_foot, 0.5)).basis[2];
	Vector3 hip_forward = feet_forward.linear_interpolate(head_forward, 0.5);

	Vector3 hip_y = -foot_direction.normalized();
	Vector3 hip_z = RenIKHelper::vector_rejection(hip_forward.normalized(), hip_y).normalized();
	Vector3 hip_x = hip_y.cross(hip_z).normalized();
	target_hip.basis = Basis(hip_x, hip_y, hip_z).inverse().orthonormalized();

	float crouch_distance = head.origin.distance_to(foot) * crouch_ratio;
	float extra_hip_distance = hip_offset.length() - crouch_distance;
	Vector3 follow_hip_direction = target_hip.basis.xform_inv(head.basis.xform(hip_offset));

	Vector3 effective_hip_direction = hip_offset.linear_interpolate(follow_hip_direction, hip_follow_head_influence).normalized();

	target_hip.origin = head.origin;
	target_hip.translate(crouch_distance * effective_hip_direction.normalized());
	if (extra_hip_distance > 0) {
		target_hip.translate(Vector3(0, 0, -extra_hip_distance * (1 / hunch_ratio)));
	}

	if (instant) {
		prev_hip = target_hip;
	}
}

/*foot_place requires raycasting unless a raycast result is provided.
	Raycasting needs to happen inside of a physics update
*/
void RenIKPlacement::foot_place(float delta, Transform head, Ref<World> w3d, bool instant) {
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
	foot_place(delta, head, left_raycast, right_raycast, laying_raycast, instant);
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
		Vector3 foot_diff = target_left_foot.origin - target_right_foot.origin;
		float dot = foot_diff.dot(velocity);
		if (dot == 0) {
			float left_dist = target_left_foot.origin.distance_squared_to(left_ground);
			float right_dist = target_right_foot.origin.distance_squared_to(right_ground);
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

int RenIKPlacement::get_loop_state(float loop_state_scaling, float loop_progress, float &loop_state_progress, Gait gait) {
	int state = -1;
	float ground_time = gait.ground_time - gait.ground_time * loop_state_scaling;
	float lift_time = gait.lift_time_base + gait.lift_time_scalar * loop_state_scaling;
	float apex_in_time = gait.apex_in_time_base + gait.apex_in_time_scalar * loop_state_scaling;
	float apex_out_time = gait.apex_out_time_base + gait.apex_out_time_scalar * loop_state_scaling;
	float drop_time = gait.drop_time_base + gait.drop_time_scalar * loop_state_scaling;
	float total_time = ground_time + lift_time + apex_in_time + apex_out_time + drop_time + ground_time;

	float progress_time = loop_progress * total_time;

	if (progress_time < ground_time) {
		state = LOOP_GROUND_IN;
		loop_state_progress = (progress_time) / ground_time;
	} else if (progress_time < ground_time + lift_time) {
		state = LOOP_LIFT;
		loop_state_progress = (progress_time - ground_time) / lift_time;
	} else if (progress_time < ground_time + lift_time + apex_in_time) {
		state = LOOP_APEX_IN;
		loop_state_progress = (progress_time - ground_time - lift_time) / apex_in_time;
	} else if (progress_time < ground_time + lift_time + apex_in_time + apex_out_time) {
		state = LOOP_APEX_OUT;
		loop_state_progress = (progress_time - ground_time - lift_time - apex_in_time) / apex_out_time;
	} else if (progress_time < ground_time + lift_time + apex_in_time + apex_out_time + drop_time) {
		state = LOOP_DROP;
		loop_state_progress = (progress_time - ground_time - lift_time - apex_in_time - apex_out_time) / drop_time;
	} else {
		state = LOOP_GROUND_OUT;
		loop_state_progress = (progress_time - ground_time - lift_time - apex_in_time - apex_out_time - drop_time) / ground_time;
	}

	return state;
}

void RenIKPlacement::loop_foot(Transform &step, Transform &stand, Transform &stand_local, Spatial *ground, Spatial **prev_ground, int &loop_state, Vector3 &grounded_stop, Transform head, float leg_length, float foot_length, Vector3 velocity, float loop_scaling, float step_progress, Vector3 ground_pos, Vector3 ground_normal, Gait gait) {
	Quat upright_foot = RenIKHelper::align_vectors(Vector3(0,1,0), head.basis.xform_inv(ground_normal));
	bool twisted = false;
	if(ground_normal.dot(head.basis[1]) < cos(rotation_threshold) && ground_normal.dot(Vector3(0,1,0)) < cos(rotation_threshold)){
		upright_foot = Quat();
		twisted = true;
	}
	Vector3 ground_velocity = RenIKHelper::vector_rejection(velocity, ground_normal);
	if(ground_velocity.length() > max_threshold * step_pace){
		ground_velocity = ground_velocity.normalized() * max_threshold * step_pace;
	}
	float loop_state_progress = 0;
	loop_state = get_loop_state(loop_scaling, step_progress, loop_state_progress, gait);
	float head_distance = head.origin.distance_to(ground_pos);
	float ease_scaling = loop_scaling * loop_scaling * loop_scaling * loop_scaling;//ease the growth a little
	float vertical_scaling = head_distance * ease_scaling;
	float horizontal_scaling = leg_length * ease_scaling;
	Transform grounded_foot = Transform(head.basis * upright_foot, ground_pos);
	Transform lifted_foot = Transform(grounded_foot.basis.rotated_local(Vector3(1, 0, 0), ease_scaling * gait.lift_angle), ground_pos + ground_normal * vertical_scaling * gait.lift_vertical_scalar + ground_normal * head_distance * gait.lift_vertical - ground_velocity.normalized() * horizontal_scaling * gait.lift_horizontal_scalar);
	Transform apex_foot = Transform(grounded_foot.basis.rotated_local(Vector3(1, 0, 0), ease_scaling * gait.apex_angle), ground_pos + ground_normal * vertical_scaling * gait.apex_vertical_scalar+ ground_normal * head_distance * gait.apex_vertical);
	Transform drop_foot = Transform(grounded_foot.basis.rotated_local(Vector3(1, 0, 0), ease_scaling * gait.drop_angle), ground_pos + ground_normal * vertical_scaling * gait.drop_vertical_scalar + ground_normal * head_distance * gait.drop_vertical_scalar + ground_velocity.normalized() * horizontal_scaling * gait.drop_horizontal_scalar);

	switch (loop_state) {
		case LOOP_GROUND_IN:
		case LOOP_GROUND_OUT:{
			//stick to where it landed
			if (ground != nullptr && ground == *prev_ground) {
				stand_foot(grounded_foot, stand, stand_local, ground);
			} else if(ground != nullptr){
				*prev_ground = ground;
				stand = grounded_foot;
				Transform ground_global = ground->get_global_transform();
				ground_global.basis.orthonormalize();
				stand_local = ground_global.affine_inverse() * stand;
			} else {
				stand = grounded_foot;
			}
			step = stand;

			float step_distance = step.origin.distance_to(ground_pos) / leg_length;
			Transform lean_offset;
			float tip_toe_angle = step_distance * gait.tip_toe_distance_scalar + horizontal_scaling * gait.tip_toe_speed_scalar;
			tip_toe_angle = tip_toe_angle > gait.tip_toe_angle_max ? gait.tip_toe_angle_max : tip_toe_angle;
			lean_offset.origin = Vector3(0, foot_length * sin(tip_toe_angle), 0);
			lean_offset.rotate_basis(Vector3(1, 0, 0), tip_toe_angle);
			step *= lean_offset;

			grounded_stop = step.origin;

			break;
		}
		case LOOP_LIFT:{
			float step_distance = step.origin.distance_to(ground_pos) / leg_length;
			Transform lean_offset;
			float tip_toe_angle = step_distance * gait.tip_toe_distance_scalar + horizontal_scaling * gait.tip_toe_speed_scalar;
			tip_toe_angle = tip_toe_angle > gait.tip_toe_angle_max ? gait.tip_toe_angle_max : tip_toe_angle;

			step.basis = grounded_foot.basis.rotated_local(Vector3(1,0,0), tip_toe_angle).slerp(lifted_foot.basis, loop_state_progress);
			step.origin = grounded_stop.cubic_interpolate(lifted_foot.origin, grounded_stop - ground_velocity * horizontal_scaling, lifted_foot.origin + ground_normal * vertical_scaling, loop_state_progress);
			break;
		}
		case LOOP_APEX_IN:
			step.basis = lifted_foot.basis.slerp(apex_foot.basis, loop_state_progress);
			step.origin = lifted_foot.origin.cubic_interpolate(apex_foot.origin, lifted_foot.origin - ground_normal * vertical_scaling, apex_foot.origin + ground_velocity * leg_length, loop_state_progress);
			break;
		case LOOP_APEX_OUT:
			step.basis = apex_foot.basis.slerp(drop_foot.basis, loop_state_progress);
			step.origin = apex_foot.origin.cubic_interpolate(drop_foot.origin, apex_foot.origin - ground_velocity * horizontal_scaling, drop_foot.origin - ground_normal * vertical_scaling, loop_state_progress);
			break;
		case LOOP_DROP:
			step.basis = drop_foot.basis.slerp(grounded_foot.basis, loop_state_progress);
			step.origin = drop_foot.origin.cubic_interpolate(grounded_foot.origin, drop_foot.origin + ground_normal * vertical_scaling, grounded_foot.origin - ground_velocity * horizontal_scaling, loop_state_progress);
			break;
	}

	if (loop_state != LOOP_GROUND_IN && loop_state != LOOP_GROUND_OUT) {
		//update standing positions to ensure a smooth transition to standing
		stand.origin = ground_pos;
		stand.basis = grounded_foot.basis;
		if (ground != nullptr) {
			Transform ground_global = ground->get_global_transform();
			ground_global.basis.orthonormalize();
			stand_local = ground_global.affine_inverse() * stand;
		}
		if(walk_state != LOOP_LIFT){
			grounded_stop = step.origin;
		} else {
			float contact_easing = gait.contact_point_ease + gait.contact_point_ease_scalar * loop_scaling;
			contact_easing = contact_easing > 1 ? 1 : contact_easing;
			grounded_stop = grounded_stop.linear_interpolate(ground_pos, contact_easing);
		}
	}
}

void RenIKPlacement::loop(Transform head, Vector3 velocity, Vector3 left_ground_pos, Vector3 left_normal, Vector3 right_ground_pos, Vector3 right_normal, bool left_grounded, bool right_grounded, Gait gait) {
	float stride_speed = step_pace * velocity.length() / ((left_leg_length + right_leg_length) / 2);
	stride_speed = log( 1 + stride_speed);
	stride_speed = stride_speed > max_threshold ? max_threshold : stride_speed;
	stride_speed = stride_speed < min_threshold ? min_threshold : stride_speed;
	float new_loop_scaling = max_threshold > min_threshold ? (stride_speed - min_threshold) / (max_threshold - min_threshold) : 0;
	loop_scaling = (loop_scaling * gait.scaling_ease + new_loop_scaling * (1 - gait.scaling_ease));
	step_progress = Math::fmod((step_progress + stride_speed * (gait.speed_scalar_min * (1 - loop_scaling) + gait.speed_scalar_max * loop_scaling)), 1.0f);

	if (left_grounded) {
		loop_foot(left_step, left_stand, left_stand_local, left_ground, &prev_left_ground, left_loop_state, left_grounded_stop, head, left_leg_length, left_foot_length, velocity, loop_scaling, step_progress, left_ground_pos, left_normal, gait);
	} else {
		Transform left_dangle = dangle_foot(head, (spine_length + left_leg_length) * dangle_ratio, left_leg_length, left_hip_offset);
		left_step.basis = left_step.basis.slerp(left_dangle.basis, 1.0f - (1.0f / dangle_stiffness));
		left_step.origin = RenIKHelper::log_clamp(left_step.origin, left_dangle.origin, 1.0 / dangle_stiffness);
	}

	if (right_grounded) {
		loop_foot(right_step, right_stand, right_stand_local, right_ground, &prev_right_ground, right_loop_state, right_grounded_stop, head, right_leg_length, right_foot_length, velocity, loop_scaling, Math::fmod((step_progress + 0.5f), 1.0f), right_ground_pos, right_normal, gait);
	} else {
		Transform right_dangle = dangle_foot(head, (spine_length + right_leg_length) * dangle_ratio, right_leg_length, right_hip_offset);
		target_right_foot.basis = target_right_foot.basis.slerp(right_dangle.basis, 1.0f - (1.0f / dangle_stiffness));
		target_right_foot.origin = RenIKHelper::log_clamp(target_right_foot.origin, right_dangle.origin, 1.0 / dangle_stiffness);
	}
}

void RenIKPlacement::step_direction(Vector3 forward, Vector3 side, Vector3 velocity, Vector3 left_ground, Vector3 right_ground, bool left_grounded, bool right_grounded) {
	if (Math::abs(velocity.dot(side)) > strafe_angle_limit) {
		if (walk_state != STRAFING && walk_state != STRAFING_TRANSITION) {
			walk_state = STRAFING_TRANSITION;
			walk_transition_progress = stepping_transition_duration; //In units of loop progression
			initialize_loop(velocity, left_ground, right_ground, left_grounded, right_grounded);
		}
	} else if (velocity.dot(forward) < 0) {
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

void RenIKPlacement::stand_foot(Transform foot, Transform &stand, Transform &stand_local, Spatial *ground) {
	Transform ground_global = ground->get_global_transform();
	ground_global.basis.orthonormalize();
	stand = ground_global * stand_local;
	stand.basis.orthonormalize();
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
void RenIKPlacement::foot_place(float delta, Transform head, PhysicsDirectSpaceState::RayResult left_raycast, PhysicsDirectSpaceState::RayResult right_raycast, PhysicsDirectSpaceState::RayResult laying_raycast, bool instant) {
	//Step 1: Find the proper state
	//Note we always enter transition states when possible

	left_ground = (Spatial *)left_raycast.collider;
	right_ground = (Spatial *)right_raycast.collider;
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
		Vector3 ground_normal = left_raycast.normal.linear_interpolate(right_raycast.normal, 0.5).normalized();
		switch(walk_state){
			case STANDING:{//brackets to stop declarations from breaking case labels
				//test that the feet aren't twisted in weird ways
				Vector3 left_head_forward = (head.basis * RenIKHelper::align_vectors(Vector3(0, 1, 0), head.basis.xform_inv(left_raycast.normal)))[2];
				Vector3 right_head_forward = (head.basis * RenIKHelper::align_vectors(Vector3(0, 1, 0), head.basis.xform_inv(right_raycast.normal)))[2];
				// left_head_forward = RenIKHelper::vector_rejection(left_head_forward, ground_normal).normalized();
				Vector3 left_forward = RenIKHelper::vector_rejection(left_stand.basis[2], left_raycast.normal).normalized();
				Vector3 right_forward = RenIKHelper::vector_rejection(right_stand.basis[2], right_raycast.normal).normalized();
				Vector3 forward = left_forward.linear_interpolate(right_forward, 0.5).normalized();
				Vector3 upward = head.basis[1];
				Vector3 left_upward = left_stand.basis[1];
				Vector3 right_upward = right_stand.basis[1];
				
				if (left_velocity.length() > effective_min_threshold
				|| right_velocity.length() > effective_min_threshold
				|| (left_raycast.collider != nullptr && right_raycast.collider != nullptr && !is_balanced(target_left_foot, target_right_foot))
				|| (left_raycast.collider != nullptr && left_stand.origin.distance_squared_to(left_raycast.position) > balance_threshold * (left_leg_length + right_leg_length) / 2)
				|| (right_raycast.collider != nullptr && right_stand.origin.distance_squared_to(right_raycast.position) > balance_threshold * (left_leg_length + right_leg_length) / 2)
				|| (left_raycast.collider != nullptr && (Spatial *)left_raycast.collider != left_ground)
				|| (right_raycast.collider != nullptr && (Spatial *)right_raycast.collider != right_ground)
				|| left_head_forward.dot(left_forward) < cos(rotation_threshold)
				|| right_head_forward.dot(right_forward) < cos(rotation_threshold)
				|| (left_raycast.collider != nullptr && left_raycast.normal.dot(left_upward) < cos(rotation_threshold) && upward.dot(left_upward) < cos(rotation_threshold))
				|| (right_raycast.collider != nullptr && right_raycast.normal.dot(right_upward) < cos(rotation_threshold) && upward.dot(right_upward) < cos(rotation_threshold))
				) {
					step_direction(head.basis[2], head.basis[0], velocity, left_raycast.position, right_raycast.position, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
				}
				break;
			}
			case STANDING_TRANSITION:
				if (left_velocity.length() > effective_min_threshold
				|| right_velocity.length() > effective_min_threshold
				|| (left_raycast.collider != nullptr && (Spatial *)left_raycast.collider != left_ground)
				|| (right_raycast.collider != nullptr && (Spatial *)right_raycast.collider != right_ground)
				) {
					step_direction(head.basis[2], head.basis[0], velocity, left_raycast.position, right_raycast.position, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
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
				&& (left_raycast.collider == nullptr || left_stand.origin.distance_squared_to(left_raycast.position) < balance_threshold * (left_leg_length + right_leg_length) / 2)
				&& (right_raycast.collider == nullptr || right_stand.origin.distance_squared_to(right_raycast.position) < balance_threshold * (left_leg_length + right_leg_length) / 2)
				) {
					walk_state = STANDING_TRANSITION;
					walk_transition_progress = standing_transition_duration; //In units of loop progression
				} else {
					step_direction(head.basis[2], head.basis[0], velocity, left_raycast.position, right_raycast.position, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
				}
				break;
			default:
				step_direction(head.basis[2], head.basis[0], velocity, left_raycast.position, right_raycast.position, left_raycast.collider != nullptr, right_raycast.collider != nullptr);
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

			target_left_foot.basis = target_left_foot.basis.slerp(left_dangle.basis * foot_basis_offset, 1.0f - (1.0f / dangle_stiffness));
			target_left_foot.origin = RenIKHelper::log_clamp(target_left_foot.origin, left_dangle.origin, 1.0 / dangle_stiffness);

			target_right_foot.basis = target_right_foot.basis.slerp(right_dangle.basis * foot_basis_offset, 1.0f - (1.0f / dangle_stiffness));
			target_right_foot.origin = RenIKHelper::log_clamp(target_right_foot.origin, right_dangle.origin, 1.0 / dangle_stiffness);

			//for easy transitions
			left_stand = target_left_foot;
			right_stand = target_right_foot;
			left_step = target_left_foot;
			right_step = target_right_foot;
			left_grounded_stop = target_left_foot.origin;
			right_grounded_stop = target_right_foot.origin;
			left_ground = nullptr;
			right_ground = nullptr;
			prev_left_ground = nullptr;
			prev_right_ground = nullptr;
			break;
		}
		case STANDING_TRANSITION:
		case STANDING :{
			float left_distance = head.origin.distance_to(right_raycast.position);//dangle foot according to where the other foot is
			float right_distance = head.origin.distance_to(left_raycast.position);
			float effective_transition_progress = walk_transition_progress / standing_transition_duration;
			effective_transition_progress = effective_transition_progress <= 1 ? effective_transition_progress : 1.0;
			if (left_ground != nullptr) {
				stand_foot(target_left_foot, left_stand, left_stand_local, left_ground);
				target_left_foot = Transform(left_stand.basis * foot_basis_offset, left_stand.origin).interpolate_with(target_left_foot, effective_transition_progress);
				left_grounded_stop = left_stand.origin;
			} else {
				Transform left_dangle = dangle_foot(head, (spine_length + left_leg_length) * dangle_ratio, left_leg_length, left_hip_offset);
				target_left_foot.basis = target_left_foot.basis.slerp(left_dangle.basis * foot_basis_offset, 1.0f - (1.0f / dangle_stiffness));
				target_left_foot.origin = RenIKHelper::log_clamp(target_left_foot.origin, left_dangle.origin, 1.0 / dangle_stiffness);
			}

			if (right_ground != nullptr) {
				stand_foot(target_right_foot, right_stand, right_stand_local, right_ground);
				target_right_foot = Transform(right_stand.basis * foot_basis_offset, right_stand.origin).interpolate_with(target_right_foot, effective_transition_progress);
				right_grounded_stop = right_stand.origin;
			} else {
				Transform right_dangle = dangle_foot(head, (spine_length + right_leg_length) * dangle_ratio, right_leg_length, right_hip_offset);
				target_right_foot.basis = target_right_foot.basis.slerp(right_dangle.basis * foot_basis_offset, 1.0f - (1.0f / dangle_stiffness));
				target_right_foot.origin = RenIKHelper::log_clamp(target_right_foot.origin, right_dangle.origin, 1.0 / dangle_stiffness);
			}

			break;
		}
		case STEPPING_TRANSITION:
		case STEPPING:{
			float effective_transition_progress = walk_transition_progress / stepping_transition_duration;
			effective_transition_progress = effective_transition_progress <= 1 ? effective_transition_progress : 1.0;
			loop(head, velocity, left_raycast.position, left_raycast.normal, right_raycast.position, right_raycast.normal, left_raycast.collider != nullptr, right_raycast.collider != nullptr, forward_gait);
			target_left_foot = Transform(left_step.basis * foot_basis_offset, left_step.origin).interpolate_with(target_left_foot, effective_transition_progress);
			target_right_foot = Transform(right_step.basis * foot_basis_offset, right_step.origin).interpolate_with(target_right_foot, effective_transition_progress);
			break;
		}
		case BACKSTEPPING_TRANSITION:
		case BACKSTEPPING:{
			float effective_transition_progress = walk_transition_progress / stepping_transition_duration;
			effective_transition_progress = effective_transition_progress <= 1 ? effective_transition_progress : 1.0;
			loop(head, velocity, left_raycast.position, left_raycast.normal, right_raycast.position, right_raycast.normal, left_raycast.collider != nullptr, right_raycast.collider != nullptr, backward_gait);
			target_left_foot = Transform(left_step.basis * foot_basis_offset, left_step.origin).interpolate_with(target_left_foot, effective_transition_progress);
			target_right_foot = Transform(right_step.basis * foot_basis_offset, right_step.origin).interpolate_with(target_right_foot, effective_transition_progress);
			break;
		}
		case STRAFING_TRANSITION:
		case STRAFING:{
			float effective_transition_progress = walk_transition_progress / stepping_transition_duration;
			effective_transition_progress = effective_transition_progress <= 1 ? effective_transition_progress : 1.0;
			loop(head, velocity, left_raycast.position, left_raycast.normal, right_raycast.position, right_raycast.normal, left_raycast.collider != nullptr, right_raycast.collider != nullptr, sideways_gait);
			target_left_foot = Transform(left_step.basis * foot_basis_offset, left_step.origin).interpolate_with(target_left_foot, effective_transition_progress);
			target_right_foot = Transform(right_step.basis * foot_basis_offset, right_step.origin).interpolate_with(target_right_foot, effective_transition_progress);
			break;
		}
		case LAYING_TRANSITION:
			break;
		case LAYING:
			break;
		case OTHER_TRANSITION:
			break;
		case OTHER:
			break;
	}

	if (instant) {
		prev_left_foot = target_left_foot;
		prev_right_foot = target_right_foot;
	}

	prevHead = head.origin;
}

bool RenIKPlacement::is_balanced(Transform left, Transform right) {

	Vector3 relative_right = left.xform_inv(right.origin);
	Vector3 relative_left = right.xform_inv(left.origin);

	return relative_right.dot(Vector3(1, 0, 0)) > 0 || relative_left.dot(Vector3(1, 0, 0)) < 0; //when these point in the same direction, then both the left and the right are on the same side of the center
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
