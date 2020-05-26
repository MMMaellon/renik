#include "renik_placement.h"

#ifndef _3D_DISABLED
void RenIKPlacement::hip_place(float delta, Transform head, Transform left_foot, Transform right_foot, float twist) {

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
	Vector3 leftStop = head.origin + Vector3(0, (-spine_length - left_leg_length) * (1 + raycast_allowance) + left_hip_offset[1], 0) + head.basis.xform(left_hip_offset);
	Vector3 rightStop = head.origin + Vector3(0, (-spine_length - right_leg_length) * (1 + raycast_allowance) + right_hip_offset[1], 0) + head.basis.xform(right_hip_offset);

	bool left_collided = dss->intersect_ray(leftStart, leftStop, left_raycast, Set<RID>(), collision_mask, collide_with_bodies, collide_with_areas);
	bool right_collided = dss->intersect_ray(rightStart, rightStop, right_raycast, Set<RID>(), collision_mask, collide_with_bodies, collide_with_areas);
	bool laying_down = dss->intersect_ray(head.origin, head.origin - Vector3(0, spine_length, 0), laying_raycast, Set<RID>(), collision_mask, collide_with_bodies, collide_with_areas);
	if (!left_collided) {
		left_raycast.collider = nullptr;
	}
	if (!right_collided) {
		right_raycast.collider = nullptr;
	}
	if (!laying_down) {
		laying_raycast.collider = nullptr;
	}
	foot_place(delta, head, left_raycast, right_raycast, laying_raycast);
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