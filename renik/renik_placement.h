#ifndef RENIK_PLACEMENT_H
#define RENIK_PLACEMENT_H

#include "servers/physics_server_3d.h"
#include <core/config/engine.h>
#include <scene/3d/node_3d.h>
#include <scene/main/node.h>

struct Gait {
  float speed_scalar_min = 1;
  float speed_scalar_max = 1;
  // timing
  float ground_time = 20;
  float lift_time_base = 10;
  float lift_time_scalar = 5;
  float apex_in_time_base = 10;
  float apex_in_time_scalar = 5;
  float apex_out_time_base = 10;
  float apex_out_time_scalar = 5;
  float drop_time_base = 10;
  float drop_time_scalar = 5;
  // ground
  float tip_toe_distance_scalar = Math_PI / 8;
  float tip_toe_speed_scalar = Math_PI / 4;
  float tip_toe_angle_max = Math_PI / 3;
  // lift
  float lift_vertical = 0.025;
  float lift_vertical_scalar = 0.25;
  float lift_horizontal_scalar = 0.5;
  float lift_angle = 0;
  // apex
  float apex_vertical = 0.01;
  float apex_vertical_scalar = 0.1;
  float apex_angle = 0;
  // drop
  float drop_vertical = 0.0;
  float drop_vertical_scalar = 0.15;
  float drop_horizontal_scalar = 0.25;
  float drop_angle = 0;
  // contact
  float contact_point_ease = 0.1;
  float contact_point_ease_scalar = 0.4;
  float scaling_ease = 0.9;

  Gait(float p_speed_min_scalar, float p_speed_max_scalar,
       float p_ground_time_min, float p_lift_time_base,
       float p_lift_time_scalar, float p_apex_in_time_base,
       float p_apex_in_time_scalar, float p_apex_out_time_base,
       float p_apex_out_time_scalar, float p_drop_time_base,
       float p_drop_time_scalar, float p_tip_toe_distance_scalar,
       float p_tip_toe_speed_scalar, float p_tip_toe_angle_max,
       float p_lift_vertical, float p_lift_vertical_scalar,
       float p_lift_horizontal_scalar, float p_lift_angle,
       float p_apex_vertical, float p_apex_vertical_scalar, float p_apex_angle,
       float p_drop_vertical, float p_drop_vertical_scalar,
       float p_drop_horizontal_scalar, float p_drop_angle,
       float p_contact_point_ease, float p_contact_point_ease_scalar,
       float p_scaling_ease) {
    speed_scalar_min = p_speed_min_scalar;
    speed_scalar_max = p_speed_max_scalar;
    ground_time = p_ground_time_min;
    lift_time_base = p_lift_time_base;
    lift_time_scalar = p_lift_time_scalar;
    apex_in_time_base = p_apex_in_time_base;
    apex_in_time_scalar = p_apex_in_time_scalar;
    apex_out_time_base = p_apex_out_time_base;
    apex_out_time_scalar = p_apex_out_time_scalar;
    drop_time_base = p_drop_time_base;
    drop_time_scalar = p_drop_time_scalar;
    tip_toe_distance_scalar = p_tip_toe_distance_scalar;
    tip_toe_speed_scalar = p_tip_toe_speed_scalar;
    tip_toe_angle_max = p_tip_toe_angle_max;
    lift_vertical = p_lift_vertical;
    lift_vertical_scalar = p_lift_vertical_scalar;
    lift_horizontal_scalar = p_lift_horizontal_scalar;
    lift_angle = p_lift_angle;
    apex_vertical = p_apex_vertical;
    apex_vertical_scalar = p_apex_vertical_scalar;
    apex_angle = p_apex_angle;
    drop_vertical = p_drop_vertical;
    drop_vertical_scalar = p_drop_vertical_scalar;
    drop_horizontal_scalar = p_drop_horizontal_scalar;
    drop_angle = p_drop_angle;
    contact_point_ease = p_contact_point_ease;
    contact_point_ease_scalar = p_contact_point_ease_scalar;
    scaling_ease = p_scaling_ease;
  }
};

struct RenIKPlacement : public RefCounted {
  GDCLASS(RenIKPlacement, RefCounted);

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

  Gait forward_gait = Gait(
      1, 0.5 // speed min speed max
      ,
      5, 10, 5, 10, 5, 5, 5, 5,
      0 // ground time min then base and scalar for lift time, apex in time,
        // apex out time, and drop time
      ,
      Math_PI / 2, Math_PI / 4,
      Math_PI / 3 // tip toe distance scalar, speed scalar, and angle max
      ,
      0.0, 0.4, 0.70,
      Math_PI /
          2 // lift vertical, vertical scalar, horizontal scalar, and angle
      ,
      0.0, 0.1, Math_PI / 8 // apex vertical, vertical scalar, angle
      ,
      0.0, 0.05, 0.25, Math_PI / -8 // drop vertical, vertical scalar, angle
      ,
      0.05, 0.4, 0.85); // contact ease, ease scalar, and scaling ease
  Gait backward_gait =
      Gait(0.5, 0.75, 5, 5, 5, 10, 5, 10, 5, 5, 5, 0, 0, 0, 0.025, 0.1, 0.33,
           Math_PI / -8, 0.1, 0.1, Math_PI / 8, 0.0, 0.1, 0.25, Math_PI / 8,
           0.1, 0.4, 0.85);
  Gait sideways_gait =
      Gait(0.75, 0.75, 10, 5, 5, 10, 5, 10, 5, 5, 5, 0, 0, 0, 0.05, 0.05, 0.2,
           0.0, 0.01, 0.1, Math_PI / 8, 0.01, 0.05, 0.25, 0.0, 0.1, 0.4, 0.85);

  float spine_length = 1;
  float left_leg_length = 1;
  float right_leg_length = 1;
  float left_foot_length = 0.125;
  float right_foot_length = 0.125;
  // Hip Placement Adjustments
  float crouch_ratio = 0.4; // Crouching means bending over at the hip while
                            // keeping the spine straight
  float hunch_ratio = 0.6; // Hunching means bending over by arching the spine
  Vector3 hip_offset;
  float hip_follow_head_influence = 0.25;

  // Foot Placement Adjustments - Only takes effect when there are no foot
  // targets These are values when at the slowest walk speed
  float floor_offset = 0.05;
  float raycast_allowance = 0.15; // how far past the max length of the limb
                                  // we'll still consider close enough
  float min_threshold = 0.025;
  float max_threshold =
      0.05; // when all scaling stops and the legs just move faster
  float min_transition_speed = 0.04;
  float rotation_threshold = Math_PI / 4.0;
  float balance_threshold = 0.03;
  float center_of_balance_position =
      0.5; // distance between hips and head that we'll call the center of
           // balance. 0 is at head

  float dangle_ratio = 0.9;
  float dangle_stiffness = 3;
  float dangle_angle = Math_PI / 8;
  float dangle_follow_head = 0.5;
  Vector3 left_hip_offset;
  Vector3 right_hip_offset;

  // Everything scales logarithmically
  float strafe_angle_limit = Math::cos(Math::deg2rad(30.0));
  float step_pace = 0.015;

  Transform3D prev_hip;        // relative to world
  Transform3D prev_left_foot;  // relative to world
  Transform3D prev_right_foot; // relative to world

  Transform3D target_hip;        // relative to world
  Transform3D target_left_foot;  // relative to world
  Transform3D target_right_foot; // relative to world

  // Saracen: these are used between the physics updates to provide smooth local
  // interpolation of leg movement.
  Transform3D interpolated_hip;
  Transform3D interpolated_left_foot;
  Transform3D interpolated_right_foot;

  const Basis foot_basis_offset =
      Basis(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1));

  RenIKPlacement() {}

  void save_previous_transforms();
  void interpolate_transforms(float fraction, bool update_hips,
                              bool update_feet);

  void hip_place(float delta, Transform3D head, Transform3D left_foot,
                 Transform3D right_foot, float twist, bool instant);
  void foot_place(float delta, Transform3D head, Ref<World3D> w3d,
                  bool instant);
  void foot_place(float delta, Transform3D head,
                  PhysicsDirectSpaceState3D::RayResult left_raycast,
                  PhysicsDirectSpaceState3D::RayResult right_raycast,
                  PhysicsDirectSpaceState3D::RayResult laying_raycast,
                  bool instant);
  // All used in leg trace
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
#define LOOP_GROUND_IN 0
#define LOOP_LIFT 1
#define LOOP_APEX_IN 2
#define LOOP_APEX_OUT 3
#define LOOP_DROP 4
#define LOOP_GROUND_OUT 5
  bool fall_override = false;
  bool prone_override = false;
  int walk_state =
      0; // -2 is falling, -1 is transitioning to standing, 0 is stand state, 1
         // is transitioning to stepping, 2 is stepping
  float walk_transition_progress = 0;
  float step_progress = 0;
  Vector3 prevHead;
  uint32_t collision_mask = 1; // the first bit is on but all others are off
  bool collide_with_areas = false;
  bool collide_with_bodies = true;

  // Standing
  Transform3D left_stand;
  Transform3D right_stand;
  Transform3D left_stand_local;  // local to ground
  Transform3D right_stand_local; // local to ground
  Node3D *left_ground = nullptr;
  Node3D *right_ground = nullptr;
  Node3D *prev_left_ground = nullptr;
  Node3D *prev_right_ground = nullptr;
  // Stepping
  Transform3D left_step;
  Transform3D right_step;
  Vector3 left_grounded_stop;
  Vector3 right_grounded_stop;
  const float standing_transition_duration = 0.25;
  const float stepping_transition_duration = 0.2;
  const float laying_transition_duration = 0.25;
  int left_loop_state = 0;
  int right_loop_state = 0;
  float loop_scaling = 0;

  // helpers
  bool is_balanced(Transform3D left, Transform3D right);
  void stand_foot(Transform3D foot, Transform3D &stand,
                  Transform3D &stand_local, Node3D *ground);
  Transform3D dangle_foot(Transform3D head, float distance, float leg_length,
                          Vector3 hip_offset);
  void initialize_loop(Vector3 velocity, Vector3 left_ground,
                       Vector3 right_ground, bool left_grounded,
                       bool right_grounded);
  void loop(Transform3D head, Vector3 velocity, Vector3 left_ground_pos,
            Vector3 left_normal, Vector3 right_ground_pos, Vector3 right_normal,
            bool left_grounded, bool right_grounded, Gait gait);
  void loop_foot(Transform3D &step, Transform3D &stand,
                 Transform3D &stand_local, Node3D *ground, Node3D **prev_ground,
                 int &loop_state, Vector3 &grounded_stop, Transform3D head,
                 float leg_length, float foot_length, Vector3 velocity,
                 float loop_scaling, float step_progress, Vector3 ground_pos,
                 Vector3 ground_normal, Gait gait);
  void step_direction(Vector3 forward, Vector3 side, Vector3 velocity,
                      Vector3 left_ground, Vector3 right_ground,
                      bool left_grounded, bool right_grounded);
  int get_loop_state(float loop_state_scaling, float loop_progress,
                     float &loop_state_progress, Gait gait);
};

#endif // RENIK_PLACEMENT_H
