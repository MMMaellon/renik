#ifndef RENIK_CHAIN_H
#define RENIK_CHAIN_H

#include <scene/3d/skeleton_3d.h>

struct RenIKChain : public Resource {
  GDCLASS(RenIKChain, Resource);

public:
  struct Joint {
    Quaternion rotation;
    BoneId id;
    Vector3 relative_prev;
    Vector3 relative_next;
    float prev_distance = 0;
    float next_distance = 0;

    float root_influence = 0;
    float leaf_influence = 0;
    float twist_influence = 1;
  };

private:
  BoneId root_bone = -1;
  BoneId first_bone = -1;
  BoneId leaf_bone = -1;

  Vector<Joint> joints;
  float total_length = 0;
  Transform3D rest_leaf;
  void init_chain(Skeleton3D *skeleton);
  float root_influence =
      0; // how much the start bone is influenced by the root rotation
  float leaf_influence =
      0; // how much the end bone is influenced by the goal rotation
  float twist_influence =
      1; // How much the chain tries to twist to follow the end when the start
         // is facing a different direction
  float twist_start = 0; // Where along the chain the twisting starts

public:
  void init(Vector3 p_chain_curve_direction, float p_root_influence,
            float p_leaf_influence, float p_twist_influence,
            float p_twist_start);
  void set_root_bone(Skeleton3D *skeleton, BoneId p_root_bone);
  void set_leaf_bone(Skeleton3D *skeleton, BoneId p_leaf_bone);
  bool is_valid();
  Vector3 chain_curve_direction; // This defines which way to prebend it
  float get_total_length();
  Vector<RenIKChain::Joint> get_joints();
  Transform3D get_relative_rest_leaf();
  BoneId get_first_bone();
  BoneId get_root_bone();
  BoneId get_leaf_bone();

  float get_root_stiffness();
  void set_root_stiffness(Skeleton3D *skeleton, float stiffness);
  float get_leaf_stiffness();
  void set_leaf_stiffness(Skeleton3D *skeleton, float stiffness);
  float get_twist();
  void set_twist(Skeleton3D *skeleton, float p_twist);
  float get_twist_start();
  void set_twist_start(Skeleton3D *skeleton, float p_twist_start);
  bool contains_bone(Skeleton3D *skeleton, BoneId bone);
};

#endif
