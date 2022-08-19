#include "renik_limb.h"

void RenIKLimb::init(float p_upper_twist_offset, float p_lower_twist_offset,
                     float p_roll_offset, float p_upper_limb_twist,
                     float p_lower_limb_twist,
                     float p_twist_inflection_point_offset,
                     float p_twist_overflow, float p_target_rotation_influence,
                     Vector3 p_pole_offset,
                     Vector3 p_target_position_influence) {
  upper_twist_offset = p_upper_twist_offset;
  lower_twist_offset = p_lower_twist_offset;
  roll_offset = p_roll_offset;
  upper_limb_twist = p_upper_limb_twist;
  lower_limb_twist = p_lower_limb_twist;
  twist_inflection_point_offset = p_twist_inflection_point_offset;
  twist_overflow = p_twist_overflow;
  target_rotation_influence = p_target_rotation_influence;
  pole_offset = p_pole_offset;
  target_position_influence = p_target_position_influence;
}

Transform3D RenIKLimb::get_extra_bones(Skeleton3D *skeleton,
                                       BoneId p_root_bone_id,
                                       BoneId p_tip_bone_id) {
  Transform3D cumulative_rest;
  BoneId current_bone_id = p_tip_bone_id;
  while (current_bone_id != -1 && current_bone_id != p_root_bone_id) {
    current_bone_id = skeleton->get_bone_parent(current_bone_id);
    if (current_bone_id == -1 || current_bone_id == p_root_bone_id) {
      break;
    }
    cumulative_rest =
        skeleton->get_bone_rest(current_bone_id) * cumulative_rest;
  }

  return cumulative_rest;
}
Vector<BoneId> RenIKLimb::get_extra_bone_ids(Skeleton3D *skeleton,
                                             BoneId p_root_bone_id,
                                             BoneId p_tip_bone_id) {
  Vector<BoneId> output;
  BoneId current_bone_id = p_tip_bone_id;
  while (current_bone_id != -1) {
    current_bone_id = skeleton->get_bone_parent(current_bone_id);
    if (current_bone_id == -1 || current_bone_id == p_root_bone_id) {
      break;
    }
    output.push_back(current_bone_id);
  }

  return output;
}

void RenIKLimb::update(Skeleton3D *skeleton) {
  if (skeleton != nullptr && leaf_id >= 0) {
    lower_id = lower_id >= 0 ? lower_id : skeleton->get_bone_parent(leaf_id);
    if (lower_id >= 0) {
      upper_id = upper_id >= 0 ? upper_id : skeleton->get_bone_parent(lower_id);
      if (upper_id >= 0) {
        // leaf = get_full_rest(skeleton, leaf_id, lower_id);
        // lower = get_full_rest(skeleton, lower_id, upper_id);
        // upper = skeleton->get_bone_rest(upper_id);

        lower_extra_bones = get_extra_bones(
            skeleton, lower_id,
            leaf_id); // lower bone + all bones after that except the leaf
        upper_extra_bones = get_extra_bones(
            skeleton, upper_id,
            lower_id); // upper bone + all bones between upper and lower
        lower_extra_bone_ids = get_extra_bone_ids(skeleton, lower_id, leaf_id);
        upper_extra_bone_ids = get_extra_bone_ids(skeleton, upper_id, lower_id);

        leaf = Transform3D(Basis(), skeleton->get_bone_rest(leaf_id).get_origin());
        lower = Transform3D(Basis(), skeleton->get_bone_rest(lower_id).get_origin());
        upper = Transform3D(Basis(), skeleton->get_bone_rest(upper_id).get_origin());
      }
    }
  }
}

void RenIKLimb::set_leaf(Skeleton3D *skeleton, BoneId p_leaf_id) {
  leaf_id = p_leaf_id;
  update(skeleton);
}

void RenIKLimb::set_upper(Skeleton3D *skeleton, BoneId p_upper_id) {
  upper_id = p_upper_id;
  update(skeleton);
}

void RenIKLimb::set_lower(Skeleton3D *skeleton, BoneId p_lower_id) {
  lower_id = p_lower_id;
  update(skeleton);
}

bool RenIKLimb::is_valid() {
  return upper_id >= 0 && lower_id >= 0 && leaf_id >= 0;
}

bool RenIKLimb::is_valid_in_skeleton(Skeleton3D *skeleton) {
  if (skeleton == nullptr || upper_id < 0 || lower_id < 0 || leaf_id < 0 ||
      upper_id >= skeleton->get_bone_count() ||
      lower_id >= skeleton->get_bone_count() ||
      leaf_id >= skeleton->get_bone_count()) {
    return false;
  }
  BoneId curr = skeleton->get_bone_parent(leaf_id);
  while (curr != -1 && curr != lower_id) {
    curr = skeleton->get_bone_parent(curr);
  }
  while (curr != -1 && curr != upper_id) {
    curr = skeleton->get_bone_parent(curr);
  }
  return curr != -1;
}

BoneId RenIKLimb::get_leaf_bone() { return leaf_id; }
BoneId RenIKLimb::get_lower_bone() { return lower_id; }
BoneId RenIKLimb::get_upper_bone() { return upper_id; }
Transform3D RenIKLimb::get_upper() { return upper; }
Transform3D RenIKLimb::get_lower() { return lower; }
Transform3D RenIKLimb::get_leaf() { return leaf; }
