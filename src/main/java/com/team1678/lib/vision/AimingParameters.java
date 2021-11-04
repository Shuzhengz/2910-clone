package com.team1678.lib.vision;

import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.math.Rotation2;

public class AimingParameters {

    private final double range;
    private final RigidTransform2 turret_to_goal;
    private final RigidTransform2 field_to_goal;
    private final Rotation2 turret_to_goal_rotation;
    private final double last_seen_timestamp;
    private final double stability;
    private final Rotation2 field_to_vision_target_normal;
    private final int track_id;

    public AimingParameters(RigidTransform2 turret_to_goal,
                            RigidTransform2 field_to_goal,
                            Rotation2 field_to_vision_target_normal, double last_seen_timestamp,
                            double stability, int track_id) {
        this.turret_to_goal = turret_to_goal;
        this.field_to_vision_target_normal = field_to_vision_target_normal;
        this.field_to_goal = field_to_goal;
        this.range = turret_to_goal.getTranslation().norm();
        this.turret_to_goal_rotation = turret_to_goal.getTranslation().getAngle();
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
        this.track_id = track_id;
    }

    public RigidTransform2 getTurretToGoal() {
        return turret_to_goal;
    }

    public RigidTransform2 getFieldToGoal() {
        return field_to_goal;
    }

    public double getRange() {
        return range;
    }

    public Rotation2 getTurretToGoalRotation() {
        return turret_to_goal_rotation;
    }

    public double getLastSeenTimestamp() {
        return last_seen_timestamp;
    }

    public double getStability() {
        return stability;
    }

    public Rotation2 getFieldToVisionTargetNormal() {
        return field_to_vision_target_normal;
    }

    public int getTrackId() {
        return track_id;
    }
}
