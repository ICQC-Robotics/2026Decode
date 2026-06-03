package org.firstinspires.ftc.teamcode.PP;

import com.pedropathing.geometry.Pose;

public final class FieldConstants {

    private FieldConstants() {}

    public static final double FIELD_SIZE_IN = 144.0;
    public static final double GOAL_AIM_INSET_IN = 3.0;

    public static final double BLUE_GOAL_CORNER_X = 0.0;
    public static final double BLUE_GOAL_CORNER_Y = FIELD_SIZE_IN;
    public static final double RED_GOAL_CORNER_X = FIELD_SIZE_IN;
    public static final double RED_GOAL_CORNER_Y = FIELD_SIZE_IN;

    // Fixed scoring aim points, not AprilTag targets.
    public static final double BLUE_GOAL_AIM_X = BLUE_GOAL_CORNER_X + GOAL_AIM_INSET_IN;
    public static final double BLUE_GOAL_AIM_Y = BLUE_GOAL_CORNER_Y - GOAL_AIM_INSET_IN;
    public static final Pose BLUE_GOAL_AIM_POINT = new Pose(BLUE_GOAL_AIM_X, BLUE_GOAL_AIM_Y);

    public static final double RED_GOAL_AIM_X = RED_GOAL_CORNER_X - GOAL_AIM_INSET_IN;
    public static final double RED_GOAL_AIM_Y = RED_GOAL_CORNER_Y - GOAL_AIM_INSET_IN;
    public static final Pose RED_GOAL_AIM_POINT = new Pose(RED_GOAL_AIM_X, RED_GOAL_AIM_Y);

    public static Pose goalAimPointForAlliance(boolean blueAlliance) {
        return blueAlliance ? BLUE_GOAL_AIM_POINT.copy() : RED_GOAL_AIM_POINT.copy();
    }

    //corners
    public static final Pose BLUE_CORNER = new Pose(26.241, 133.326, Math.toRadians(54));
    public static final Pose RED_CORNER  = new Pose(7, 8.75, Math.toRadians(90));

}
