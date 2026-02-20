package org.firstinspires.ftc.teamcode.Robot.Vision.Kalman;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
public final class LimelightToPedro {

    private static final double M_TO_IN = 39.37007874;
    private static final double HALF_FIELD_IN = 72;

    private LimelightToPedro() {}

    public static Pose toPedroPose(Pose3D llPose) {
        if (llPose == null) return null;

        double xLL_in = llPose.getPosition().x * M_TO_IN;
        double yLL_in = llPose.getPosition().y * M_TO_IN;

        double xPedro = yLL_in + HALF_FIELD_IN;
        double yPedro = -xLL_in + HALF_FIELD_IN;

        double yawLL_deg = llPose.getOrientation().getYaw();
        double headingPedro_rad = Math.toRadians(yawLL_deg - 90.0);

        return new Pose(xPedro, yPedro, headingPedro_rad);
    }
}
