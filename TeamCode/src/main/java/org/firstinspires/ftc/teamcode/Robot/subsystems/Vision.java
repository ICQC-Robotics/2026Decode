package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision extends SubsystemBase {
    private final Limelight3A limelight;
    private LLResult lastResult;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "ll");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
    }

    public boolean hasTarget() {
        return lastResult != null && lastResult.isValid();
    }

    public Pose3D getBotPose() {
        return hasTarget() ? lastResult.getBotpose_MT2() : null;
    }

    public Pose2d getBotPose2d() {
        Pose3D pose = getBotPose();
        if (pose == null) return new Pose2d();
        return new Pose2d(
                pose.getPosition().x,
                pose.getPosition().y,
                new Rotation2d(pose.getOrientation().getYaw())
        );
    }

    public double getBotX() {
        Pose3D pose = getBotPose();
        return pose != null ? pose.getPosition().x : 0.0;
    }

    public double getBotY() {
        Pose3D pose = getBotPose();
        return pose != null ? pose.getPosition().y : 0.0;
    }

    public double getBotHeading() {
        Pose3D pose = getBotPose();
        return pose != null ? Math.toDegrees(pose.getOrientation().getYaw()) : 0.0;
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public void reset() {
        limelight.stop();
        limelight.start();
    }
}
