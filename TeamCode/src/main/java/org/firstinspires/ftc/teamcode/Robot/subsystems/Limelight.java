package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.pedropathing.geometry.Pose;


public class Limelight extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult lastResult;
    public final double turretRadius = 0;
    public final double turretOffsetX = 0;
    public final double turretOffsetY = 0;

    public Limelight(Limelight3A vision) {
        limelight = vision;
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
    }

    public Pose getBotposeMT1(int t) {
        if (lastResult == null || !lastResult.isValid()) return null;
        t -= 135;
        t *= -1;
        Pose3D pose = lastResult.getBotpose();
        Pose p = new Pose(pose.getPosition().x, pose.getPosition().y, pose.getOrientation().getYaw(AngleUnit.RADIANS));
        p.setHeading(p.getHeading() + t);
        p = p.withX(p.getX() + turretRadius * Math.sin(t));
        p = p.withY(p.getY() - turretRadius * Math.cos(t));
        return p.withX(p.getX() - turretOffsetX).withY(p.getY() - turretOffsetY);
    }

    public void updateRobotYawDeg(double robotYawDeg) {
        limelight.updateRobotOrientation(robotYawDeg);
    }
}
