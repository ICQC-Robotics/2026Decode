package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot.Vision.IntakeColorProcessing;
import org.firstinspires.ftc.vision.VisionPortal;

public class Vision extends SubsystemBase {

    //ll
    private final Limelight3A limelight;
    private LLResult lastResult;
    public final double turretRadius = 2.75;
    public final double turretOffsetX = 0;
    public final double turretOffsetY = 3;

    //webcam
    private VisionPortal portal;
    private IntakeColorProcessing processor;

    public Vision(Limelight3A vision, HardwareMap hardwareMap, String cameraName) {
        limelight = vision;
        limelight.pipelineSwitch(0);
        limelight.start();

        processor = new IntakeColorProcessing();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .addProcessor(processor)
                .build();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
    }

    public Pose getBotposeMT1(double t) {
        if (lastResult == null || !lastResult.isValid()) return null;
        t -= 135;
        double tRad = Math.toRadians(t);
        Pose3D pose = lastResult.getBotpose();
        Pose p = new Pose(
                pose.getPosition().x,
                pose.getPosition().y,
                pose.getOrientation().getYaw(AngleUnit.RADIANS)
        );

        p.setHeading(p.getHeading() + tRad);
        p = p.withX(p.getX() + turretRadius * Math.sin(tRad));
        p = p.withY(p.getY() - turretRadius * Math.cos(tRad));

        return p.withX(p.getX() - turretOffsetX).withY(p.getY() - turretOffsetY);
    }

    public void updateRobotYawDeg(double robotYawDeg) {
        limelight.updateRobotOrientation(robotYawDeg);
    }

    public IntakeColorProcessing.Zone getZone() {
        return processor.getDetectedZone();
    }

    public void stop() {
        portal.stopStreaming();
    }

    public void close() {
        portal.close();
    }
}