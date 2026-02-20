package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult lastResult;

    public Limelight(Limelight3A vision) {
        limelight = vision;
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
    }

    public Pose3D getBotposeMT2() {
        if (lastResult == null || !lastResult.isValid()) return null;
        return lastResult.getBotpose_MT2();
    }

    public void updateRobotYawDeg(double robotYawDeg) {
        limelight.updateRobotOrientation(robotYawDeg);
    }
}
