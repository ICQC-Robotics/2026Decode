package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

    public double getTx() {
        return hasTarget() ? lastResult.getTx() : 0.0;
    }

    public double getTy() {
        return hasTarget() ? lastResult.getTy() : 0.0;
    }

    public double getTa() {
        return hasTarget() ? lastResult.getTa() : 0.0;
    }

    public Pose3D getBotPose() {
        return hasTarget() ? lastResult.getBotpose() : null;
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }
}

