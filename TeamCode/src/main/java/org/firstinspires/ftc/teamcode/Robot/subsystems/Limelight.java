package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class Limelight extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult lastResult;

    private double tx = Double.NaN;
    private double ty = Double.NaN;
    private double ta = Double.NaN;


    // ll mounting constants
    private static final double LIMELIGHT_HEIGHT_IN = 0;
    private static final double APRILTAG_HEIGHT_IN = 29.5;
    private static final double LIMELIGHT_PITCH_DEG = 0;

    public Limelight(Limelight3A vision) {
        limelight = vision;
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
        updateThisFrame();
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public void reset() {
        limelight.stop();
        limelight.start();
        lastResult = null;
        tx = Double.NaN;
        ty = Double.NaN;
        ta = Double.NaN;
    }

    public boolean hasTarget() {
        return !Double.isNaN(tx);
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    private void updateThisFrame() {
        tx = Double.NaN;
        ty = Double.NaN;
        ta = Double.NaN;

        if (lastResult == null || !lastResult.isValid()) return;
        if (lastResult.getFiducialResults() == null || lastResult.getFiducialResults().isEmpty()) return;

        double newTx = lastResult.getTx();
        double newTy = lastResult.getTy();
        double newTa = lastResult.getTa();
        if (Double.isNaN(newTx) || Double.isNaN(newTy) || Double.isNaN(newTa)) return;

        tx = newTx;
        ty = newTy;
        ta = newTa;
    }
}
