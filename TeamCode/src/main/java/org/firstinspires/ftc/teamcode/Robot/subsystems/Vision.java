package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision extends SubsystemBase {

    public enum Alliance {
        BLUE(20),
        RED(24);

        public final int tagId;
        Alliance(int tagId) { this.tagId = tagId; }
    }

    private final Limelight3A limelight;
    private LLResult lastResult;

    private int targetTagId = Alliance.BLUE.tagId;

    private double tx = Double.NaN;
    private double ty = Double.NaN;
    private double ta = Double.NaN;

    private static final double TX_DEADBAND_DEG = 0.3;

    public Vision(Limelight3A vision) {
        limelight = vision;
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
        updateTargetThisFrame();
    }

    public void setAlliance(Alliance alliance) {
        targetTagId = alliance.tagId;
    }

    public void setTargetTagId(int id) {
        targetTagId = id;
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

    public int getCurrentTagID() {
        LLResultTypes.FiducialResult f = getTargetFiducial();
        if (f == null) return -1;
        return (int) f.getFiducialId();
    }

    public double getTx() {
        if (Double.isNaN(tx)) return Double.NaN;
        return applyDeadband(tx);
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    private LLResultTypes.FiducialResult getTargetFiducial() {
        if (lastResult == null || !lastResult.isValid()) return null;
        if (lastResult.getFiducialResults() == null) return null;

        for (LLResultTypes.FiducialResult f : lastResult.getFiducialResults()) {
            if ((int) f.getFiducialId() == targetTagId) return f;
        }
        return null;
    }

    private void updateTargetThisFrame() {
        tx = Double.NaN;
        ty = Double.NaN;
        ta = Double.NaN;

        if (lastResult == null || !lastResult.isValid()) return;
        if (lastResult.getFiducialResults() == null || lastResult.getFiducialResults().isEmpty()) return;

        LLResultTypes.FiducialResult f = getTargetFiducial();
        if (f == null) return;

        double newTx = lastResult.getTx();
        double newTy = lastResult.getTy();
        double newTa = lastResult.getTa();
        if (Double.isNaN(newTx) || Double.isNaN(newTy) || Double.isNaN(newTa)) return;

        tx = newTx;
        ty = newTy;
        ta = newTa;
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) <= TX_DEADBAND_DEG) {
            return 0.0;
        }
        return value;
    }

    public Alliance getAlliance() {
        if (targetTagId == Alliance.BLUE.tagId) {
            return Alliance.BLUE;
        } else {
            return Alliance.RED;
        }
    }
}
