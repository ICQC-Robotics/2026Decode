package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

public class Vision extends SubsystemBase {
    private final MecanumDrive mD;
    private final Limelight3A limelight;
    private LLResult lastResult;

    public Vision(HardwareMap hardwareMap, Drive drive) {
        mD = drive.getMecanumDrive();
        limelight = hardwareMap.get(Limelight3A.class, "ll");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
    }

    public int getTagID() {
        if (lastResult == null || lastResult.getFiducialResults().isEmpty()) {
            return -1;
        }

        int lastIndex = lastResult.getFiducialResults().size() - 1;
        return lastResult.getFiducialResults().get(lastIndex).getFiducialId();
    }

    public Pose3D getBotPose() {
        if(lastResult == null) return null;

        limelight.updateRobotOrientation(Math.toRadians(mD.localizer.getPose().heading.toDouble()));
        Pose3D pose = lastResult.getBotpose_MT2();
        return new Pose3D(pose.getPosition().toUnit(DistanceUnit.INCH), pose.getOrientation());
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

    public double getTx() {
        if (lastResult == null) return Double.NaN;
        if (!lastResult.isValid()) return Double.NaN;
        return lastResult.getTx();
    }

    public double getTa() {
        if (lastResult == null) return Double.NaN;
        if (!lastResult.isValid()) return Double.NaN;
        return lastResult.getTa();
    }

    public double getTy() {
        if (lastResult == null) return Double.NaN;
        if (!lastResult.isValid()) return Double.NaN;
        return lastResult.getTy();
    }
}
