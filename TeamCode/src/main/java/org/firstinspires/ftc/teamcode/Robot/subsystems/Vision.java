package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Vision extends SubsystemBase {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private AprilTagDetection latestDetection = null;

    public Vision(WebcamName webcam) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                // .setDrawAxes(true)
                // .setDrawTagOutline(true)
                // .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(false)
                .build();
    }

    public boolean hasTag() {
        return latestDetection != null;
    }

    public AprilTagDetection getTag() {
        return latestDetection;
    }

    public int getTagID() {
        return hasTag() ? latestDetection.id : -1;
    }

    public double getTagX() {
        return hasTag() ? latestDetection.ftcPose.x : Double.NaN;
    }

    public double getTagY() {
        return hasTag() ? latestDetection.ftcPose.y : Double.NaN;
    }

    public double getTagZ() {
        return hasTag() ? latestDetection.ftcPose.z : Double.NaN;
    }

    public double getTagYaw() {
        return hasTag() ? latestDetection.ftcPose.yaw : Double.NaN;
    }

    public double getTagPitch() {
        return hasTag() ? latestDetection.ftcPose.pitch : Double.NaN;
    }

    public double getTagRoll() {
        return hasTag() ? latestDetection.ftcPose.roll : Double.NaN;
    }

    public List<AprilTagDetection> getAllTags() {
        return aprilTagProcessor.getDetections();
    }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }

    @Override
    public void periodic() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        latestDetection = !detections.isEmpty() ? detections.get(0) : null;
    }
}
