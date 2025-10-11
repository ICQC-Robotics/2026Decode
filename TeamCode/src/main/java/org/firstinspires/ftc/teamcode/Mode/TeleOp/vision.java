// ts coded by Shariq Shaik

package org.firstinspires.ftc.teamcode.Mode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "webcamtesting", group = "TeleOp")
public class vision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Build the AprilTag processor ---
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // --- Build the Vision Portal ---
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam lebron"))
                .setCameraResolution(new Size(640, 480))
                .build();

        telemetry.addLine("Initialized. Press START to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);

                telemetry.addLine("Tag Detected!");
                telemetry.addData("ID", tag.id);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            } else {
                telemetry.addLine("No Tag Detected");
            }

            telemetry.update();
        }

        // Stop camera stream when done
        visionPortal.close();
    }
}
