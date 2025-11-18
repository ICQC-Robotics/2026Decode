package org.firstinspires.ftc.teamcode.Mode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "AutoAim", group = "TeleOp")
public class AutoAim extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam lebron"))
                .setCameraResolution(new Size(640, 480))
                .build();

        telemetry.addLine("Initialized. Press START to Auto Aim.");
        telemetry.update();

        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {

            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (detections.size() == 0) {
                telemetry.addLine("No Tag Detected - Holding");
                stopDrive();
            } else {
                AprilTagDetection tag = detections.get(0);

                double yaw = tag.ftcPose.yaw;
                double kp = 0.01;
                double turnPower = yaw * kp;


                turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

                if (Math.abs(yaw) < 1.0) {

                    stopDrive();
                    telemetry.addLine("Aligned!");
                } else if (yaw > 0) {

                    leftDrive.setPower(-turnPower);
                    rightDrive.setPower(turnPower);
                    telemetry.addData("Turning", "Left");
                } else {
                   
                    leftDrive.setPower(-turnPower);
                    rightDrive.setPower(turnPower);
                    telemetry.addData("Turning", "Right");
                }

                telemetry.addData("Yaw", yaw);
            }

            telemetry.update();
        }

        visionPortal.close();
    }

    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
