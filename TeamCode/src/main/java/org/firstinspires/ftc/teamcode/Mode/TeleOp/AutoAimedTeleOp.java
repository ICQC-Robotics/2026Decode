package org.firstinspires.ftc.teamcode.Mode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AutoAimedTeleOp")
public class AutoAimedTeleOp extends OpMode {
    DcMotorEx fL, fR, bL, bR, intake, shooter, transfer;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    private void stopDrive() {
        fL.setPower(0);
        bR.setPower(0);
    }

    @Override
    public void init() {
        fL = hardwareMap.get(DcMotorEx.class, "m1");
        fR = hardwareMap.get(DcMotorEx.class, "m2");
        bL = hardwareMap.get(DcMotorEx.class, "m3");
        bR = hardwareMap.get(DcMotorEx.class, "m4");

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam lebron"))
                .setCameraResolution(new Size(640, 480))
                .build();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setVelocityPIDFCoefficients(70, 0, 0, 70);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double flPower = drive + strafe + turn;
        double frPower = drive - strafe - turn;
        double blPower = drive - strafe + turn;
        double brPower = drive + strafe - turn;

        fL.setPower(flPower);
        fR.setPower(frPower);
        bL.setPower(blPower);
        bR.setPower(brPower);

        if(gamepad1.left_bumper){
            intake.setPower(1);
        }
        else{
            intake.setPower(0);
        }
        if(gamepad1.right_bumper){
            transfer.setPower(-1);
        }
        else{
            transfer.setPower(0);
        }
        if(gamepad1.left_trigger>0.1){
            shooter.setVelocity(0);
        } else if (gamepad1.right_trigger>0.1){
            shooter.setVelocity(2100);
        }

        //AutoAim with X
        if (gamepad1.a){
            while (true) {
                List<AprilTagDetection> detections = tagProcessor.getDetections();
                if (gamepad1.a){break;}
                telemetry.update();
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

                        fL.setPower(-turnPower);
                        bL.setPower(-turnPower);
                        fR.setPower(turnPower);
                        bR.setPower(turnPower);
                        telemetry.addData("Turning", "Left");
                    } else {

                        fL.setPower(-turnPower);
                        bL.setPower(-turnPower);
                        fR.setPower(turnPower);
                        bR.setPower(turnPower);
                        telemetry.addData("Turning", "Right");
                    }

                    telemetry.addData("Yaw", yaw);
                }
            }

        }
    }
}

