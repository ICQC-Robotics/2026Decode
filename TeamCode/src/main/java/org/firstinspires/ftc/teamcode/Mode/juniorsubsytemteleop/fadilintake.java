package org.firstinspires.ftc.teamcode.Mode.juniorsubsytemteleop;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "Fadil Chasity Combo", group = "TeleOp")
public class fadilintake extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private DcMotor shooterMotor, transferMotor;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "m1");
        frontRight = hardwareMap.get(DcMotor.class, "m2");
        backLeft = hardwareMap.get(DcMotor.class, "m3");
        backRight = hardwareMap.get(DcMotor.class, "m4");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam lebron"))
                .setCameraResolution(new Size(640, 480))
                .build();

        telemetry.addLine("Fadil Chasity Combo Initialized");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad1.left_bumper) {
                intake.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            shooterMotor.setPower(0);
            transferMotor.setPower(0);
            if (gamepad1.a) {
                shooterMotor.setPower(1);
                transferMotor.setPower(1);
            }

            List<AprilTagDetection> detections = tagProcessor.getDetections();
            if (detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);
                telemetry.addLine("AprilTag Detected!");
                telemetry.addData("ID", tag.id);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
            } else {
                telemetry.addLine("No Tag Detected");
            }

            telemetry.addData("FrontLeft", frontLeftPower);
            telemetry.addData("FrontRight", frontRightPower);
            telemetry.addData("BackLeft", backLeftPower);
            telemetry.addData("BackRight", backRightPower);
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.addData("Transfer Power", transferMotor.getPower());
            telemetry.update();
        }

        visionPortal.close();
    }
}
