package org.firstinspires.ftc.teamcode.Mode.juniorsubsytemteleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Fadil Chasity Drive", group="Examples")
public class fadilchasity extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "m1");
        frontRight = hardwareMap.get(DcMotor.class, "m2");
        backLeft = hardwareMap.get(DcMotor.class, "m3");
        backRight = hardwareMap.get(DcMotor.class, "m4");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // Gamepad inputs
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x;  // strafe left/right
            double rx = gamepad1.right_stick_x; // rotation

            // Mecanum drive formulas
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Normalize the powers if any exceed 1.0
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
}
