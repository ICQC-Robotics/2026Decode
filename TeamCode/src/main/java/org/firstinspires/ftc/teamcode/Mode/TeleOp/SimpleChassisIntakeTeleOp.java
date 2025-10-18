package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Simple Chassis + Intake", group = "TeleOp")
public class SimpleChassisIntakeTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "m1");
        frontRight = hardwareMap.get(DcMotor.class, "m2");
        backLeft   = hardwareMap.get(DcMotor.class, "m3");
        backRight  = hardwareMap.get(DcMotor.class, "m4");
        intake     = hardwareMap.get(DcMotor.class, "intake");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- DRIVING (Tank style) ---
        double leftPower  = -gamepad1.left_stick_y;   // invert so up is forward
        double rightPower = -gamepad1.right_stick_y;

        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);

        // --- INTAKE ---
        // If 'A' pressed, run intake forward; else stop
        if (gamepad1.a) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }

        // (Optional) Add reverse with another button:
        // if (gamepad1.b) intake.setPower(-1.0);

        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }
}
