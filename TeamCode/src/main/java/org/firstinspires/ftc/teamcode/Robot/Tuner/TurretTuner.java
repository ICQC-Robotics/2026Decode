package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.controller.PIDController;

@TeleOp(name = "Turret PID Tuner", group = "")
public class TurretTuner extends OpMode {

    DcMotorEx turretMotor;

    double P = 0.01;
    double D = 0.0;

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    static final double GEAR_RATIO = 2.9047619048;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 270;

    double ticksPerDeg;
    double zeroOffset;

    double targetAngle = 135;

    PIDController pid;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ticksPerDeg =
                (turretMotor.getMotorType().getTicksPerRev() * GEAR_RATIO) / 360.0;

        zeroOffset = turretMotor.getCurrentPosition();

        pid = new PIDController(P, 0, D);
        pid.setTolerance(1.0);

        telemetry.addLine("initialized, check the code on how to use with controllers");
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) targetAngle -= 1;
        if (gamepad1.right_bumper) targetAngle += 1;

        targetAngle = clamp(targetAngle, MIN_ANGLE, MAX_ANGLE);

        if (gamepad1.b) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpad_up) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_down) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpad_right) {
            D += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_left) {
            D -= stepSizes[stepIndex];
        }

        pid.setPID(P, 0, D);

        double currentAngle = getAngleDeg();
        double out = pid.calculate(currentAngle, targetAngle);

        out = clamp(out, -0.6, 0.6);

        turretMotor.setPower(out);

        telemetry.addData("Target Angle", "%.1f", targetAngle);
        telemetry.addData("Current Angle", "%.1f", currentAngle);
        telemetry.addData("Error", "%.1f", targetAngle - currentAngle);
        telemetry.addLine("----------------------");
        telemetry.addData("P", "%.5f", P);
        telemetry.addData("D", "%.5f", D);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.update();
    }

    double getAngleDeg() {
        return (turretMotor.getCurrentPosition() - zeroOffset) / ticksPerDeg;
    }

    double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}