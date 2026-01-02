package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@TeleOp(name = "Turret Tuner")
public class TurretTuner extends OpMode {

    public static double kP = 15.0;
    public static double kI = 0.0;
    public static double kD = 0.5;
    public static double kF = 0.1;

    public static double targetDeg = 135.0;
    public static double toleranceDeg = 1.0;

    //change
    public static double dStep = 0.2;
    public static double fStep = 0.05;
    public static double targetStep = 2.0;

    private Robot robot;

    private double lastP, lastI, lastD, lastF;
    private double lastTarget;

    private boolean lastUp, lastDown, lastLeft, lastRight;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new Pose(72, 72));

        lastP = kP;
        lastI = kI;
        lastD = kD;
        lastF = kF;
        lastTarget = targetDeg;

        robot.turret.setPIDF(kP, kI, kD, kF);
        robot.turret.setTargetDeg(targetDeg);
    }

    @Override
    public void loop() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (up && !lastUp) targetDeg += targetStep;
        if (down && !lastDown) targetDeg -= targetStep;

        if (right && !lastRight) kD += dStep;
        if (left && !lastLeft)  kD -= dStep;

        if (gamepad1.right_bumper) kF += fStep;
        if (gamepad1.left_bumper)  kF -= fStep;

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;

        if (targetDeg < 0) targetDeg = 0;
        if (targetDeg > 270) targetDeg = 270;
        if (kD < 0) kD = 0;
        if (kF < 0) kF = 0;

        if (kP != lastP || kI != lastI || kD != lastD || kF != lastF) {
            robot.turret.setPIDF(kP, kI, kD, kF);
            lastP = kP;
            lastI = kI;
            lastD = kD;
            lastF = kF;
        }

        if (targetDeg != lastTarget) {
            robot.turret.setTargetDeg(targetDeg);
            lastTarget = targetDeg;
        }

        double current = robot.turret.getAngleDeg();
        double error = lastTarget - current;

        telemetry.addData("Target (deg)", lastTarget);
        telemetry.addData("Current (deg)", current);
        telemetry.addData("Error (deg)", error);
        telemetry.addData("At Target", robot.turret.atTarget(toleranceDeg));

        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);

        telemetry.update();
    }
}
