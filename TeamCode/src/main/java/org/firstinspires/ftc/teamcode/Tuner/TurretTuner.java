package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@Configurable
@TeleOp(name = "Turret Tuner")
public class TurretTuner extends OpMode {

    public static double kP = 15.0;
    public static double kI = 0.0;
    public static double kD = 0.5;
    public static double kF = 0.1;

    public static double targetDeg = 135.0;
    public static double toleranceDeg = 1.0;

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new Pose(72, 72));
        robot.turret.setPIDF(kP, kI, kD, kF);
        robot.turret.setTargetDeg(targetDeg);
    }

    @Override
    public void loop() {
        if (targetDeg < 0) targetDeg = 0;
        if (targetDeg > 270) targetDeg = 270;
        if (kD < 0) kD = 0;
        if (kF < 0) kF = 0;

        robot.turret.setPIDF(kP, kI, kD, kF);
        robot.turret.setTargetDeg(targetDeg);

        double current = robot.turret.getAngleDeg();
        double error = targetDeg - current;

        telemetry.addData("Target (deg)", targetDeg);
        telemetry.addData("Current (deg)", current);
        telemetry.addData("Error (deg)", error);
        telemetry.addData("At Target", robot.turret.atTarget(toleranceDeg));

        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);

        telemetry.update();
    }
}
