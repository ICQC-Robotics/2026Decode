package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@Configurable
@TeleOp(name = "Turret Tuner")
public class TurretTuner extends OpMode {

    public static double kP = 0.02;
    public static double kD = 0.001;
    public static double kF = 0.05;

    public static double targetDeg = 135.0;
    public static double toleranceDeg = 1.0;

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new Pose(72, 72));
    }

    @Override
    public void loop() {

        targetDeg = Range.clip(targetDeg, 0, 270);
        kD = Math.max(0, kD);
        kF = Math.max(0, kF);

        robot.turret.setPIDF(kP, kD, kF);
        robot.turret.setTargetDeg(targetDeg);

        double current = robot.turret.getAngleDeg();

        telemetry.addData("Target (deg)", targetDeg);
        telemetry.addData("Current (deg)", current);
        telemetry.addData("Error (deg)", targetDeg - current);
        telemetry.addData("At Target", robot.turret.atTarget(toleranceDeg));

        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);

        telemetry.update();
    }
}
