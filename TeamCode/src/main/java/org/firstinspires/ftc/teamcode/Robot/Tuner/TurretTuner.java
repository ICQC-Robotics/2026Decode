package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Configurable
@Config
@TeleOp(name = "Turret Tuner")
public class TurretTuner extends OpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double targetDeg = 135.0;
    public static double toleranceDeg = 1.0;

    private Robot negabot;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        negabot.turret.setPIDF(kP, kI, kD, kF);
        negabot.turret.setTargetDeg(targetDeg);

        double current = negabot.turret.getAngleDeg();
        double target = negabot.turret.getTargetDeg();
        double error = target - current;

        telemetry.addData("Target (deg)", target);
        telemetry.addData("Current (deg)", current);
        telemetry.addData("Error (deg)", error);
        telemetry.addData("At Target", negabot.turret.atTarget(toleranceDeg));

        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);

        telemetry.update();
    }
}
