package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Configurable
@Config
@TeleOp(name = "Shooter Tuner")
public class ShooterTuner extends OpMode {
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.1;

    public static double targetRPM = 3000;

    private Robot negabot;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap,
                new GamepadEx(gamepad1),
                new GamepadEx(gamepad2),
                true
        );
    }

    @Override
    public void loop() {
        negabot.shooter.setPIDF(kP, kI, kD, kF);
        negabot.shooter.setVelocity(targetRPM);

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", negabot.shooter.getVelocity());
        telemetry.update();
    }
}
