package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Configurable
@TeleOp(name = "Shooter Tuner")
public class ShooterTuner extends OpMode {
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;

    public static double targetRPM = 3000;

    private Robot negabot;

    @Override
    public void init() {
        kP = .005;
        kI = 0;
        kD = 0;
        kF = .1;
        negabot = new Robot(hardwareMap, new GamepadEx(gamepad1), new GamepadEx(gamepad2), true);
        negabot.shooter.setPIDF(kP, kI, kD, kF);
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
