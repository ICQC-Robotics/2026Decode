package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Configurable
@Config
@TeleOp(name = "Shooter Tuner")
public class ShooterTuner extends OpMode {
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.1;

    Servo servo1, servo3;
    public static double s1 = 0.5;
    public static double targetRPM = 3000;
    private Limelight3A limelight;
    private Robot negabot;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap,
                telemetry, new Pose(72, 72)
        );
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        limelight = hardwareMap.get(Limelight3A.class, "ll");
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        double distance = (29.5 - 12) / Math.tan(Math.toRadians(12 + result.getTy()));

        negabot.shooter.setPIDF(kP, kI, kD, kF);
        negabot.shooter.setVelocity(targetRPM);
        negabot.intake.setSpeed(-1);
        servo1.setPosition(s1);
        servo3.setPosition(0.47);

        telemetry.addData("distance", distance);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", negabot.shooter.getVelocity());
        telemetry.update();

    }
}
