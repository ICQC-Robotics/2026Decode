package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
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

    private Robot negabot;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap,
                telemetry
        );
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");
    }

    @Override
    public void loop() {
        negabot.shooter.setPIDF(kP, kI, kD, kF);
        negabot.shooter.setVelocity(targetRPM);
        negabot.intake.setSpeed(-1);
        servo1.setPosition(s1);
        servo3.setPosition(0.47);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", negabot.shooter.getVelocity());
        telemetry.update();
    }
}
