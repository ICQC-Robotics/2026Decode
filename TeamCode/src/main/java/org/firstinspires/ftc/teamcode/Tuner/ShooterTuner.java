package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    public static double hoodPosition = 0.2;
    public static double feedPower = 0.0;
    private Limelight3A limelight;
    private Robot negabot;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap,
                telemetry, new Pose(72, 72)
        );
        limelight = hardwareMap.get(Limelight3A.class, "ll");
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        double distance = Double.NaN;
        if (result != null && result.isValid()) {
            distance = (29.5 - 12) / Math.tan(Math.toRadians(12 + result.getTy()));
        }

        negabot.shooter.setVelocity(targetRPM);
        negabot.shooter.setHoodPosition(hoodPosition);

        negabot.shooter.periodic();
        negabot.intake.setSpeed(feedPower);

        telemetry.addData("distance", distance);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", negabot.shooter.getVelocity());
        telemetry.addData("Right RPM", negabot.shooter.getRightVelocity());
        telemetry.addData("Left RPM", negabot.shooter.getLeftVelocity());
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Feed Power", feedPower);
        telemetry.update();
    }

}
