package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;

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

    // Which goal the distance is measured to. Toggle in Dashboard.
    public static boolean BLUE_ALLIANCE = true;

    // Triangle (Button.Y) snaps the pose here, same idea as the Solo reset.
    // Tune these in Dashboard to match the physical spot you reset from.
    public static double RESET_X = 32.54;
    public static double RESET_Y = 134.56;
    public static double RESET_HEADING_DEG = 90.0;

    private Robot negabot;
    private GamepadEx driver;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap,
                telemetry, new Pose(72, 72)
        );
        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        // Alliance -> which goal the distance is measured to
        Robot.ALLIANCE = BLUE_ALLIANCE ? Robot.Alliance.BLUE : Robot.Alliance.RED;

        // ── input ────────────────────────────────────────────────────────────
        driver.readButtons();

        // Triangle = relocalize: snap the follower pose to the known reset spot
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            negabot.drive.follower.setPose(
                    new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_HEADING_DEG)));
        }

        // ── drive + localization ─────────────────────────────────────────────
        negabot.drive.follower.update();      // localize first (matches Solo's order)
        negabot.drive.movement(driver);       // then apply stick powers so they win

        // ── shooter ──────────────────────────────────────────────────────────
        negabot.shooter.setVelocity(targetRPM);
        negabot.shooter.setHoodPosition(hoodPosition);
        negabot.shooter.periodic();
        negabot.intake.setSpeed(feedPower);

        // ── telemetry ────────────────────────────────────────────────────────
        telemetry.addData("Goal Distance (in)", AutoAim.calculateDistanceIn(negabot.drive));
        telemetry.addData("Alliance", BLUE_ALLIANCE ? "BLUE" : "RED");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", negabot.shooter.getVelocity());
        telemetry.addData("Right RPM", negabot.shooter.getRightVelocity());
        telemetry.addData("Left RPM", negabot.shooter.getLeftVelocity());
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Feed Power", feedPower);
        telemetry.addLine("— pose (Triangle to relocalize) —");
        telemetry.addData("X", negabot.drive.getX());
        telemetry.addData("Y", negabot.drive.getY());
        telemetry.addData("Heading (deg)", negabot.drive.getHeadingDeg());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (negabot != null) {
            negabot.shooter.stop();
            negabot.intake.setSpeed(0);
            negabot.drive.stop();
        }
    }
}
