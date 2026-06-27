package org.firstinspires.ftc.teamcode.Mode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;

/**
 * Stripped-down RED teleop for far shooting: drive and intake work exactly like {@link SoloRed},
 * but there is no auto-aim or pose tracking. Instead the turret is pinned to a fixed angle and the
 * flywheel is pinned to a fixed velocity for the whole match. Tune both live in Dashboard.
 */
@Config
@TeleOp(name = "BasicFarRed")
public class BasicFarRed extends CommandOpMode {
    // The fixed shot. Turret holds this angle and the flywheel holds this velocity all match.
    public static double VELOCITY         = 3900; // RPM
    public static double TURRET_ANGLE_DEG = 135;  // 135 = forward in the turret's frame

    GamepadEx g;
    Robot negabot;

    private boolean defaultsSet = false;

    private final Pose redReset = new Pose(117.759, 133.326, Math.toRadians(126));

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, telemetry, new Pose(0, 0, 0));
        negabot.reset();

        Robot.ALLIANCE = Robot.Alliance.RED;

        // setting position
        if (Robot.LAST_POSE != null) {
            negabot.drive.follower.setPose(Robot.LAST_POSE.copy());
        } else {
            negabot.drive.follower.setPose(new Pose(0, 0, 0));
        }

        negabot.drive.setDefaultCommand(new DriveCommand(negabot.drive, g));

        negabot.Action(g,
                GamepadKeys.Button.RIGHT_BUMPER,
                new AutoIntake(negabot.intake, negabot.wait).accept(),
                new AutoIntake(negabot.intake, negabot.wait).finish()
        );

        negabot.Action(g,
                GamepadKeys.Button.LEFT_BUMPER,
                new AutoIntake(negabot.intake, negabot.wait).reject(),
                new AutoIntake(negabot.intake, negabot.wait).finish()
        );

        negabot.Action(
                g,
                GamepadKeys.Button.Y,
                new InstantCommand(() -> negabot.drive.follower.setPose(redReset)),
                null
        );
    }

    // default commands are activated on run so they latch onto the subsystems once the opmode is live
    public void run() {
        if (!defaultsSet && opModeIsActive()) {
            // Pin the turret to a fixed angle and the flywheel to a fixed velocity for the whole match.
            negabot.turret.setDefaultCommand(
                    new RunCommand(() -> negabot.turret.setTargetDeg(TURRET_ANGLE_DEG), negabot.turret));
            negabot.shooter.setDefaultCommand(
                    new RunCommand(() -> negabot.shooter.setVelocity(VELOCITY), negabot.shooter));
            defaultsSet = true;
        }

        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        negabot.run();
    }
}
