package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.PPTracking;
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;

@TeleOp(group=".")
public class SoloBlue extends CommandOpMode {
    GamepadEx g;
    Robot negabot;

    private boolean shooterStandby = false;
    private boolean turretTracking = false;

    private boolean poseLocked = false;

    private Robot.Alliance alliance = Robot.Alliance.BLUE;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0, 0));
        negabot.reset();


        //setting position
        if (Robot.LAST_POSE != null) {
            negabot.drive.follower.setPose(Robot.LAST_POSE.copy());
            poseLocked = true;
        }
        else {
            negabot.drive.follower.setPose(new Pose(0,0, 0));

        }

        negabot.drive.setDefaultCommand(new DriveCommand(negabot.drive, g));
       // negabot.turret.restoreAngleDeg(Robot.LAST_TURRET_DEG);

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


        negabot.Action(g,
                       GamepadKeys.Button.A,
                       new AutoAim(negabot.drive,
                                   negabot.shooter,
                                   negabot.intake,
                                   negabot.wait
                       ),
                       null
        );

        negabot.Action(
                g,
                GamepadKeys.Button.Y,
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                }),
                null
        );

        negabot.Action(
                g,
                GamepadKeys.Button.DPAD_UP,
                new InstantCommand(() -> {
                    if (!opModeIsActive() || Robot.LAST_POSE != null || poseLocked) return;
                    Pose corner = FieldConstants.BLUE_CORNER;
                    negabot.drive.follower.setPose(corner);
                    poseLocked = true;
                }),
                null
        );


    }

    //this is so then these default commands are activated on run
    public void run() {
        if (!shooterStandby && opModeIsActive() && !turretTracking) {
            negabot.shooter.setDefaultCommand(new ShooterStandBy(negabot.shooter, negabot.drive));
            negabot.turret.setDefaultCommand(new PPTracking(negabot.turret, negabot.drive, alliance));
            turretTracking = true;
            shooterStandby = true;
        }

        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        negabot.run();
    }
}



