package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
import org.firstinspires.ftc.teamcode.Robot.commands.Relocalize;
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;

@TeleOp(group=".")
public class SoloRed extends CommandOpMode {
    GamepadEx g;
    Robot negabot;

    private boolean shooterStandby = false;
    private boolean turretTracking = false;

    private boolean poseLocked = false;

    private Pose redReset = new Pose(117.759, 133.326, Math.toRadians(126));

    private Robot.Alliance alliance = Robot.Alliance.RED;

    // make this a field so your DPAD buttons can access it
    private PPTracking ppTracking;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0, 0));
        negabot.reset();

        Robot.ALLIANCE = Robot.Alliance.RED;

        //setting position
        if (Robot.LAST_POSE != null) {
            negabot.drive.follower.setPose(Robot.LAST_POSE.copy());
            poseLocked = true;
        }
        else {
            negabot.drive.follower.setPose(new Pose(0,0, 0));
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

        /*

        negabot.Action(
                g,
                GamepadKeys.Button.X,
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                }),
                null
        );
*/
        negabot.Action(
                g,
                GamepadKeys.Button.Y,
                new InstantCommand(() -> {
                    Pose corner = redReset;
                    negabot.drive.follower.setPose(corner);
                    poseLocked = true;
                    ppTracking.resetDegOffset();
                }),

                null
        );

        // create PPTracking ONCE (do not set as default)
        ppTracking = new PPTracking(negabot.turret, negabot.drive, alliance);

        // A: start tracking -> autoaim -> stop tracking
        negabot.Action(
                g,
                GamepadKeys.Button.A,
                new SequentialCommandGroup(
                        new InstantCommand(() -> CommandScheduler.getInstance().schedule(ppTracking)),
                        new AutoAim(negabot.drive, negabot.shooter, negabot.intake, negabot.wait),
                        new Relocalize(negabot.drive, negabot.vision, negabot.turret),
                        new InstantCommand(() -> CommandScheduler.getInstance().cancel(ppTracking))
                ),
                null
        );
        // Keep your DPAD offset buttons here exactly like before
        negabot.Action(
                g,
                GamepadKeys.Button.DPAD_RIGHT,
                new InstantCommand(() -> {
                    ppTracking.incDeg();  // adds +1 degree
                    telemetry.addData("Offset", ppTracking.offset);
                    telemetry.update();
                }),
                null
        );

        negabot.Action(
                g,
                GamepadKeys.Button.DPAD_LEFT,
                new InstantCommand(() -> {
                    ppTracking.decDeg();  // subtracts 1 degree
                    telemetry.addData("Offset", ppTracking.offset);
                    telemetry.update();
                }),
                null
        );
    }

    //this is so then these default commands are activated on run
    public void run() {

        if (!shooterStandby && opModeIsActive() && !turretTracking) {
            negabot.shooter.setDefaultCommand(new ShooterStandBy(negabot.shooter, negabot.drive));

            // IMPORTANT: do NOT set PPTracking as default anymore
            // negabot.turret.setDefaultCommand(ppTracking);

            turretTracking = true;
            shooterStandby = true;
        }



        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        negabot.run();
    }
}
