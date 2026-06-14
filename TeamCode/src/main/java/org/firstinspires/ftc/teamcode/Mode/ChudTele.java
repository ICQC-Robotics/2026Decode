package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;

@TeleOp(group=".")
public class ChudTele extends CommandOpMode {
    GamepadEx g;
    Robot negabot;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        // resetTurret = false so the turret encoder is preserved for the auto carryover.
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0, 0), false);
        negabot.reset();

        // Restore the pose + turret angle saved at the end of auto (no encoder reset).
        negabot.restoreFromAuto();

        negabot.shooter.setDefaultCommand(new ShooterStandBy(negabot.shooter, negabot.drive));
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


        negabot.Action(g,
                        GamepadKeys.Button.A,

                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                                }),
                                new WaitCommand(100),
                                new AutoIntake(negabot.intake, negabot.wait).accept(),
                                new WaitCommand(500),
                                new AutoIntake(negabot.intake, negabot.wait).finish(),
                                new InstantCommand(() -> {
                                    negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                                })
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
    }

    public void run() {
        negabot.run();
    }
}



