package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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
public class Duo extends CommandOpMode {
    GamepadEx g1;
    GamepadEx g2;
    Robot negabot;

    @Override
    public void initialize() {
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0));
        negabot.reset();

        negabot.drive.setDefaultCommand(new DriveCommand(negabot.drive, g1));
        negabot.shooter.setDefaultCommand(new ShooterStandBy(negabot.shooter, negabot.drive));

        negabot.Action(g2,
                GamepadKeys.Button.DPAD_DOWN,
                new AutoIntake(negabot.intake, negabot.wait).accept(),
                null
        );

        negabot.Action(g2,
                GamepadKeys.Button.DPAD_UP,
                new AutoIntake(negabot.intake, negabot.wait).reject(),
                null
        );

        negabot.Action(g2,
                GamepadKeys.Button.DPAD_LEFT,
                new AutoIntake(negabot.intake, negabot.wait).finish(),
                null
        );

        negabot.Action(g2,
                GamepadKeys.Button.A,
                new AutoAim(negabot.drive,
                        negabot.shooter,
                        negabot.intake,
                        negabot.wait
                ),
                null
        );

        negabot.Action(
                g2,
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
