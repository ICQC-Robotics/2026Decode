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
import org.firstinspires.ftc.teamcode.Robot.commands.AutoTracking;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;

@TeleOp(group=".")
public class Solo extends CommandOpMode {
    GamepadEx g;
    Robot negabot;
    private boolean shooterStandby = false;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0));
        negabot.drive.setDefaultCommand(new DriveCommand(negabot.drive, g));
        negabot.turret.setDefaultCommand(new AutoTracking(negabot.turret, negabot.vision));

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
                       new AutoAim(negabot.vision,
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


    }
    public void run() {
        if (!shooterStandby && opModeIsActive()) {
            negabot.shooter.setDefaultCommand(new ShooterStandBy(negabot.shooter));
            shooterStandby = true;
        }
        negabot.run();
    }
}



