package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;

@TeleOp(group=".")
public class Solo extends CommandOpMode {
    GamepadEx g;
    Robot goonbot;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        goonbot = new Robot(hardwareMap, g, null);
        goonbot.drive.setDefaultCommand(new DriveCommand(goonbot.drive, g));

        goonbot.Action(g,
                       GamepadKeys.Button.RIGHT_BUMPER,
                       new AutoIntake(goonbot.intake, goonbot.wait).accept(),
                       new AutoIntake(goonbot.intake, goonbot.wait).finish()
        );

        goonbot.Action(g,
                GamepadKeys.Button.LEFT_BUMPER,
                new AutoIntake(goonbot.intake, goonbot.wait).reject(),
                new AutoIntake(goonbot.intake, goonbot.wait).finish()
        );

        goonbot.Action(g,
                       GamepadKeys.Button.A,
                       new AutoAim(goonbot.vision,
                                   goonbot.shooter,
                                   goonbot.intake,
                                   goonbot.drive,
                                   goonbot.wait
                       ),
                       null
        );

        goonbot.Action(
                g,
                GamepadKeys.Button.Y,
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                }),
                null
        );
    }

    public void run() {
        super.run();
        }
}



