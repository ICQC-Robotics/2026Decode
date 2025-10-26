package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.SetBlueAlliance;
import org.firstinspires.ftc.teamcode.Robot.commands.SetRedAlliance;

@TeleOp(group=".")
public class Duo extends CommandOpMode {
    GamepadEx g1;
    GamepadEx g2;
    Robot negabot;

    @Override
    public void initialize() {
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        negabot = new Robot(hardwareMap, g1, g2, true);

        negabot.drive.setDefaultCommand(new DriveCommand(negabot.drive, g1));

        negabot.Action(g1,
                GamepadKeys.Button.X,
                new SetBlueAlliance(negabot),
                null);

        negabot.Action(g1,
                GamepadKeys.Button.B,
                new SetRedAlliance(negabot),
                null
        );

        negabot.Action(g2,
                GamepadKeys.Button.DPAD_DOWN,
                new AutoIntake(negabot.intake, negabot.shooter).accept(),
                null
        );

        negabot.Action(g2,
                GamepadKeys.Button.DPAD_UP,
                new AutoIntake(negabot.intake, negabot.shooter).reject(),
                null
        );

        negabot.Action(g2,
                GamepadKeys.Button.DPAD_LEFT,
                new AutoIntake(negabot.intake, negabot.shooter).finish(),
                null
        );

        negabot.Action(g2,
                GamepadKeys.Button.A,
                new AutoAim(negabot.vision,
                        negabot.shooter,
                        negabot.drive,
                        negabot.wait,
                        negabot.isBlueAlliance()
                ),
                null
        );
    }
}
