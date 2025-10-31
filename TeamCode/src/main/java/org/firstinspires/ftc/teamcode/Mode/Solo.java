package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.SetBlueAlliance;
import org.firstinspires.ftc.teamcode.Robot.commands.SetRedAlliance;

@TeleOp(group=".")
public class Solo extends OpMode {
    GamepadEx g;
    Robot negabot;

    @Override
    public void init() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, g, null, true);

        negabot.drive.setDefaultCommand(new DriveCommand(negabot.drive, g));

        negabot.Action(g,
                GamepadKeys.Button.X,
                new SetBlueAlliance(negabot),
                null
        );

        negabot.Action(g,
                GamepadKeys.Button.B,
                new SetRedAlliance(negabot),
                null
        );


        negabot.Action(g,
                GamepadKeys.Button.RIGHT_BUMPER,
                new AutoIntake(negabot.intake, negabot.shooter).accept(),
                new AutoIntake(negabot.intake, negabot.shooter).finish()
        );

        negabot.Action(g,
                GamepadKeys.Button.LEFT_BUMPER,
                new AutoIntake(negabot.intake, negabot.shooter).reject(),
                new AutoIntake(negabot.intake, negabot.shooter).finish()
        );

        negabot.Action(g,
                GamepadKeys.Button.A,
                new AutoAim(negabot.vision,
                        negabot.shooter,
                        negabot.intake,
                        negabot.drive,
                        negabot.wait,
                        negabot.isBlueAlliance()
                ),
                null
        );
    }

    @Override
    public void loop() {
        telemetry.addData("X", negabot.vision.getBotX());
        telemetry.addData("Y", negabot.vision.getBotY());
        telemetry.update();
    }
}

