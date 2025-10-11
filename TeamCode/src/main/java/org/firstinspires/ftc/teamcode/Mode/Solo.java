package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.SetBlueAlliance;
import org.firstinspires.ftc.teamcode.Robot.commands.SetRedAlliance;

@TeleOp(group=".")
public class Solo extends OpMode {
    GamepadEx g = new GamepadEx(gamepad1);
    Robot negabot;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, g, null, true);

        negabot.drive.movement(g);

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
                       GamepadKeys.Button.DPAD_DOWN,
                       new AutoIntake(negabot.intake).accept(),
                       null
        );

        negabot.Action(g,
                GamepadKeys.Button.DPAD_UP,
                new AutoIntake(negabot.intake).reject(),
                null
        );

        negabot.Action(g,
                GamepadKeys.Button.DPAD_LEFT,
                new AutoIntake(negabot.intake).finish(),
                null
        );

        negabot.Action(g,
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

    @Override
    public void loop() {

    }
}

