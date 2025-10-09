package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.SetBlueAlliance;
import org.firstinspires.ftc.teamcode.Robot.commands.SetRedAlliance;

public class SoloBlue extends OpMode {
    GamepadEx g1 = new GamepadEx(gamepad1);
    Robot negabot = new Robot(hardwareMap, g1, null, true);

    @Override
    public void init() {
        negabot.Action(g1,
                GamepadKeys.Button.X,
                new SetBlueAlliance(negabot),
                null);

        negabot.Action(g1,
                GamepadKeys.Button.B,
                new SetRedAlliance(negabot),
                null
        );
    }

    @Override
    public void loop() {

    }
}

