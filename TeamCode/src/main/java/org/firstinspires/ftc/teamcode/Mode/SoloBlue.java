package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class SoloBlue extends OpMode {
    Robot negabot = new Robot(hardwareMap, new GamepadEx(new Gamepad()), null, true);

    @Override
    public void init() {
    }

    @Override
    public void loop() {

    }
}

