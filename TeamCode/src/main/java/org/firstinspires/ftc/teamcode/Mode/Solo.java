package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Solo extends OpMode {
    Robot negabot = new Robot(hardwareMap, new GamepadEx(new Gamepad()), null);

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}

