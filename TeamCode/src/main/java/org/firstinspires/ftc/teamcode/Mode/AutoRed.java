package org.firstinspires.ftc.teamcode.Mode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(group=".")
public class AutoRed extends OpMode {
    Robot negabot;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, null, null, false);
    }

    @Override
    public void loop() {

    }
}
