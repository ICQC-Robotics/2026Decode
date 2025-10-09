package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class SetBlueAlliance extends InstantCommand {
    public SetBlueAlliance(Robot robot) {
        super(() -> robot.setAlliance(true));
    }
}
