package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class SetRedAlliance extends InstantCommand {
    public SetRedAlliance(Robot robot) {
        super(() -> robot.setAlliance(false));
    }
}
