package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.Robot.subsystems.FieldCentricDrive;

public class DriveCommand extends CommandBase {
    private final FieldCentricDrive driveSubsystem;
    private final GamepadEx gamepad;

    public DriveCommand(FieldCentricDrive drive, GamepadEx g) {
        this.driveSubsystem = drive;
        this.gamepad = g;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.movement(gamepad);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
