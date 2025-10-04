package org.firstinspires.ftc.teamcode.Mode.raihancode;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeCommand extends CommandBase {
    private final ActiveIntakeSubsystem intake;
    private final double power;

    public IntakeCommand(ActiveIntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.intake.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
