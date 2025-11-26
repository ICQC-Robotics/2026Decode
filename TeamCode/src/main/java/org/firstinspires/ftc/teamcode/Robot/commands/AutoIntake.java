package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoIntake extends CommandBase {
    private Intake intake;
    private Wait wait;

    public AutoIntake(Intake intake, Wait wait) {
        this.intake = intake;
        this.wait = wait;
        addRequirements(intake);
    }

    public Command accept() {
        return new InstantCommand(() -> {
            intake.setSpeed(1);
        });
    }

    public Command reject() {
        return new InstantCommand(() -> {
            intake.setSpeed(-1);
        });
    }

    public Command autoAccept(double seconds) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    while (wait.elapsed() < seconds) {
                        intake.setSpeed(1);
                    }
                })
        );
    }

    public Command finish() {
        return new InstantCommand(() -> {
            intake.setSpeed(0);
        });
    }
}
