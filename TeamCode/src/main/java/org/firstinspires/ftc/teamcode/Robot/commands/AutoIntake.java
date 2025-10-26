package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import com.arcrobotics.ftclib.command.InstantCommand;

public class AutoIntake extends CommandBase {
    enum Positions {
        LOWER_INTAKE(.47),
        UPPER_INTAKE(.7);

        private final double pos;

        Positions(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }
    }

    Intake intake;

    public AutoIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public Command accept() {
        return new InstantCommand(() -> {
            intake.set(Positions.LOWER_INTAKE.getPos());
            intake.setSpeed(-1);
        });
    }

    public Command reject() {
        return new InstantCommand(() -> {
            intake.set(Positions.LOWER_INTAKE.getPos());
            intake.setSpeed(1);
        });
    }

    public Command finish() {
        return new InstantCommand(() -> {
            intake.setSpeed(-0.5);
            intake.set(Positions.LOWER_INTAKE.getPos());
        });
    }
}
