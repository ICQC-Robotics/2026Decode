package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoIntake extends CommandBase {
    public enum Positions {
        LOWER_INTAKE(.45),
        UPPER_INTAKE(.7);

        private final double pos;

        Positions(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }
    }

    private Intake intake;
    private Wait wait;

    public AutoIntake(Intake intake, Wait wait) {
        this.intake = intake;
        this.wait = wait;
        addRequirements(intake);
    }

    public Command raiseIntake() {
        return new InstantCommand(() -> {
            intake.set(Positions.LOWER_INTAKE.getPos());

        }, intake);
    }

    public Command accept() {
        return new InstantCommand(() -> {
            intake.set(Positions.LOWER_INTAKE.getPos());
            intake.setSpeed(-1);
        }, intake);
    }

    public Command autoAccept(double seconds) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intake.set(Positions.LOWER_INTAKE.getPos());
                    intake.setSpeed(-1);
                }, intake),
                new WaitCommand(wait, seconds),
                new InstantCommand(() -> intake.setSpeed(0), intake)
        );
    }

    public Command acceptSlow() {
        return new InstantCommand(() -> {
            intake.set(Positions.LOWER_INTAKE.getPos());
            intake.setSpeed(-0.45);
        }, intake);
    }

    public Command acceptSlowish() {
        return new InstantCommand(() -> {
            intake.set(Positions.LOWER_INTAKE.getPos());
            intake.setSpeed(-0.70);
        }, intake);
    }

    public Command reject() {
        return new InstantCommand(() -> {
            intake.set(Positions.LOWER_INTAKE.getPos());
            intake.setSpeed(1);
        }, intake);
    }

    public Command finish() {
        return new InstantCommand(() -> {
            intake.setSpeed(0);
            intake.set(Positions.LOWER_INTAKE.getPos());
        }, intake);
    }
}
