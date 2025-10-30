package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoAim extends SequentialCommandGroup {

    enum Positions {
        OPEN_COVER(0),
        CLOSED_COVER(1);
        private final double pos;
        Positions(double pos) {
            this.pos = pos;
        }
        public double getPos() {
            return pos;
        }
    }

    private final int targetTagId;

    public AutoAim(Vision vision,
                   Shooter shooter,
                   Intake intake,
                   Drive drive,
                   Wait wait,
                   boolean isBlueAlliance) {
        targetTagId = isBlueAlliance ? 20 : 24;

        addCommands(
                new ParallelCommandGroup(
                        new AutoIntake(intake, shooter).accept(),
                        shoot(vision, shooter, drive, wait)
                )
        );
    }

    private SequentialCommandGroup shoot(Vision vision, Shooter shooter, Drive drive, Wait wait) {
        return new SequentialCommandGroup(
                new CommandBase() {
                    private final double kP = 0.02, MIN_TURN_POWER = 0.05, MAX_TURN_POWER = 0.4, TX_TOLERANCE_DEG = 1.0;
                    {addRequirements(drive);}

                    @Override
                    public void initialize() {
                        drive.stop();
                    }

                    @Override
                    public void execute() {
                        double tx = vision.getTx();
                        if (Double.isNaN(tx)) {
                            drive.stop();
                            return;
                        }

                        double turnPower = kP * tx;

                        if (Math.abs(turnPower) < MIN_TURN_POWER)
                            turnPower = Math.signum(turnPower) * MIN_TURN_POWER;

                        if (turnPower > MAX_TURN_POWER) turnPower = MAX_TURN_POWER;
                        if (turnPower < -MAX_TURN_POWER) turnPower = -MAX_TURN_POWER;

                        if (Math.abs(tx) <= TX_TOLERANCE_DEG)
                            drive.stop();
                        else
                            drive.turnInPlace(turnPower);
                    }

                    @Override
                    public boolean isFinished() {
                        double tx = vision.getTx();
                        if (Double.isNaN(tx)) return true;
                        return Math.abs(tx) <= TX_TOLERANCE_DEG;
                    }

                    public void end(boolean interrupted) {
                        drive.stop();
                    }

                },

                new CommandBase() {
                    @Override
                    public void initialize() {
                        wait.start();
                    }

                    @Override
                    public boolean isFinished() {
                        return wait.elapsed() >= 1;
                    }
                },

                new InstantCommand(() ->
                        shooter.setMagazineCover(Positions.OPEN_COVER.getPos())
                ),

                new InstantCommand(() -> {
                    double distance = Math.hypot(vision.getBotX(), vision.getBotY());
                    double minV = 2300;
                    double maxV = 1900;

                    double velocity = minV + (maxV - minV) * (distance - 30) / (70 - 30);
                    shooter.setVelocity(velocity);
                }),

                new InstantCommand(() ->
                        shooter.setMagazineCover(Positions.CLOSED_COVER.getPos())
                )
        );
    }
}
