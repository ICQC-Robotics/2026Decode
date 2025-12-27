package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoAim extends SequentialCommandGroup {
    public enum Positions {
        OPEN_COVER(0.03),
        CLOSED_COVER(0.27);
        private final double pos;

        Positions(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }
    }

    //vision
    double limelightHeight = 17.0;
    double aprilTagHeight = 29.5;
    private final double limelightPitch = 23.0;

    //tuned at 12.8v
    private final double minV = 1950, maxV = 2450;
    private final double minD = 30, maxD = 70;


    public AutoAim(Vision vision, Shooter shooter, Intake intake, FieldCentricDrive drive, Wait wait) {
        addCommands(
                new ParallelCommandGroup(
                        AimCommand(vision, drive, wait),
                        new SequentialCommandGroup(
                                ShootCommand(vision, shooter, intake, wait)
                        )
                )
        );
        addRequirements(drive, shooter, intake);
    }

    private SequentialCommandGroup AimCommand(Vision vision, FieldCentricDrive drive, Wait wait) {
        return new SequentialCommandGroup(
                new CommandBase() {
                    private final double kP = 0.03, MIN_TURN_POWER = 0.05, MAX_TURN_POWER = 0.4, TX_TOLERANCE_DEG = .3;

                    {
                        addRequirements(drive);
                    }

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
                }
        );
    }


    public Command openShooterCover(Shooter shooter) {
        return new InstantCommand(() -> {
            shooter.setMagazineCover(Positions.OPEN_COVER.getPos());
        });
    }

    public Command closeShooterCover(Shooter shooter) {
        return new InstantCommand(() -> {
            shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
        });
    }

    private SequentialCommandGroup ShootCommand(Vision vision, Shooter shooter, Intake intake, Wait wait) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    double v = calculateVelocity(vision);
                    if (v < 2000) v = 2000;
                    shooter.setVelocity(v);
                }, shooter),

                new CommandBase() {
                    @Override
                    public boolean isFinished() {
                        double actual = shooter.getVelocity();
                        double target = calculateVelocity(vision);
                        return Math.abs(actual - target) < 150;
                    }
                },

                new AutoIntake(intake, wait).acceptSlowish(),
                openShooterCover(shooter),
                new WaitCommand(wait, 1),
                new AutoIntake(intake, wait).finish(),
                closeShooterCover(shooter),
                new InstantCommand(() -> {
                    shooter.setVelocity(0);
                }, shooter)
        );
    }


    public double calculateVelocity(Vision vision) {
        //calc dist
        double actualHeight = aprilTagHeight - limelightHeight;
        double angle = limelightPitch + vision.getTy();
        double d = actualHeight / Math.tan(Math.toRadians(angle));
        if (d < 30) d = 30;

        return minV + (maxV - minV) * (d - minD)/(maxD-minD); //returns velocity
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
