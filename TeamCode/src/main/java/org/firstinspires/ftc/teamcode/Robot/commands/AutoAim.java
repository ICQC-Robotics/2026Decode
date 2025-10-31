package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoAim extends SequentialCommandGroup {

    enum Positions {
        OPEN_COVER(0.03),
        CLOSED_COVER(0.24);
        private final double pos;
        Positions(double pos) {
            this.pos = pos;
        }
        public double getPos() {
            return pos;
        }
    }

    private final int targetTagId;

    public AutoAim(Vision vision, Shooter shooter, Intake intake, Drive drive, Wait wait, boolean isBlueAlliance) {
        targetTagId = isBlueAlliance ? 20 : 24;
        addCommands(
                new ParallelCommandGroup(
                    // new AutoIntake(intake, shooter).accept(),
                     AimCommand(vision, shooter, drive, wait),
                     new SequentialCommandGroup(
                          ShootCommand(vision, shooter, intake, wait)
                     )
                )
        );
    }

    private SequentialCommandGroup AimCommand(Vision vision, Shooter shooter, Drive drive, Wait wait) {
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
                }
        );
    }


    public Command openShooterCover (Shooter shooter) {
        return new InstantCommand(() -> {
            shooter.setMagazineCover(Positions.OPEN_COVER.getPos());
        });
    }
    public Command closeShooterCover (Shooter shooter) {
        return new InstantCommand(() -> {
            shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
        });
    }
    private ParallelCommandGroup ShootCommand(Vision vision, Shooter shooter, Intake intake, Wait wait) {
        return new ParallelCommandGroup(
            new InstantCommand(() -> {
                shoot(vision, shooter);
            }),
            new SequentialCommandGroup(

                openShooterCover(shooter),
                new AutoIntake(intake, shooter).stopShoot(wait),
                new WaitCommand(wait, 2),
                new AutoIntake(intake, shooter).accept(),
                new WaitCommand(wait, 0.12),
                new AutoIntake(intake, shooter).stopShoot(wait),
                new WaitCommand(wait, 1),
                new AutoIntake(intake, shooter).accept(),
                new WaitCommand(wait, 0.15),
                new AutoIntake(intake, shooter).stopShoot(wait),
                new WaitCommand(wait, 1),
                new AutoIntake(intake, shooter).accept(),
                new WaitCommand(wait, 0.65),
                new AutoIntake(intake, shooter).stopShoot(wait),
                    closeShooterCover(shooter)

            )
        );
    }

    private void shoot(Vision vision, Shooter shooter) {
        double distance = Math.hypot(vision.getBotX(), vision.getBotY());
        double minD = 30, maxD = 128;
        double minV = 1900, maxV = 2850;
        //128in at 2850rpm
        //30 at 1900rpm


        //velocity = minV + (maxV-minV)*(d-minD)/(maxD-minD)

        double velocity = (minV + (maxV - minV)) * ((distance - minD)/maxD-minD);



        shooter.setPIDF(0.05, 0.0, 0, 0.60 * (velocity / 3000.0));
        shooter.setVelocity(velocity);
    }

}
