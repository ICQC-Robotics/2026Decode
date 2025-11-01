package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class AutoAim extends SequentialCommandGroup {

    private static final double STANDBY_RPM = 1500.0;
    private static final double TX_OK_DEG = 2.0;
    private static final double TA_MIN_SEEN = 1e-4;

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
                    private final double kP = 0.03, MIN_TURN_POWER = 0.05, MAX_TURN_POWER = 0.4, TX_TOLERANCE_DEG = .5;

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

    private ParallelCommandGroup ShootCommand(Vision vision, Shooter shooter, Intake intake, Wait wait) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> {
                    shoot(vision, shooter);
                }),
                new SequentialCommandGroup(
                        new WaitCommand(wait, 4),
                        openShooterCover(shooter),
                        new WaitCommand(wait, 1),
                        new AutoIntake(intake, shooter).accept(),
                        new WaitCommand(wait, 2),
                        new AutoIntake(intake, shooter).stopShoot(wait),
                        closeShooterCover(shooter)

                )
        );
    }

    private void shoot(Vision vision, Shooter shooter) {
        double tA = vision.getTa();
        double tA_frac = (tA > 1.0) ? tA / 100.0 : tA;

        final double tA_MIN = 0.0073;
        final double tA_MAX = 0.059;
        final double V_MIN = 1600;
        final double V_MAX = 2600;
        final double GAMMA = 1.6;

        tA_frac = Math.max(tA_MIN, Math.min(tA_frac, tA_MAX));
        double u = (tA_MAX - tA_frac) / (tA_MAX - tA_MIN);
        double velocity = V_MIN + (V_MAX - V_MIN) * Math.pow(u, GAMMA);
        velocity = Math.min(velocity, V_MAX);

        shooter.setPIDF(0.095, 0.0, 0, 0.57 * (velocity / 3000));
        shooter.setVelocity(velocity);
    }
}
