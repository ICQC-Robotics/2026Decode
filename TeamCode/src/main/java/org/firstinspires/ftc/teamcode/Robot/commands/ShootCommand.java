package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

/**
 * One-shot shoot command.
 *
 * Only requires {@code intake} so the drive, turret-tracking default command
 * (PPTracking), and shooter standby (ShooterStandBy) all continue running
 * uninterrupted while this command is active. The driver can still drive.
 *
 * Sequence:
 *   1. Open magazine cover.
 *   2. Wait until the flywheel is at target velocity AND the turret is on target
 *      (minimum 100 ms for the cover servo to physically open).
 *   3. Feed the ring for FEED_TIME_S seconds.
 *   4. Close the cover and stop the intake.
 */
public class ShootCommand extends SequentialCommandGroup {

    private static final double RPM_TOLERANCE       = 75.0;
    private static final double TURRET_TOLERANCE_DEG = 2.0;
    private static final double MIN_COVER_OPEN_S    = 0.10;
    private static final double FEED_TIME_S         = 0.60;

    /** Use this when a turret is present (turret must also be at target to fire). */
    public ShootCommand(Shooter shooter, Turret turret, Intake intake) {
        addCommands(
                // Open the cover so the ring has a path out
                new InstantCommand(() -> {
                        shooter.setRefineActive(true);   // standby switches to exact distance
                        shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                }),

                // Gate: wait for flywheel at speed + turret on target + cover open time
                new CommandBase() {
                    private final ElapsedTime coverTimer = new ElapsedTime();

                    { addRequirements(intake); }

                    @Override
                    public void initialize() {
                        coverTimer.reset();
                    }

                    @Override
                    public boolean isFinished() {
                        return coverTimer.seconds() >= MIN_COVER_OPEN_S
                                && shooter.isAtTargetVelocity(RPM_TOLERANCE)
                                && turret.atTarget(TURRET_TOLERANCE_DEG);
                    }
                },

                // Feed the ring
                new CommandBase() {
                    private final ElapsedTime feedTimer = new ElapsedTime();

                    { addRequirements(intake); }

                    @Override
                    public void initialize() {
                        intake.setSpeed(-1.0);
                        feedTimer.reset();
                    }

                    @Override
                    public boolean isFinished() {
                        return feedTimer.seconds() >= FEED_TIME_S;
                    }

                    @Override
                    public void end(boolean interrupted) {
                        intake.setSpeed(0);
                    }
                },

                // Close the cover and ensure intake is stopped
                new InstantCommand(() -> {
                    shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                    intake.setSpeed(0);
                    shooter.setRefineActive(false);   // back to pre-spin preset
                })
        );
        // The group's only hard requirement is intake — drive/shooter/turret keep running.
        addRequirements(intake);
    }

    /** Use this when there is no turret (fires as soon as flywheel is at speed). */
    public ShootCommand(Shooter shooter, Intake intake) {
        addCommands(
                new InstantCommand(() -> {
                        shooter.setRefineActive(true);   // standby switches to exact distance
                        shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                }),

                new CommandBase() {
                    private final ElapsedTime coverTimer = new ElapsedTime();

                    { addRequirements(intake); }

                    @Override
                    public void initialize() {
                        coverTimer.reset();
                    }

                    @Override
                    public boolean isFinished() {
                        return coverTimer.seconds() >= MIN_COVER_OPEN_S
                                && shooter.isAtTargetVelocity(RPM_TOLERANCE);
                    }
                },

                new CommandBase() {
                    private final ElapsedTime feedTimer = new ElapsedTime();

                    { addRequirements(intake); }

                    @Override
                    public void initialize() {
                        intake.setSpeed(-1.0);
                        feedTimer.reset();
                    }

                    @Override
                    public boolean isFinished() {
                        return feedTimer.seconds() >= FEED_TIME_S;
                    }

                    @Override
                    public void end(boolean interrupted) {
                        intake.setSpeed(0);
                    }
                },

                new InstantCommand(() -> {
                    shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                    intake.setSpeed(0);
                    shooter.setRefineActive(false);   // back to pre-spin preset
                })
        );
        addRequirements(intake);
    }
}
