package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.ShooterAimingModel;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoAim extends SequentialCommandGroup {

    public enum Positions {
        OPEN_COVER(.75),
        CLOSED_COVER(.5);

        private final double pos;

        Positions(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }
    }

    private static final double RPM_TOLERANCE = 50;
    private static final double FEED_TIME_S = .6;

    private static final double BUMP_NEAR = 0.02;
    private static final double BUMP_FAR  = 0.07;

    /**
     * Distance window we care about for lookup/limiting.
     * These are public so other commands (e.g. ShooterStandBy) can reuse them.
     */
    public static final double MIN_DIST = 20;
    public static final double MAX_DIST = 150;

    /** Velocity bounds (rpm). Keep these as the single source of truth. */
    public static final double MIN_V = 2500;
    public static final double MAX_V = 4100;

    private double spinUpRPM = MIN_V;
    private ShooterAimingModel.Solution solution;

    public AutoAim(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        addCommands(
                shootSequence(drive, shooter, intake, wait)
        );
        addRequirements(shooter, intake);
    }

    private SequentialCommandGroup shootSequence(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        return new SequentialCommandGroup(

                // Start opening the cover immediately
                new InstantCommand(() ->
                        shooter.setMagazineCover(Positions.OPEN_COVER.getPos()), shooter),

                // Wait for BOTH:
                // 1) shooter to reach velocity
                // 2) cover to have had 0.5s to open
                new ParallelCommandGroup(

                        new CommandBase() {
                            {
                                addRequirements(shooter);
                            }

                            @Override
                            public void execute() {
                                double d = calculateDistanceIn(drive);
                                solution = shooter.aimForDistance(d);
                                spinUpRPM = solution.rpm;
                            }

                            @Override
                            public boolean isFinished() {
                                return shooter.isHoodSettled()
                                        && shooter.isAtTargetVelocity(RPM_TOLERANCE);
                            }
                        },

                        new WaitCommand(wait, 0.1)
                ),

                // Only starts after BOTH parallel commands above are done
                new InstantCommand(() -> intake.setSpeed(-1), intake),

                // Feed for FEED_TIME_S while maintaining shooter RPM
                new CommandBase() {
                    {
                        addRequirements(shooter, intake);
                    }

                    @Override
                    public void initialize() {
                        wait.start();
                    }

                    @Override
                    public void execute() {
                        double d = calculateDistanceIn(drive);
                        solution = shooter.aimForDistance(d);
                        spinUpRPM = solution.rpm;
                    }

                    @Override
                    public boolean isFinished() {
                        return wait.elapsed() >= FEED_TIME_S;
                    }

                    @Override
                    public void end(boolean interrupted) {
                        intake.setSpeed(0);
                    }
                },

                // Reset state
                new InstantCommand(() -> {
                    shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
                    intake.setSpeed(0);
                }, shooter, intake)
        );
    }

    public static double calculateDistanceIn(Drive drive) {
        Pose robot = drive.follower.getPose();
        if (robot == null) return MIN_DIST;

        Pose goal = FieldConstants.goalAimPointForAlliance(Robot.ALLIANCE == Robot.Alliance.BLUE);
        double goalX = goal.getX();
        double goalY = goal.getY();

        // robot pose (center)
        double x = robot.getX();
        double y = robot.getY();

        // IMPORTANT: heading should be in RADIANS if you're using Math.cos/sin
        // Pose heading is typically radians in pedro (double-check, but usually yes)
        double headingRad = robot.getHeading();

        // shift center -> turret pivot (forward along heading)
        double turretX = x + Turret.FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = y + Turret.FORWARD_OFFSET_IN * Math.sin(headingRad);

        double dx = goalX - turretX;
        double dy = goalY - turretY;

        return Math.hypot(dx, dy);
    }


    /**
     * Distance -> RPM mapping using lookup table + linear interpolation between points.
     */
    public static double getRpmForDistance(double distanceIn) {
        return ShooterAimingModel.previewDefaultRpm(clamp(distanceIn, MIN_DIST, MAX_DIST));
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
