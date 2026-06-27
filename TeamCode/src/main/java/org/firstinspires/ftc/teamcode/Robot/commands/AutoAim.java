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
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoAim extends SequentialCommandGroup {

    public enum Positions {
        OPEN_COVER(.75),
        CLOSED_COVER(.486);

        private final double pos;

        Positions(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }
    }

    private static final double RPM_TOLERANCE = 20; // TODO: change if needed
    private static final double TURRET_TOLERANCE_DEG = 2.0;
    // Safety cap so a noisy pose or an unreachable turret target can't hang the feed step forever.
    private static final long AIM_TIMEOUT_MS = 500;
    private static final double FEED_TIME_S = .55;
    public static final double MIN_DIST = 20;
    public static final double MAX_DIST = 160;

    public static final double MIN_V = 2820;
    public static final double MAX_V = 4300 - 100;

    private static class Profile {
        final String name;
        final double hood;
        final double minDistanceIn;
        final double maxDistanceIn;
        final double[][] rpmLut;

        Profile(String name, double hood, double minDistanceIn, double maxDistanceIn, double[][] rpmLut) {
            this.name = name;
            this.hood = hood;
            this.minDistanceIn = minDistanceIn;
            this.maxDistanceIn = maxDistanceIn;
            this.rpmLut = rpmLut;
        }

    }

    private static final Profile[] PROFILES = new Profile[] {
            new Profile("HOOD_0_20", 0.2, 35.0, 52.0, new double[][] {
                    {40.0, 2600.0-50},
                    {50.0, 2800.0-50}
            }),
            new Profile("HOOD_0_40", 0.4, 50.0, 70.0, new double[][] {
                    {54, 2900.0-50},
                    {62.5, 2800.0-50},
                    {67.0, 2850.0-50}
            }),
            new Profile("HOOD_0_70", 0.7, 70.0, 165.0, new double[][] {
                    {71, 3140-50},
                    {73, 3150-50},
                    {98, 3500.0-50},
                    {118.0, 3675.0-150},
                    {133.0, 3900.0-140},
                    {140.0, 3990.0-50},
                    {152.0, 4070.0-60},
                    {160.0, 4200.0-30}
            })
    };

    private double spinUpRPM = MIN_V;

    public AutoAim(Drive drive, Shooter shooter, Intake intake, Wait wait, Turret turret) {
        addCommands(
                shootSequence(drive, shooter, intake, wait, turret)
        );
        addRequirements(shooter, intake);
    }

    private SequentialCommandGroup shootSequence(Drive drive, Shooter shooter, Intake intake, Wait wait, Turret turret) {
        return new SequentialCommandGroup(

                new InstantCommand(() -> {
                    double d = calculateDistanceIn(drive);
                    shooter.setMagazineCover(Positions.OPEN_COVER.getPos());
                    shooter.setHood(getHoodForDistance(d));
                }, shooter),

                new ParallelCommandGroup(

                        new CommandBase() {
                            {
                                addRequirements(shooter);
                            }

                            @Override
                            public void execute() {
                                double d = calculateDistanceIn(drive);
                                spinUpRPM = clamp(getRpmForDistance(d), MIN_V, MAX_V);
                                shooter.setVelocity(spinUpRPM);
                            }

                            @Override
                            public boolean isFinished() {
                                return Math.abs(shooter.getVelocity() - spinUpRPM) <= RPM_TOLERANCE;
                            }
                        },

                        new CommandBase() {
                            private long startMs;

                            @Override
                            public void initialize() {
                                startMs = System.currentTimeMillis();
                            }

                            @Override
                            public boolean isFinished() {
                                return turret.atTarget(TURRET_TOLERANCE_DEG)
                                        || System.currentTimeMillis() - startMs >= AIM_TIMEOUT_MS;
                            }
                        },

                        new WaitCommand(wait, 0.1)
                ),

                new InstantCommand(() -> intake.setSpeed(-1), intake),

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
                        double rpm = clamp(getRpmForDistance(d), MIN_V, MAX_V);
                        shooter.setVelocity(rpm);
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

                new InstantCommand(() -> {
                    shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
                    intake.setSpeed(0);
                }, shooter, intake)
        );
    }

    public static double calculateDistanceIn(Drive drive) {
        Pose robot = drive.follower.getPose();
        if (robot == null) return MIN_DIST;

        double goalX, goalY;
        if (Robot.ALLIANCE == Robot.Alliance.BLUE) {
            goalX = FieldConstants.BLUE_GOAL_X;
            goalY = FieldConstants.BLUE_GOAL_Y;
        } else {
            goalX = FieldConstants.RED_GOAL_X;
            goalY = FieldConstants.RED_GOAL_Y;
        }

        // robot pose (center)
        double x = robot.getX();
        double y = robot.getY();

        // IMPORTANT: heading should be in RADIANS if you're using Math.cos/sin
        // Pose heading is typically radians in pedro (double-check, but usually yes)
        double headingRad = robot.getHeading();

        // shift center -> turret pivot (forward along heading)
        double turretX = x + PPTracking.TURRET_FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = y + PPTracking.TURRET_FORWARD_OFFSET_IN * Math.sin(headingRad);

        double dx = goalX - turretX;
        double dy = goalY - turretY;

        return Math.hypot(dx, dy);
    }


    /**
     * Distance -> RPM mapping. Hood is selected as a fixed profile first,
     * then RPM is linearly interpolated only inside that profile's velocity table.
     */
    public static double getRpmForDistance(double distanceIn) {
        Profile profile = getProfileForDistance(distanceIn);
        return lookupInterpolated(clamp(distanceIn, profile.minDistanceIn, profile.maxDistanceIn), profile.rpmLut);
    }

    /**
     * Distance -> hood mapping. This is intentionally NOT interpolated.
     */
    public static double getHoodForDistance(double distanceIn) {
        return getProfileForDistance(distanceIn).hood;
    }

    private static Profile getProfileForDistance(double distanceIn) {
        for (int i = 0; i < PROFILES.length; i++) {
            Profile profile = PROFILES[i];
            boolean isLastProfile = i == PROFILES.length - 1;

            if (distanceIn >= profile.minDistanceIn
                    && (distanceIn < profile.maxDistanceIn
                    || (isLastProfile && distanceIn <= profile.maxDistanceIn))) {
                return profile;
            }
        }

        if (distanceIn < PROFILES[0].minDistanceIn) {
            return PROFILES[0];
        }

        return PROFILES[PROFILES.length - 1];
    }

    /**
     * Generic table lookup with linear interpolation.
     * Table format: { {x0, y0}, {x1, y1}, ... } where x is ascending.
     */
    private static double lookupInterpolated(double x, double[][] table) {
        if (table == null || table.length == 0) return 0.0;
        if (table.length == 1) return table[0][1];

        // Clamp to endpoints
        if (x <= table[0][0]) return table[0][1];
        int last = table.length - 1;
        if (x >= table[last][0]) return table[last][1];

        // Find segment
        for (int i = 0; i < last; i++) {
            double x0 = table[i][0];
            double y0 = table[i][1];
            double x1 = table[i + 1][0];
            double y1 = table[i + 1][1];

            if (x >= x0 && x <= x1) {
                double span = (x1 - x0);
                if (span <= 1e-9) return y0; // avoid divide-by-zero if bad table data
                double t = (x - x0) / span;
                return y0 + t * (y1 - y0);
            }
        }

        // Should never hit if table is sorted, but safe fallback
        return table[last][1];
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
