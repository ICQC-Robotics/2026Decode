package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoAim extends SequentialCommandGroup {

    public enum Positions {
        OPEN_COVER(0.1),
        CLOSED_COVER(1);

        private final double pos;

        Positions(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }
    }

    private static final double RPM_TOLERANCE = 25; // TODO: change if needed
    private static final double FEED_TIME_S = 1;

    private static final double BUMP_NEAR = 0.02;
    private static final double BUMP_FAR  = 0.07;

    /**
     * Distance window we care about for lookup/limiting.
     * These are public so other commands (e.g. ShooterStandBy) can reuse them.
     */
    public static final double MIN_DIST = 20;
    public static final double MAX_DIST = 150;

    /** Velocity bounds (rpm). Keep these as the single source of truth. */
    public static final double MIN_V = 2820;
    public static final double MAX_V = 4300 - 100;

    /**
     * Lookup tables (distance inches -> value).
     *
     * NOTE: The current values match your old linear interpolation (so behavior is unchanged),
     * but now you can tune any point without re-deriving a formula.
     *
     * Tables MUST be sorted by distance ascending.
     */
    private static final double[][] RPM_LUT = new double[][] {

            {  49, 3700 },
            {  65, 3100 },
            {  78, 3400 },
            {  95, 3650 },
            {  128, 3960 },
            {  135, 4040 },
            {  144, 4070 },


    };

    // Hood LUT used only for d <= 75 (to preserve your original piecewise behavior)


    // Cover-bump LUT used only for d <= 120 (to preserve your original cap at > 120)


    private double spinUpRPM = MIN_V;

    public AutoAim(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        addCommands(
                shootSequence(drive, shooter, intake, wait)
        );
        addRequirements(shooter, intake);
    }

    private SequentialCommandGroup shootSequence(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        return new SequentialCommandGroup(

                new InstantCommand(() -> {
                    shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
                    double d = calculateDistanceIn(drive);
                }, shooter),

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

                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setMagazineCover(Positions.OPEN_COVER.getPos()), shooter),
                        new WaitCommand(wait, 0.5),
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
                                double rpm = getRpmForDistance(d);


                                shooter.setVelocity(rpm);
                            }

                            @Override
                            public boolean isFinished() {
                                return wait.elapsed() >= FEED_TIME_S;
                            }
                        },

                        new InstantCommand(() -> {
                            shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
                            intake.setSpeed(0);
                        }, shooter, intake)
                ));
    }

    public double calculateDistanceIn(Drive drive) {
        Pose robot = drive.follower.getPose();
        if (robot == null) return MIN_DIST;

        double goalX;
        double goalY;

        if (Robot.ALLIANCE == Robot.Alliance.BLUE) {
            goalX = FieldConstants.BLUE_GOAL_X;
            goalY = FieldConstants.BLUE_GOAL_Y;
        } else {
            goalX = FieldConstants.RED_GOAL_X;
            goalY = FieldConstants.RED_GOAL_Y;
        }

        double dx = goalX - robot.getX();
        double dy = goalY - robot.getY();

        return Math.hypot(dx, dy);
    }


    /**
     * Distance -> RPM mapping using lookup table + linear interpolation between points.
     */
    public static double getRpmForDistance(double distanceIn) {
        return lookupInterpolated(clamp(distanceIn, MIN_DIST, MAX_DIST), RPM_LUT);
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
