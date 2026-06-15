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

    private static final double RPM_TOLERANCE = 20; // TODO: change if needed
    private static final double FEED_TIME_S = .6;
    public static final double MIN_DIST = 20;
    public static final double MAX_DIST = 150;

    public static final double MIN_V = 2820;
    public static final double MAX_V = 4300 - 100;

    private static final double[][] RPM_LUT = new double[][] {

            {  49, 3700 },
            {  65, 3150 },
            {  78, 3350 },
            {  95, 3550 },
            {  128, 3980 },
            {  135, 4050 },
            {  139, 4090 },
            {  144, 4170 },


    };

    private static final double[][] HOOD_LUT = new double[][] {
            {  49, 0.20 },
            {  65, 0.40 },
            {  80, 0.70 },
            { 144, 0.70 },
    };

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
     * Distance -> RPM mapping using lookup table + linear interpolation between points.
     */
    public static double getRpmForDistance(double distanceIn) {
        return lookupInterpolated(clamp(distanceIn, MIN_DIST, MAX_DIST), RPM_LUT);
    }
    public static double getHoodForDistance(double distanceIn) {
        return lookupInterpolated(clamp(distanceIn, MIN_DIST, MAX_DIST), HOOD_LUT);
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
