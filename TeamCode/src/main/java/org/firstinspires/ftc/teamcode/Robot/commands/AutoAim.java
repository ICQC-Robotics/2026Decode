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

    private static final double RPM_TOLERANCE = 50;
    private static final double FEED_TIME_S = 1;

<<<<<<< Updated upstream
    //linear interp vals
    private static final double MIN_DIST = 30;
    private static final double MAX_DIST = 130;
    public static final double MIN_V = 3350;
    public static final double MAX_V = 4050;
=======
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

            {  63, 3150 },
            {  96, 3500 },
            {  110, 3890 },
            {  125, 3930 },
            {  130, 4110 },
            {  140, 4180 },
            {  145, 4200 },

    };
    private double fromEQ(double d){
        return d * d * -.0160561 + d * 16.78274 + 2137.60691;
    }

    // Hood LUT used only for d <= 75 (to preserve your original piecewise behavior)


    // Cover-bump LUT used only for d <= 120 (to preserve your original cap at > 120)

>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
                        spinUpRPM = clamp(calculateRpm(d), MIN_V, MAX_V);
=======
                        //TODO: spinUpRPM = clamp(getRpmForDistance(d), MIN_V, MAX_V);
>>>>>>> Stashed changes

                        spinUpRPM = clamp(fromEQ(d), MIN_V, MAX_V);
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
                                addRequirements(shooter,intake);
                            }

                            @Override
                            public void initialize() {
                                wait.start();
                            }

                            @Override
                            public void execute() {
                                double d = calculateDistanceIn(drive);
                                double rpm = calculateRpm(d);

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
        double goalX = (Robot.ALLIANCE == Robot.Alliance.BLUE) ? FieldConstants.BLUE_GOAL_X : FieldConstants.RED_GOAL_X;
        double goalY = (Robot.ALLIANCE == Robot.Alliance.BLUE) ? FieldConstants.BLUE_GOAL_Y : FieldConstants.RED_GOAL_Y;

        double dx = goalX - robot.getX();
        double dy = goalY - robot.getY();

        return Math.hypot(dx, dy);
    }

    // linear interp
    public double calculateRpm(double distanceIn) {
        if (distanceIn < MIN_DIST) distanceIn = MIN_DIST;
        if (distanceIn > MAX_DIST) distanceIn = MAX_DIST;
        if (distanceIn > 120) {
            return 5400;
        }
        return MIN_V + (MAX_V - MIN_V) * (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);


    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
