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

    private static final double RPM_TOLERANCE = 25; //TODO: change if needed
    private static final double FEED_TIME_S = 1;

    private static final double BUMP_NEAR = 0.02;
    private static final double BUMP_FAR  = 0.07;

    //linear interp vals
    private static final double MIN_DIST = 20;
    private static final double MAX_DIST = 150;
    public static final double MIN_V = 2820;
    public static final double MAX_V = 4300;

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
                    shooter.setHoodPos(setHood(d));

                }, shooter),

                new CommandBase() {
                    {
                        addRequirements(shooter);
                    }

                    @Override
                    public void execute() {
                        double d = calculateDistanceIn(drive);
                        spinUpRPM = clamp(calculateRpm(d), MIN_V, MAX_V);

                        shooter.setHoodPos(setHood(d));
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

                                double hood = clamp(setHood(d) - calculateCoverIncrease(d), 0.0, 0.5);
                                shooter.setHoodPos(hood);
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

        return MIN_V + (MAX_V - MIN_V) * (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);


    }

    private double setHood(double distanceIn) {
        double hoodNear = 0.2;
        double hoodFar  = 0.01;

        if (distanceIn > 75) return hoodFar;

        double t = (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        return hoodNear + t * (hoodFar - hoodNear);
    }

    private double calculateCoverIncrease(double distanceIn) {
        double t = (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        if (distanceIn>120)
        {
            return 0.09;
        }
        return BUMP_NEAR + t * (BUMP_FAR - BUMP_NEAR);
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
