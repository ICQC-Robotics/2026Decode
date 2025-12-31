package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
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

    private static final double LIMELIGHT_HEIGHT_IN = 12;
    private static final double APRILTAG_HEIGHT_IN = 29.5;
    private static final double LIMELIGHT_PITCH_DEG = 12;

    //change if needed
    private static final double RPM_TOLERANCE = 150;
    private static final double WAIT_S = 2;

    private static final double BUMP_NEAR = 0.02;
    private static final double BUMP_FAR  = 0.07;

    //linear interp vals
    private static final double MIN_DIST = 36;
    private static final double MAX_DIST = 130;
    public static final double MIN_V = 3050;
    public static final double MAX_V = 4300;

    private double lastValidDistanceIn = Double.NaN;
    private double desiredRPM = 0;

    public AutoAim(Vision vision, Shooter shooter, Intake intake, Wait wait) {
        addCommands(
                shootSequence(vision, shooter, intake, wait)
        );
        addRequirements(shooter, intake);
    }

    private SequentialCommandGroup shootSequence(Vision vision, Shooter shooter, Intake intake, Wait wait) {
        return new SequentialCommandGroup(

                new InstantCommand(() -> {
                    shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());

                    double d = calculateDistanceIn(vision);
                    if (!Double.isNaN(d)) {
                        lastValidDistanceIn = d;
                        shooter.setHoodPos(setHood(d));
                    }
                }, shooter),

                new CommandBase() {
                    {
                        addRequirements(shooter);
                    }

                    @Override
                    public void execute() {
                        desiredRPM = updateRPM(vision, shooter);
                        shooter.setVelocity(desiredRPM);
                    }

                    @Override
                    public boolean isFinished() {
                        if (Double.isNaN(lastValidDistanceIn) || desiredRPM <= 0) {
                            return false;
                        }
                        return Math.abs(shooter.getVelocity() - desiredRPM) <= RPM_TOLERANCE;
                    }
                },

                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setMagazineCover(Positions.OPEN_COVER.getPos()), shooter),

                        new InstantCommand(() -> {
                            if (Double.isNaN(lastValidDistanceIn)) return;

                            double hoodPosF = shooter.hood.getPosition()
                                    + calculateCoverIncrease(lastValidDistanceIn);

                            if (hoodPosF > 1.0) hoodPosF = 1.0;
                            if (hoodPosF < 0.0) hoodPosF = 0.0;

                            shooter.setHoodPos(hoodPosF);
                        }, shooter),

                        new InstantCommand(() -> intake.setSpeed(-0.75), intake),

                        new CommandBase() {
                            {
                                addRequirements(shooter);
                            }

                            @Override
                            public void initialize() {
                                wait.start();
                            }

                            @Override
                            public void execute() {
                                double desiredRPM = updateRPM(vision, shooter);
                                shooter.setVelocity(desiredRPM);
                            }

                            @Override
                            public boolean isFinished() {
                                return wait.elapsed() >= WAIT_S;
                            }
                        },

                        new InstantCommand(() -> {
                            shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
                            intake.setSpeed(0);
                            lastValidDistanceIn = Double.NaN;
                        }, shooter, intake)
                )
        );
    }

    private double updateRPM(Vision vision, Shooter shooter) {
        double distanceIn = calculateDistanceIn(vision);
        if (!Double.isNaN(distanceIn)) {
            lastValidDistanceIn = distanceIn;
        }
        if (Double.isNaN(lastValidDistanceIn)) {
            return 0;
        }
        shooter.setHoodPos(setHood(lastValidDistanceIn));
        return clamp(calculateRpm(lastValidDistanceIn), MIN_V, MAX_V);
    }


    // distance = (aprilTagHeight - llHeight) / tan(llPitch + ty)
    public double calculateDistanceIn(Vision vision) {
        double ty = vision.getTy();
        if (Double.isNaN(ty)) return Double.NaN;

        double angleDeg = LIMELIGHT_PITCH_DEG + ty;
        double angleRad = Math.toRadians(angleDeg);

        double dh = APRILTAG_HEIGHT_IN - LIMELIGHT_HEIGHT_IN;
        double tan = Math.tan(angleRad);

        if (Math.abs(tan) < 1e-6) return Double.NaN;

        double d = dh / tan;
        if (d < 0) return Double.NaN;
        return d;
    }

    // linear interp
    public double calculateRpm(double distanceIn) {
        if (distanceIn < MIN_DIST) distanceIn = MIN_DIST;
        if (distanceIn > MAX_DIST) distanceIn = MAX_DIST;
        return MIN_V + (MAX_V - MIN_V) * (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);
    }

    private double setHood(double distanceIn) {
        double hoodNear = 0.75;
        double hoodFar  = 0.25;

        double t = (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        return hoodNear + t * (hoodFar - hoodNear);
    }

    private double calculateCoverIncrease(double distanceIn) {
        double t = (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        return BUMP_NEAR + t * (BUMP_FAR - BUMP_NEAR);
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
