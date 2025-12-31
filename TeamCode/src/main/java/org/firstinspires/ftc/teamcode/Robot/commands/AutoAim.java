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

    private static final double LIMELIGHT_HEIGHT_IN = 12.0;
    private static final double APRILTAG_HEIGHT_IN = 29.5;
    private static final double LIMELIGHT_PITCH_DEG = 12.0;

    private static final double RPM_TOLERANCE = 150.0;
    private static final double SHOOT_WAIT_S = 3.0;

    private static final double A = -0.00618007;
    private static final double B = 9.22787;
    private static final double C = 3100;

    private static final double MIN_RPM = 2333;
    private static final double MAX_RPM = 4700;

    //TODO: find
    private static final double BUMP_NEAR = 0.02;
    private static final double BUMP_FAR  = 0.1;

    private static final double MIN_DIST = 35;
    private static final double MAX_DIST = 135;

    public AutoAim(Vision vision, Shooter shooter, Intake intake, Wait wait) {
        addCommands(
                shootSequence(vision, shooter, intake, wait)
        );

        addRequirements(shooter, intake);
    }

    private SequentialCommandGroup shootSequence(Vision vision, Shooter shooter, Intake intake, Wait wait) {
        return new SequentialCommandGroup(

                new InstantCommand(() -> {
                    double distanceIn = calculateDistanceIn(vision);

                    if (Double.isNaN(distanceIn)) {
                        shooter.setVelocity(0);
                        shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
                        return;
                    }

                    double rpm = calculateRpm(distanceIn);
                    rpm = clamp(rpm, MIN_RPM, MAX_RPM);
                    shooter.setHoodPos(setHood(distanceIn));
                    shooter.setVelocity(rpm);
                }, shooter),

                new CommandBase() {
                    @Override
                    public boolean isFinished() {
                        double distanceIn = calculateDistanceIn(vision);
                        if (Double.isNaN(distanceIn)) return true;

                        double target = clamp(calculateRpm(distanceIn), MIN_RPM, MAX_RPM);
                        double actual = shooter.getVelocity();
                        return Math.abs(actual - target) <= RPM_TOLERANCE;
                    }
                },

                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setMagazineCover(Positions.OPEN_COVER.getPos()), shooter),



                        new WaitCommand(wait, 1),

                        new InstantCommand(() -> {
                            double distanceIn = calculateDistanceIn(vision);
                            if (Double.isNaN(distanceIn)) return;

                            double x = calculateCoverIncrease(distanceIn);
                            double current = shooter.hood.getPosition();
                            double bumped = current + x;

                            if (bumped > 1.0) bumped = 1.0;
                            if (bumped < 0.0) bumped = 0.0;

                            shooter.setHoodPos(bumped);
                        }, shooter),

                        new InstantCommand(() -> intake.setSpeed(-1), intake),
                        new WaitCommand(wait, SHOOT_WAIT_S),



                        new InstantCommand(() -> {
                            shooter.setVelocity(0);
                            shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
                            intake.setSpeed(0);
                        }, shooter, intake)
                ));
    }

    //distance = (aprilTagHeight - llHeight) / tan(llPitch + ty)
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

    public double calculateRpm(double distanceIn) {
        return (A * distanceIn * distanceIn) + (B * distanceIn) + C;
    }

    //TODO: tune these vals
    private double setHood(double distanceIn) {
        double hoodNear = .4;
        double hoodFar = .1;

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
