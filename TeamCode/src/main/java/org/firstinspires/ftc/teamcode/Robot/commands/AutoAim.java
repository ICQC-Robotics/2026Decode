package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mode.Auto;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoAim extends SequentialCommandGroup {
    private static final double LIMELIGHT_HEIGHT_IN = 12.0;
    private static final double APRILTAG_HEIGHT_IN = 29.5;
    private static final double LIMELIGHT_PITCH_DEG = 12.0;

    private static final double COVER_OPEN_POS = 0.10;
    private static final double COVER_CLOSED_POS = 0.90;

    private static final double RPM_TOLERANCE = 150.0;
    private static final double SHOOT_WAIT_S = 2.0;

    //TODO: find
    private static final double A = 0.0;
    private static final double B = 0.0;
    private static final double C = 1945;

    //TODO: find
    private static final double MIN_RPM = 1945;
    private static final double MAX_RPM = 2710;

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
                        shooter.setMagazineCover(COVER_CLOSED_POS);
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
                        new InstantCommand(() -> intake.setSpeed(-.75), intake),

                        new InstantCommand(() -> shooter.setMagazineCover(COVER_OPEN_POS), shooter),

                        new WaitCommand(wait, SHOOT_WAIT_S),

                        new InstantCommand(() -> {
                            shooter.setVelocity(0);
                            shooter.setMagazineCover(COVER_CLOSED_POS);
                            intake.setSpeed(0); // or intake.finish(), if you have it
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
        double minDist = 30;
        double maxDist = 160;

        double hoodNear = 0.25;
        double hoodFar = 0.75;

        double t = (distanceIn - minDist) / (maxDist - minDist);
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        return hoodNear + t * (hoodFar - hoodNear);
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
