package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.ShooterAimingModel;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class ShootOnMove extends SequentialCommandGroup {
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

    private static final double RPM_TOLERANCE = 25;
    private static final double FEED_TIME_S = 1;
    public static final double MIN_DIST = 20;
    public static final double MAX_DIST = 150;
    public static final double MIN_V = AutoAim.MIN_V;
    public static final double MAX_V = AutoAim.MAX_V;

    private static final double SHOOTER_ANGLE_DEG = 40.0;
    private static final double WHEEL_RADIUS_IN = 2.835;
    private static final double SHOOTER_EFFICIENCY = 0.75;
    private static final double DRAG_COEFF = 0.65;
    private static final double MIN_FLIGHT_TIME_S = 0.05;
    private static final double MAX_FLIGHT_TIME_S = 0.70;
    private static final int LEAD_ITERATIONS = 5;

    private final Turret turret;
    private double spinUpRPM = MIN_V;

    public ShootOnMove(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        this(drive, null, shooter, intake, wait);
    }

    public ShootOnMove(Drive drive, Turret turret, Shooter shooter, Intake intake, Wait wait) {
        this.turret = turret;
        addCommands(shootSequence(drive, shooter, intake, wait));
        if (turret == null) {
            addRequirements(shooter, intake);
        } else {
            addRequirements(turret, shooter, intake);
        }
    }

    private SequentialCommandGroup shootSequence(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setMagazineCover(Positions.CLOSED_COVER.getPos()), shooter),

                new CommandBase() {
                    {
                        if (turret == null) {
                            addRequirements(shooter);
                        } else {
                            addRequirements(turret, shooter);
                        }
                    }

                    @Override
                    public void execute() {
                        MovingShot shot = calculateMovingShot(drive);
                        aimTurretForShot(shot);
                        ShooterAimingModel.Solution solution = shooter.aimForDistance(shot.distanceIn);
                        spinUpRPM = solution.rpm;
                    }

                    @Override
                    public boolean isFinished() {
                        return shooter.isHoodSettled()
                                && turretReady()
                                && Math.abs(shooter.getVelocity() - spinUpRPM) <= RPM_TOLERANCE;
                    }
                },

                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setMagazineCover(Positions.OPEN_COVER.getPos()), shooter),
                        new WaitCommand(100),
                        new InstantCommand(() -> intake.setSpeed(-1), intake),
                        new CommandBase() {
                            {
                                if (turret == null) {
                                    addRequirements(shooter, intake);
                                } else {
                                    addRequirements(turret, shooter, intake);
                                }
                            }

                            @Override
                            public void initialize() {
                                wait.start();
                            }

                            @Override
                            public void execute() {
                                MovingShot shot = calculateMovingShot(drive);
                                aimTurretForShot(shot);
                                shooter.aimForDistance(shot.distanceIn);
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
                ));
    }

    public double calculateDistanceIn(Drive drive) {
        return calculateMovingShot(drive).distanceIn;
    }

    public double calculateAngle(Drive drive) {
        return calculateMovingShot(drive).turretAngleDeg;
    }

    private MovingShot calculateMovingShot(Drive drive) {
        Pose robot = drive.follower.getPose();
        if (robot == null) return new MovingShot(MIN_DIST, 135.0, 0.0);

        Vector vel = drive.follower.getVelocity();
        double gX = (Robot.ALLIANCE == Robot.Alliance.BLUE) ? FieldConstants.BLUE_GOAL_X : FieldConstants.RED_GOAL_X;
        double gY = (Robot.ALLIANCE == Robot.Alliance.BLUE) ? FieldConstants.BLUE_GOAL_Y : FieldConstants.RED_GOAL_Y;

        double headingRad = robot.getHeading();
        double turretX = robot.getX() + PPTracking.TURRET_FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = robot.getY() + PPTracking.TURRET_FORWARD_OFFSET_IN * Math.sin(headingRad);
        double vx = (vel == null) ? 0.0 : vel.getXComponent();
        double vy = (vel == null) ? 0.0 : vel.getYComponent();

        double distance = Math.hypot(gX - turretX, gY - turretY);
        double time = getTime(distance);

        for (int i = 0; i < LEAD_ITERATIONS; i++) {
            double predictedTurretX = turretX + vx * time;
            double predictedTurretY = turretY + vy * time;
            distance = Math.hypot(gX - predictedTurretX, gY - predictedTurretY);
            time = getTime(distance);
        }

        double aimX = gX - (turretX + vx * time);
        double aimY = gY - (turretY + vy * time);
        return new MovingShot(distance, turretAngleDeg(aimX, aimY, headingRad), time);
    }

    public static double getRpmForDistance(double distanceIn) {
        return ShooterAimingModel.previewDefaultRpm(Math.max(MIN_DIST, Math.min(distanceIn, MAX_DIST)));
    }

    public static double getTime(double distanceIn) {
        double rpm = getRpmForDistance(distanceIn);
        double wheelSurfaceSpeed = (rpm / 60.0) * (2.0 * Math.PI * WHEEL_RADIUS_IN);
        double launchSpeed = wheelSurfaceSpeed * SHOOTER_EFFICIENCY * DRAG_COEFF;
        double horizontalSpeed = launchSpeed * Math.cos(Math.toRadians(SHOOTER_ANGLE_DEG));
        if (horizontalSpeed <= 1e-6) return MAX_FLIGHT_TIME_S;
        return clamp(distanceIn / horizontalSpeed, MIN_FLIGHT_TIME_S, MAX_FLIGHT_TIME_S);
    }

    private void aimTurretForShot(MovingShot shot) {
        if (turret != null) {
            turret.setTargetDeg(shot.turretAngleDeg);
        }
    }

    private boolean turretReady() {
        return turret == null || turret.atTarget(2.0);
    }

    private static double turretAngleDeg(double fieldDx, double fieldDy, double headingRad) {
        double bearingDeg = Math.toDegrees(Math.atan2(fieldDy, fieldDx));
        double headingDeg = Math.toDegrees(headingRad);
        double deflectionDeg = wrap180(bearingDeg - headingDeg);
        return wrap360(135.0 - deflectionDeg);
    }

    private static double wrap360(double a) {
        a %= 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    private static double wrap180(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static class MovingShot {
        final double distanceIn;
        final double turretAngleDeg;

        MovingShot(double distanceIn, double turretAngleDeg, double flightTimeS) {
            this.distanceIn = distanceIn;
            this.turretAngleDeg = turretAngleDeg;
        }
    }
}
