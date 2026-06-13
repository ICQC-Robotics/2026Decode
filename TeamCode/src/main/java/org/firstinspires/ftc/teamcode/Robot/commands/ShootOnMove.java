package org.firstinspires.ftc.teamcode.Robot.commands;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class ShootOnMove extends SequentialCommandGroup {
    public enum Positions {
        OPEN_COVER(AutoAim.Positions.OPEN_COVER.getPos()),
        CLOSED_COVER(AutoAim.Positions.CLOSED_COVER.getPos());

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
    public static final double MIN_DIST = 20;
    public static final double MAX_DIST = 150;
    public static final double MIN_V = AutoAim.MIN_V;
    public static final double MAX_V = AutoAim.MAX_V;

    public static double FLIGHT_TIME_S = 1.0;

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
                        addShotTelemetry(drive, shooter, shot);
                    }

                    @Override
                    public boolean isFinished() {
                        return shooter.isHoodSettled()
                                && turretReady()
                                && shooter.isAtTargetVelocity(RPM_TOLERANCE);
                    }
                },

                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setMagazineCover(Positions.OPEN_COVER.getPos()), shooter),
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
                                addShotTelemetry(drive, shooter, shot);
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
        Pose goal = FieldConstants.goalAimPointForAlliance(Robot.ALLIANCE == Robot.Alliance.BLUE);
        double gX = goal.getX();
        double gY = goal.getY();

        double headingRad = robot.getHeading();
        double turretX = robot.getX() + Turret.FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = robot.getY() + Turret.FORWARD_OFFSET_IN * Math.sin(headingRad);
        double vx = (vel == null) ? 0.0 : vel.getXComponent();
        double vy = (vel == null) ? 0.0 : vel.getYComponent();

        double time = getTime();
        double aimX = gX - (turretX + vx * time);
        double aimY = gY - (turretY + vy * time);
        double distance = Math.hypot(aimX, aimY);
        return new MovingShot(distance, turretAngleDeg(aimX, aimY, headingRad), time);
    }

    private void addShotTelemetry(Drive drive, Shooter shooter, MovingShot shot) {
        drive.telemetry.addData("SOM Distance", shot.distanceIn);
        drive.telemetry.addData("SOM Flight Time", shot.flightTimeS);
        drive.telemetry.addData("SOM Turret Target", shot.turretAngleDeg);
        drive.telemetry.addData("SOM Turret Ready", turretReady());
        drive.telemetry.addData("SOM Hood", shooter.getTargetHoodPosition());
        drive.telemetry.addData("SOM Profile", shooter.getLastProfileName());
        drive.telemetry.addData("SOM Target RPM", shooter.getTargetVelocity());
        drive.telemetry.addData("SOM Actual RPM", shooter.getVelocity());
        drive.telemetry.addData("SOM Right RPM", shooter.getRightVelocity());
        drive.telemetry.addData("SOM Left RPM", shooter.getLeftVelocity());
        drive.telemetry.addData("SOM RPM Ready", shooter.isAtTargetVelocity(RPM_TOLERANCE));
    }

    public static double getRpmForDistance(double distanceIn) {
        return ShooterAimingModel.previewDefaultRpm(Math.max(MIN_DIST, Math.min(distanceIn, MAX_DIST)));
    }

    public static double getTime() {
        return Math.max(0.0, FLIGHT_TIME_S);
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

    private static class MovingShot {
        final double distanceIn;
        final double turretAngleDeg;
        final double flightTimeS;

        MovingShot(double distanceIn, double turretAngleDeg, double flightTimeS) {
            this.distanceIn = distanceIn;
            this.turretAngleDeg = turretAngleDeg;
            this.flightTimeS = flightTimeS;
        }
    }
}
