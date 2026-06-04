package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.ShooterAimingModel;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

import java.util.function.DoubleSupplier;

public class StationaryShoot extends SequentialCommandGroup {

    private static final double RPM_TOLERANCE = 25;
    private static final double TURRET_TOLERANCE_DEG = 2.0;
    private static final double FEED_TIME_S = 0.6;
    private static final double MIN_DIST = AutoAim.MIN_DIST;

    private final Turret turret;
    private final DoubleSupplier turretOffsetDeg;

    public StationaryShoot(Drive drive, Turret turret, Shooter shooter, Intake intake, Wait wait) {
        this(drive, turret, shooter, intake, wait, 0.0);
    }

    public StationaryShoot(Drive drive, Turret turret, Shooter shooter, Intake intake, Wait wait,
                           double turretOffsetDeg) {
        this(drive, turret, shooter, intake, wait, () -> turretOffsetDeg);
    }

    public StationaryShoot(Drive drive, Turret turret, Shooter shooter, Intake intake, Wait wait,
                           DoubleSupplier turretOffsetDeg) {
        this.turret = turret;
        this.turretOffsetDeg = turretOffsetDeg;
        addCommands(shootSequence(drive, shooter, intake, wait));
        addRequirements(shooter, intake);
    }

    private SequentialCommandGroup shootSequence(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos()), shooter),

                new CommandBase() {
                    {
                        addRequirements(shooter);
                    }

                    @Override
                    public void execute() {
                        StationaryShot shot = calculateStationaryShot(drive);
                        ShooterAimingModel.Solution solution = shooter.aimForDistance(shot.distanceIn);
                        addShotTelemetry(drive, shooter, shot, solution);
                    }

                    @Override
                    public boolean isFinished() {
                        return shooter.isHoodSettled()
                                && turretReady()
                                && shooter.isAtTargetVelocity(RPM_TOLERANCE);
                    }
                },

                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos()), shooter),
                        new WaitCommand(100),
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
                                StationaryShot shot = calculateStationaryShot(drive);
                                ShooterAimingModel.Solution solution = shooter.aimForDistance(shot.distanceIn);
                                addShotTelemetry(drive, shooter, shot, solution);
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
                            shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                            intake.setSpeed(0);
                        }, shooter, intake)
                )
        );
    }

    private StationaryShot calculateStationaryShot(Drive drive) {
        Pose robot = drive.follower.getPose();
        if (robot == null) return new StationaryShot(MIN_DIST, 135.0);

        Pose goal = FieldConstants.goalAimPointForAlliance(Robot.ALLIANCE == Robot.Alliance.BLUE);
        double headingRad = robot.getHeading();
        double turretX = robot.getX() + PPTracking.TURRET_FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = robot.getY() + PPTracking.TURRET_FORWARD_OFFSET_IN * Math.sin(headingRad);

        double aimX = goal.getX() - turretX;
        double aimY = goal.getY() - turretY;
        double distance = Math.hypot(aimX, aimY);
        return new StationaryShot(distance, turretAngleDeg(aimX, aimY, headingRad));
    }

    private boolean turretReady() {
        return turret.atTarget(TURRET_TOLERANCE_DEG);
    }

    private void addShotTelemetry(Drive drive, Shooter shooter, StationaryShot shot,
                                  ShooterAimingModel.Solution solution) {
        drive.telemetry.addData("Stationary Distance", shot.distanceIn);
        drive.telemetry.addData("Stationary Turret Target", shot.turretAngleDeg);
        drive.telemetry.addData("Stationary Turret Ready", turretReady());
        drive.telemetry.addData("Stationary Hood", shooter.getTargetHoodPosition());
        drive.telemetry.addData("Stationary Profile", solution.profileName);
        drive.telemetry.addData("Stationary Target RPM", shooter.getTargetVelocity());
        drive.telemetry.addData("Stationary Actual RPM", shooter.getVelocity());
        drive.telemetry.addData("Stationary RPM Ready", shooter.isAtTargetVelocity(RPM_TOLERANCE));
    }

    private double turretAngleDeg(double fieldDx, double fieldDy, double headingRad) {
        double bearingDeg = Math.toDegrees(Math.atan2(fieldDy, fieldDx));
        double headingDeg = Math.toDegrees(headingRad);
        double deflectionDeg = wrap180(bearingDeg - headingDeg);
        return wrap360(135.0 - deflectionDeg + turretOffsetDeg.getAsDouble());
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

    private static class StationaryShot {
        final double distanceIn;
        final double turretAngleDeg;

        StationaryShot(double distanceIn, double turretAngleDeg) {
            this.distanceIn = distanceIn;
            this.turretAngleDeg = turretAngleDeg;
        }
    }
}
