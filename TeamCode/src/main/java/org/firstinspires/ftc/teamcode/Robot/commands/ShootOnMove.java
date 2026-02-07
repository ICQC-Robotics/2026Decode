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
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class ShootOnMove extends SequentialCommandGroup {
    public enum Positions {
        OPEN_COVER(0.1),
        CLOSED_COVER(1);
        private final double pos;
        Positions(double pos) { this.pos = pos; }
        public double getPos() { return pos; }
    }
    private static final double RPM_TOLERANCE = 25;
    private static final double FEED_TIME_S = 1;
    public static final double MIN_DIST = 20;
    public static final double MAX_DIST = 150;
    public static final double MIN_V = 2820;
    public static final double MAX_V = 4200;
    private static final double[][] RPM_LUT = new double[][] {
            {  63, 3150 }, {  96, 3500 }, {  110, 3890 },
            {  125, 3930 }, {  130, 4110 }, {  140, 4180 }, {  145, 4200 },
    };

    private double spinUpRPM = MIN_V;
    private static final double SHOOTER_ANGLE_DEG = 40.0;
    private static final double WHEEL_RADIUS_IN = 2.835;
    private static final double SHOOTER_EFFICIENCY = 0.75;
    private static final double DRAG_COEFF = 0.65;

    public AutoAim(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        addCommands(shootSequence(drive, shooter, intake, wait));
        addRequirements(shooter, intake);
    }
    private SequentialCommandGroup shootSequence(Drive drive, Shooter shooter, Intake intake, Wait wait) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setMagazineCover(Positions.CLOSED_COVER.getPos()), shooter),

                new CommandBase() {
                    { addRequirements(shooter); }
                    @Override
                    public void execute() {
                        double d = calculateDistanceIn(drive);
                        spinUpRPM = Math.max(MIN_V, Math.min(getRpmForDistance(d), MAX_V));
                        shooter.setVelocity(spinUpRPM);
                    }
                    @Override
                    public boolean isFinished() {
                        return Math.abs(shooter.getVelocity() - spinUpRPM) <= RPM_TOLERANCE;
                    }
                },
                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setMagazineCover(Positions.OPEN_COVER.getPos()), shooter),
                        new WaitCommand(100),
                        new InstantCommand(() -> intake.setSpeed(-1), intake),
                        new CommandBase() {
                            { addRequirements(shooter, intake); }
                            @Override
                            public void initialize() { wait.start(); }
                            @Override
                            public void execute() {
                                shooter.setVelocity(getRpmForDistance(calculateDistanceIn(drive)));
                            }
                            @Override
                            public boolean isFinished() { return wait.elapsed() >= FEED_TIME_S; }
                        },
                        new InstantCommand(() -> {
                            shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
                            intake.setSpeed(0);
                        }, shooter, intake)
                ));
    }
    public double calculateDistanceIn(Drive drive) {
        Pose robot = drive.follower.getPose();
        Pose vel = drive.follower.getVelocity();
        if (robot == null || vel == null) return MIN_DIST;
        double gX = (Robot.ALLIANCE == Robot.Alliance.BLUE) ? FieldConstants.BLUE_GOAL_X : FieldConstants.RED_GOAL_X;
        double gY = (Robot.ALLIANCE == Robot.Alliance.BLUE) ? FieldConstants.BLUE_GOAL_Y : FieldConstants.RED_GOAL_Y;
        double dx = gX - robot.getX();
        double dy = gY - robot.getY();
        double staticDist = Math.hypot(dx, dy);
        double targetRPM = getRpmForDistance(staticDist);
        double v0 = (targetRPM * 0.104719755 * WHEEL_RADIUS_IN) * SHOOTER_EFFICIENCY;
        double vx0 = v0 * Math.cos(Math.toRadians(SHOOTER_ANGLE_DEG));
        double k = 0.976 * DRAG_COEFF;
        double tFlight = (Math.exp(staticDist * k) - 1) / (k * vx0);
        double vX = gX - (vel.getX() * tFlight);
        double vY = gY - (vel.getY() * tFlight);
        return Math.hypot(vX - robot.getX(), vY - robot.getY());
    }
    public static double getRpmForDistance(double distanceIn) {
        double d = Math.max(MIN_DIST, Math.min(distanceIn, MAX_DIST));
        for (int i = 0; i < RPM_LUT.length - 1; i++) {
            if (d >= RPM_LUT[i][0] && d <= RPM_LUT[i + 1][0]) {
                double t = (d - RPM_LUT[i][0]) / (RPM_LUT[i + 1][0] - RPM_LUT[i][0]);
                return RPM_LUT[i][1] + t * (RPM_LUT[i + 1][1] - RPM_LUT[i][1]);
            }
        }
        return RPM_LUT[RPM_LUT.length - 1][1];
    }
}