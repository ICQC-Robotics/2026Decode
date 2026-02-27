package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

@Autonomous
public class RedSideClose12 extends CommandOpMode {
    Robot negabot;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16, GatePath, GateBack, GatePath1, HitGate;

    // Mirror rules:
    // Field mirror about x = 72  =>  x' = 144 - x
    // Pose heading mirror        =>  heading' = pi - heading
    // Turret mirror (135 forward)=>  turret' = 270 - turret
    private static double mx(double x) { return 144.0 - x; }
    private static double mHeading(double rad) { return normRad(Math.PI - rad); }
    private static double normRad(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad >  Math.PI)  rad -= 2.0 * Math.PI;
        return rad;
    }
    private static double mTurretDeg(double deg) { return 270.0 - deg; }

    Pose startPose = new Pose(mx(34.36974789915966), 136.4201680672269, mHeading(Math.toRadians(0)));

    Pose gateIntake  = new Pose(mx(14),     60, mHeading(Math.toRadians(-20)));
    Pose gateIntake1 = new Pose(mx(16.302), 65, mHeading(Math.toRadians(-20)));

    final double COVER_OPEN = 0.25;
    final double COVER_CLOSE = 1.0;

    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, startPose);

        Drive d = negabot.drive;
        Follower f = d.follower;
        Turret t = negabot.turret;
        Intake intake = negabot.intake;

        t.resetEncoder();

        path(f);

        waitForStart();
        negabot.schedule(
                new InstantCommand(() -> { negabot.shooter.setVelocity(3345); }),
                new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new SequentialCommandGroup(
                        shotPrep(f, Path1,  mTurretDeg(9)),
                        shoot(),
                        new FollowPathCommand(f, Path2, true),
                        new FollowPathCommand(f, Path3, true),

                        shotPrep(f, Path4,  mTurretDeg(95)),
                        new WaitCommand(500),
                        shoot(),

                        new FollowPathCommand(f, Path5, true),
                        new FollowPathCommand(f, Path6, true),

                        shotPrep(f, Path7,  mTurretDeg(106)),
                        shoot(),

                        new FollowPathCommand(f, Path8, true),
                        new FollowPathCommand(f, Path9, true),
                        new FollowPathCommand(f, Path10, true),

                        new WaitCommand(800),
                        shotPrep(f, Path11, mTurretDeg(60)),
                        shoot(),

                        new FollowPathCommand(f, Path12, true),
                        new WaitCommand(3000),
                        new FollowPathCommand(f, Path13, true)
                )
        );
    }

    public Command shoot() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                new WaitCommand(500),
                new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_CLOSE); })
        );
    }

    public Command shotPrep(Follower f, PathChain path, double turretAngleDeg) {
        return new ParallelCommandGroup(
                new FollowPathCommand(f, path, true),
                new InstantCommand(() -> { negabot.intake.setSpeed(0); }),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_OPEN); })
                ),
                new InstantCommand(() -> { negabot.turret.setTargetDeg(turretAngleDeg); })
        );
    }

    public void path(Follower follower) {

        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(34.370), 136.420),
                        new Pose(mx(50.371), 83.649)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(10))
        ).build();

        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(50.371), 83.649),
                        new Pose(mx(18.457), 83.952)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(0))
        ).build();

        Path3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(mx(18.457), 83.952),
                        new Pose(mx(51.136), 77.619),
                        new Pose(mx(24.184), 73.024),
                        new Pose(mx(14.989), 76.848)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(90))
        ).build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(14.989), 76.848),
                        new Pose(mx(50.824), 83.952)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(90))
        ).build();

        Path5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(mx(50.824), 83.952),
                        new Pose(mx(52.183), 59.776),
                        new Pose(mx(37.553), 60.265)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(0))
        ).build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(37.553), 60.265),
                        new Pose(mx(15.599), 59.607)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(0))
        ).build();

        Path7 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(mx(15.599), 59.607),
                        new Pose(mx(41.420), 70.210),
                        new Pose(mx(50.681), 84.238)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(90))
        ).build();

        Path8 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(mx(50.681), 84.238),
                        new Pose(mx(63.307), 32.508),
                        new Pose(mx(36.810), 35.723)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(0))
        ).build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(36.810), 35.723),
                        new Pose(mx(11.936), 36.270)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(0))
        ).build();

        Path10 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(11.936), 36.270),
                        new Pose(mx(15.513), 67.748)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(90))
        ).build();

        Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(15.513), 67.748),
                        new Pose(mx(50.765), 83.742)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(60))
        ).build();

        Path12 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(50.765), 83.742),
                        new Pose(mx(15.639), 67.577)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(90))
        ).build();

        Path13 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(15.639), 67.577),
                        new Pose(mx(22.954), 67.531)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(90))
        ).build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}