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
public class RedSideClose18 extends CommandOpMode {

    Robot negabot;

    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16;

    // Mirror across x = 72 => x' = 144 - x
    private static double mx(double x) { return 144.0 - x; }

    // Mirror a heading across the same vertical mirror line (reflect x component):
    // if 0 rad is +x, mirrored heading is pi - heading
    private static double mHeading(double rad) { return normRad(Math.PI - rad); }

    // Normalize to (-pi, pi]
    private static double normRad(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad >  Math.PI)  rad -= 2.0 * Math.PI;
        return rad;
    }

    // Turret: 135 deg is straight forward; mirror turret around that
    private static double mTurretDeg(double deg) { return 270.0 - deg; }

    Pose startPose = new Pose(
            mx(33.00965344815612),
            136.4201680672269,
            mHeading(Math.toRadians(0))
    );

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

                        shotPrep(f, Path3,  mTurretDeg(100)),
                        new WaitCommand(500),
                        shoot(),

                        new FollowPathCommand(f, Path4, true),
                        new FollowPathCommand(f, Path5, true),

                        shotPrep(f, Path6,  mTurretDeg(106)),
                        shoot(),

                        new FollowPathCommand(f, Path7, true),
                        new FollowPathCommand(f, Path8, true),
                        new FollowPathCommand(f, Path9, true),
                        new WaitCommand(500),

                        shotPrep(f, Path10, mTurretDeg(60)),
                        shoot(),

                        new FollowPathCommand(f, Path11, true),

                        shotPrep(f, Path12, mTurretDeg(60)),
                        shoot(),

                        new FollowPathCommand(f, Path13, true),
                        new FollowPathCommand(f, Path14, true),

                        shotPrep(f, Path15, mTurretDeg(60)),
                        shoot(),

                        new FollowPathCommand(f, Path16, true)
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
                new BezierLine(
                        new Pose(mx(18.457), 83.952),
                        new Pose(mx(50.824), 83.952)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(90))
        ).build();

        Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(mx(50.824), 83.952),
                        new Pose(mx(52.183), 59.776),
                        new Pose(mx(40.553), 60.265)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(0))
        ).build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(40.553), 60.265),
                        new Pose(mx(15.599), 59.607)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(0))
        ).build();

        Path6 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(mx(15.599), 59.607),
                        new Pose(mx(41.420), 70.210),
                        new Pose(mx(50.681), 84.238)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(90))
        ).build();

        Path7 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(mx(50.681), 84.238),
                        new Pose(mx(63.307), 32.508),
                        new Pose(mx(40.810), 35.723)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(0))
        ).build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(40.810), 35.723),
                        new Pose(mx(11.936), 36.270)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(0))
        ).build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(11.936), 36.270),
                        new Pose(mx(15.513), 67.748)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(0)),
                mHeading(Math.toRadians(90))
        ).build();

        Path10 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(15.513), 67.748),
                        new Pose(mx(50.765), 83.742)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(60))
        ).build();

        Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(50.765), 83.742),
                        new Pose(mx(11.454), 8.950)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(60)),
                mHeading(Math.toRadians(90))
        ).build();

        Path12 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(mx(11.454), 8.950),
                        new Pose(mx(16.011), 59.114),
                        new Pose(mx(50.820), 83.927)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(60))
        ).build();

        Path13 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(50.820), 83.927),
                        new Pose(mx(8.672), 41.019)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(60)),
                mHeading(Math.toRadians(90))
        ).build();

        Path14 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(8.672), 41.019),
                        new Pose(mx(8.273), 9.868)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(90))
        ).build();

        Path15 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(8.273), 9.868),
                        new Pose(mx(50.420), 83.921)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(90)),
                mHeading(Math.toRadians(60))
        ).build();

        Path16 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(mx(50.420), 83.921),
                        new Pose(mx(43.912), 74.801)
                )
        ).setLinearHeadingInterpolation(
                mHeading(Math.toRadians(60)),
                mHeading(Math.toRadians(60))
        ).build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}