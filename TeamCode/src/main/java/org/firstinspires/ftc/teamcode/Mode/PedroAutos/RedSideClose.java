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
public class RedSideClose extends CommandOpMode {
    // Mirror over vertical line x = 72 => x' = 144 - x
    private static final double MIRROR_X = 72.0;

    // Turret: 0 = left, 135 = forward, 270 = right
    // Mirroring left/right => angle' = 270 - angle
    private static double mirrorTurretDeg(double deg) {
        return 270.0 - deg;
    }

    // Field heading (0° = +X/right, CCW positive): mirror across vertical axis => heading' = 180° - heading
    private static double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg <= -180.0) deg += 360.0;
        if (deg > 180.0) deg -= 360.0;
        return deg;
    }

    private static double mirrorHeadingDeg(double deg) {
        return normalizeDeg(180.0 - deg);
    }

    private static double mh(double deg) { // mirror heading in degrees -> radians
        return Math.toRadians(mirrorHeadingDeg(deg));
    }

    private static double mx(double x) {   // mirror x over MIRROR_X
        return 2.0 * MIRROR_X - x;
    }

    Robot negabot;

    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10,
            GatePath, GateBack, GatePath1, HitGate;

    Pose startPose = new Pose(mx(26.241), 133.326, mh(54));

    Pose gateIntake  = new Pose(mx(14),     60, mh(-20));
    Pose gateIntake1 = new Pose(mx(16.302), 65, mh(-20));

    final double COVER_OPEN  = 0.1;
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
                new InstantCommand(() -> { negabot.shooter.setVelocity(3900); }),
                new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new InstantCommand(() -> { negabot.shooter.setHoodPos(0.15); }),

                new SequentialCommandGroup(
                        shotPrep(f, Path1, 0,    0.35),
                        new WaitCommand(1000),
                        shoot(),

                        new FollowPathCommand(f, Path2, true),
                        new WaitCommand(1000),

                        shotPrep(f, Path3, 50.5, 0.1),
                        shoot(),

                        new FollowPathCommand(f, Path4, true),
                        new WaitCommand(1000),

                        shotPrep(f, Path5, 71,   0.1),
                        shoot(),

                        new FollowPathCommand(f, Path6, true),
                        new FollowPathCommand(f, Path7, true),
                        new WaitCommand(1000),

                        shotPrep(f, Path8, 73,   0.1),
                        shoot(),

                        new FollowPathCommand(f, Path9, true),

                        shotPrep(f, Path10, 73,  0.1),
                        shoot(),

                        // forward stays forward under mirror
                        new InstantCommand(() -> { negabot.turret.setTargetDeg(mirrorTurretDeg(135)); })
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

    public Command shotPrep(Follower f, PathChain path, double turretAngleDeg, double hoodPos) {
        final double mirroredTurretDeg = mirrorTurretDeg(turretAngleDeg);

        return new ParallelCommandGroup(
                new FollowPathCommand(f, path, true),
                new InstantCommand(() -> { negabot.intake.setSpeed(0); }),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_OPEN); })
                ),
                new InstantCommand(() -> { negabot.turret.setTargetDeg(mirroredTurretDeg); }),
                new InstantCommand(() -> { negabot.shooter.setHoodPos(hoodPos); })
        );
    }

    public void path(Follower follower) {

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(26.241), 132.326),
                                new Pose(mx(58.811), 84.606)
                        )
                )
                .setLinearHeadingInterpolation(mh(54), mh(10))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(58.811), 84.606),
                                new Pose(mx(21.009), 84.327)
                        )
                )
                .setLinearHeadingInterpolation(mh(10), mh(0))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(21.009), 84.327),
                                new Pose(mx(58.857), 84.813)
                        )
                )
                .setLinearHeadingInterpolation(mh(0), mh(54))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(58.857), 84.813),
                                new Pose(mx(61.800), 55.034),
                                new Pose(mx(19.108), 60.266)
                        )
                )
                .setLinearHeadingInterpolation(mh(54), mh(0))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(19.108), 60.266),
                                new Pose(mx(52.366), 67.788),
                                new Pose(mx(58.890), 84.600)
                        )
                )
                .setLinearHeadingInterpolation(mh(0), mh(71))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(58.890), 84.600),
                                new Pose(mx(71.483), 31.948),
                                new Pose(mx(16.789), 35.886)
                        )
                )
                .setLinearHeadingInterpolation(mh(71), mh(0))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(16.789), 35.886),
                                new Pose(mx(38.064), 72.593),
                                new Pose(mx(19.000), 69.408)
                        )
                )
                .setLinearHeadingInterpolation(mh(0), mh(0))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(19.000), 69.408),
                                new Pose(mx(47.001), 76.199),
                                new Pose(mx(58.499), 84.619)
                        )
                )
                .setLinearHeadingInterpolation(mh(0), mh(71))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(58.499), 84.619),
                                new Pose(mx(0.288),  35.425),
                                new Pose(mx(10.200), 21.994),
                                new Pose(mx(8.579),  9.362)
                        )
                )
                .setLinearHeadingInterpolation(mh(72), mh(90))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(8.579),  9.362),
                                new Pose(mx(58.394), 84.308)
                        )
                )
                .setLinearHeadingInterpolation(mh(90), mh(72))
                .build();

        GatePath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(53.766), 88.326),
                                new Pose(mx(39.925), 67.000),
                                new Pose(gateIntake.getX(), gateIntake.getY())
                        )
                )
                .setLinearHeadingInterpolation(mh(71), mh(-30))
                .build();

        GateBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(gateIntake.getX(), gateIntake.getY()),
                                new Pose(mx(58.85695), 84.81285)
                        )
                )
                .setLinearHeadingInterpolation(gateIntake.getHeading(), mh(71))
                .build();

        GatePath1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(53.766), 88.326),
                                new Pose(mx(39.925), 67.000),
                                new Pose(gateIntake1.getX(), gateIntake1.getY())
                        )
                )
                .setLinearHeadingInterpolation(mh(71), mh(-30))
                .build();

        HitGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(19.108), 60.266),
                                new Pose(mx(15.809), 70.234)
                        )
                )
                .setLinearHeadingInterpolation(mh(0), mh(0))
                .build();
    }

    @Override
    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}

