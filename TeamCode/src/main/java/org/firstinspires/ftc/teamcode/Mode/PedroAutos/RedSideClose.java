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

    Robot negabot;

    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15,
            GatePath, GateBack, GatePath1, HitGate;

    // Mirrored across x = 72:
    // x' = 144 - x
    // heading' = 180° - heading
    Pose startPose = new Pose(117.759, 132.326, Math.toRadians(126));

    Pose gateIntake  = new Pose(mirrorX(14),     60, mirrorHeadingRad(Math.toRadians(-20)));
    Pose gateIntake1 = new Pose(mirrorX(16.302), 65, mirrorHeadingRad(Math.toRadians(-20)));

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
                new InstantCommand(() -> { negabot.shooter.setVelocity(3400); }),
                new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new SequentialCommandGroup(
                        shotPrep(f, Path1, mirrorTurretDeg(49), 0.3),
                        new WaitCommand(500),
                        shoot(),

                        new FollowPathCommand(f, Path2, true),
                        new FollowPathCommand(f, Path3, true),

                        shotPrep(f, Path4, mirrorTurretDeg(50), 0.15),
                        new WaitCommand(500),
                        shoot(),

                        new FollowPathCommand(f, Path5, true),
                        new FollowPathCommand(f, Path6, true),
                        new FollowPathCommand(f, Path7, true),

                        new WaitCommand(500),
                        shotPrep(f, Path8, mirrorTurretDeg(52), 0.15),
                        new WaitCommand(500),
                        shoot(),

                        new FollowPathCommand(f, Path9, true),
                        new FollowPathCommand(f, Path10, true),

                        shotPrep(f, Path11, mirrorTurretDeg(52), 0.15),
                        new WaitCommand(500),
                        shoot(),

                        new FollowPathCommand(f, Path12, true),
                        new FollowPathCommand(f, Path13, true),

                        shotPrep(f, Path14, mirrorTurretDeg(51), 0.15),
                        new WaitCommand(400),
                        shoot(),

                        new FollowPathCommand(f, Path15, true)
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

    public Command shotPrep(Follower f, PathChain path, double turretAngle, double hoodPos) {
        return new ParallelCommandGroup(
                new FollowPathCommand(f, path, true),
                new InstantCommand(() -> { negabot.intake.setSpeed(0); }),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_OPEN); })
                ),
                new InstantCommand(() -> { negabot.turret.setTargetDeg(turretAngle); })
        );
    }

    public void path(Follower follower) {

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(26.241), 132.326),
                                new Pose(mirrorX(58.510), 84.907)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(126))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(58.510), 84.907),
                                new Pose(mirrorX(46.239), 62.530)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(46.239), 62.530),
                                new Pose(mirrorX(18.670), 61.568)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mirrorX(18.670), 61.568),
                                new Pose(mirrorX(47.700), 68.846),
                                new Pose(mirrorX(58.447), 84.841)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(126))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(58.447), 84.841),
                                new Pose(mirrorX(22.315), 84.098)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(22.315), 84.098),
                                new Pose(mirrorX(28.457), 71.502)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(28.457), 71.502),
                                new Pose(mirrorX(23.558), 70.063)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(23.558), 70.063),
                                new Pose(mirrorX(58.633), 84.777)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(126))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(58.633), 84.777),
                                new Pose(mirrorX(44.998), 37.914)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(180))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(44.998), 37.914),
                                new Pose(mirrorX(16.662), 37.434)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(14.662), 37.434),
                                new Pose(mirrorX(58.473), 84.777)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(126))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(58.473), 84.777),
                                new Pose(mirrorX(17.500), 32.450)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(90))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(19.500), 32.450),
                                new Pose(mirrorX(19.400), 11.300)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(19.400), 11.300),
                                new Pose(mirrorX(58.550), 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(126))
                .build();

        Path15 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(58.550), 85.000),
                                new Pose(mirrorX(46.975), 75.170)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(126))
                .build();
    }

    @Override
    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }

    // ======================
    // Mirror helpers
    // ======================

    private static double mirrorX(double x) {
        // Mirror across x = 72 => x' = 144 - x
        return 144.0 - x;
    }

    private static double mirrorHeadingRad(double headingRad) {
        // Mirror across vertical axis => theta' = pi - theta
        double out = Math.PI - headingRad;
        while (out < 0) out += 2 * Math.PI;
        while (out >= 2 * Math.PI) out -= 2 * Math.PI;
        return out;
    }

    private static double mirrorTurretDeg(double turretDeg) {
        // 135° is straight forward, so mirror around 135:
        // turret' = 270 - turret
        double out = 270.0 - turretDeg;
        while (out < 0) out += 360.0;
        while (out >= 360.0) out -= 360.0;
        return out;
    }
}
