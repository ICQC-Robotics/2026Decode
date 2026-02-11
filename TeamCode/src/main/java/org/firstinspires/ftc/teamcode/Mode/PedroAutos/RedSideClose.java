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
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, GatePath, GateBack, GatePath1, HitGate;

    // Mirrored over x = 72  => x' = 144 - x
    // Heading mirrored (field heading) => heading' = 180 - heading
    Pose startPose = new Pose(117.759, 133.326, Math.toRadians(126));

    // Mirrored: x only, y same, heading -20 -> -160
    Pose gateIntake  = new Pose(130.000, 60, Math.toRadians(-160));
    Pose gateIntake1 = new Pose(127.698, 65, Math.toRadians(-160));

    final double COVER_OPEN = 0.1;
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
                new InstantCommand(() -> { negabot.shooter.setVelocity(3750);}),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new SequentialCommandGroup(
                        shotPrep(f, Path1,  mirrorTurretDeg(0), 0.3),
                        new WaitCommand(1000),
                        shoot(),
                        new FollowPathCommand(f, Path2, true),
                        new WaitCommand(1000),
                        shotPrep(f, Path3, mirrorTurretDeg(50.5), 0.15),
                        shoot(),
                        new FollowPathCommand(f, Path4, true),
                        new WaitCommand(1000),
                        shotPrep(f, Path5, mirrorTurretDeg(71), 0.15),
                        shoot(),
                        new FollowPathCommand(f, Path6, true),
                        new FollowPathCommand(f, Path7, true),
                        new WaitCommand(1000),
                        shotPrep(f, Path8, mirrorTurretDeg(72.5), 0.15),
                        shoot(),
                        new FollowPathCommand(f, Path9, true),
                        shotPrep(f, Path10, mirrorTurretDeg(72.5), 0.15),
                        shoot(),
                        new InstantCommand(() -> {negabot.turret.setTargetDeg(mirrorTurretDeg(135)); })
                )
                        /*
                        new SequentialCommandGroup(
                                new InstantCommand(() -> { t.setTargetDeg(270); }),
                                new FollowPathCommand(f, Path1, true),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} ),
                                new FollowPathCommand(f, Path2, true),
                                new InstantCommand(() -> {intake.setSpeed(0);} ),
                                new InstantCommand(() -> { t.setTargetDeg(219.5); }),
                                new InstantCommand(() -> {negabot.shooter.setHoodPos(0.1); }),
                                new FollowPathCommand(f, Path3, true),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} ),
                                new FollowPathCommand(f, Path4, true),
                                new InstantCommand(() -> { t.setTargetDeg(196.5); }),
                                new InstantCommand(() -> {negabot.shooter.setHoodPos(0.05); }),
                                new InstantCommand(() -> {intake.setSpeed(0);} ),
                                new FollowPathCommand(f, Path5, true),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} ),
                                new FollowPathCommand(f, Path6, true),
                                new InstantCommand(() -> {intake.setSpeed(0);} ),
                                new FollowPathCommand(f, Path7, true),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(0);} ),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} ))
                         */
        );
    }

    public Command shoot(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.intake.setSpeed(-1);} ),
                new WaitCommand(500),
                new InstantCommand(() -> {negabot.intake.setSpeed(-1);} ),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} )
        );
    }

    public Command shotPrep(Follower f, PathChain path, double turretAngle, double hoodPos){
        return new ParallelCommandGroup(
                new FollowPathCommand(f, path, true),
                new InstantCommand(() -> {negabot.intake.setSpeed(0);} ),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} )
                ),
                new InstantCommand(() -> { negabot.turret.setTargetDeg(turretAngle); })
        );
    }

    public void path(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(117.759, 132.326),
                                new Pose(85.189, 84.606)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(170))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.189, 84.606),
                                new Pose(123.991, 86.327)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(170), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.991, 84.327),
                                new Pose(85.143, 84.813)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(126))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.143, 84.813),
                                new Pose(82.200, 55.034),
                                new Pose(125.892, 62.266)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(180))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.892, 62.266),
                                new Pose(91.634, 67.788),
                                new Pose(85.110, 84.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(109))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.110, 84.600),
                                new Pose(72.517, 31.948),
                                new Pose(127.211, 35.886)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127.211, 35.886),
                                new Pose(105.936, 72.593),
                                new Pose(126.000, 69.408)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(126.000, 69.408),
                                new Pose(96.999, 76.199),
                                new Pose(85.501, 84.619)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(109))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.501, 84.619),
                                new Pose(143.712, 35.425),
                                new Pose(126.800, 21.994),
                                new Pose(120.421, 9.362)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(90))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.421, 9.362),
                                new Pose(85.606, 84.308)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(108))
                .build();

        GatePath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90.234, 88.326),
                                new Pose(104.075, 67.000),
                                new Pose(gateIntake.getX(), gateIntake.getY())
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(-150))
                .build();

        GateBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(gateIntake.getX(), gateIntake.getY()),
                                new Pose(85.14305, 84.81285)
                        )
                ).setLinearHeadingInterpolation(gateIntake.getHeading(), Math.toRadians(109))
                .build();

        GatePath1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90.234, 88.326),
                                new Pose(104.075, 67.000),
                                new Pose(gateIntake1.getX(), gateIntake1.getY())
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(-150))
                .build();

        HitGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.892, 60.266),
                                new Pose(128.191, 70.234)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
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

