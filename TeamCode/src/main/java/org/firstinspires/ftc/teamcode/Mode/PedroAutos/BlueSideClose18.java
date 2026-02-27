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
public class BlueSideClose18 extends CommandOpMode {
        Robot negabot;
        PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16, GatePath, GateBack, GatePath1, HitGate;
        Pose startPose = new Pose(34.36974789915966, 136.4201680672269, Math.toRadians(0));

        Pose gateIntake = new Pose(14, 60, Math.toRadians(-20));
        Pose gateIntake1 = new Pose(16.302, 65, Math.toRadians(-20));
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
                        new InstantCommand(() -> { negabot.shooter.setVelocity(3345);}),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                        new SequentialCommandGroup(
                                shotPrep(f, Path1, 9),
                                shoot(),
                                new FollowPathCommand(f, Path2, true),
                                shotPrep(f, Path3, 100),
                                new WaitCommand(500),
                                shoot(),
                                new FollowPathCommand(f, Path4, true),
                                new FollowPathCommand(f, Path5, true),
                                shotPrep(f, Path6, 106),
                                shoot(),
                                new FollowPathCommand(f, Path7, true),
                                new FollowPathCommand(f, Path8, true),
                                new FollowPathCommand(f, Path9, true),
                                new WaitCommand(500),
                                shotPrep(f, Path10, 60),
                                shoot(),
                                new FollowPathCommand(f, Path11, true),
                                shotPrep(f, Path12, 60),
                                shoot(),
                                new FollowPathCommand(f, Path13, true),
                                new FollowPathCommand(f, Path14, true),
                                shotPrep(f, Path15, 60),
                                shoot(),
                                new FollowPathCommand(f, Path16, true)

                        )
                        /*
                        new SequentialCommandGroup(
                                new InstantCommand(() -> { t.setTargetDeg(0); }),
                                new FollowPathCommand(f, Path1, true),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} ),
                                new FollowPathCommand(f, Path2, true),
                                new InstantCommand(() -> {intake.setSpeed(0);} ),
                                new InstantCommand(() -> { t.setTargetDeg(50.5); }),
                                new InstantCommand(() -> {negabot.shooter.setHoodPos(0.1); }),
                                new FollowPathCommand(f, Path3, true),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} ),
                                new FollowPathCommand(f, Path4, true),
                                new InstantCommand(() -> { t.setTargetDeg(73.5); }),
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

        public Command shotPrep(Follower f, PathChain path, double turretAngle){
            return new ParallelCommandGroup(

                    new FollowPathCommand(f, path, true),
                    new InstantCommand(() -> {negabot.intake.setSpeed(0);} ),
                    new SequentialCommandGroup(
                            new WaitCommand(500),
                            new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);}
                            )),
                    new InstantCommand(() -> { negabot.turret.setTargetDeg(turretAngle); })

            );
        }

        public void path(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(34.370, 136.420),

                                    new Pose(50.371, 83.649)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.371, 83.649),

                                    new Pose(18.457, 83.952)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.457, 83.952),

                                    new Pose(50.824, 83.952)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(50.824, 83.952),
                                    new Pose(52.183, 59.776),
                                    new Pose(37.553, 60.265)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(37.553, 60.265),

                                    new Pose(15.599, 59.607)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.599, 59.607),
                                    new Pose(41.420, 70.210),
                                    new Pose(50.681, 84.238)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(50.681, 84.238),
                                    new Pose(63.307, 32.508),
                                    new Pose(36.810, 35.723)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(36.810, 35.723),

                                    new Pose(11.936, 36.270)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.936, 36.270),

                                    new Pose(15.513, 67.748)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.513, 67.748),

                                    new Pose(50.765, 83.742)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.765, 83.742),

                                    new Pose(11.454, 8.950)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.454, 8.950),
                                    new Pose(16.011, 59.114),
                                    new Pose(50.820, 83.927)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.820, 83.927),

                                    new Pose(8.672, 41.019)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))

                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.672, 41.019),

                                    new Pose(8.273, 9.868)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Path15 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.273, 9.868),

                                    new Pose(50.420, 83.921)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))

                    .build();

            Path16 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.420, 83.921),

                                    new Pose(43.912, 74.801)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(60))

                    .build();
        }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}

