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
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

@Autonomous
public class BlueSideClose extends CommandOpMode {
        Robot negabot;
        PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, GatePath, GateBack, GatePath1, HitGate;
        Pose startPose = new Pose(26.241, 133.326, Math.toRadians(54));

        Pose gateIntake = new Pose(14, 60, Math.toRadians(-20));
        Pose gateIntake1 = new Pose(16.302, 65, Math.toRadians(-20));
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
                        new InstantCommand(() -> { negabot.shooter.setVelocity(3470);}),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                        new SequentialCommandGroup(
                                shotPrep(f, Path1, 50, 0.3),
                                new WaitCommand(1000),
                                shoot(),
                                new FollowPathCommand(f, Path2, true),
                                new FollowPathCommand(f, Path3, true),
                                new WaitCommand(1000),
                                new FollowPathCommand(f, Path4, true),
                                new FollowPathCommand(f, Path5, true),
                                new WaitCommand(500),
                                shotPrep(f, Path6, 52, 0.15),
                                new WaitCommand(500),
                                shoot(),
                                new FollowPathCommand(f, Path7, true),
                                new WaitCommand(1000),
                                shotPrep(f, Path8, 52, 0.15),
                                new WaitCommand(500),
                                shoot(),
                                new FollowPathCommand(f, Path9, true),
                                new FollowPathCommand(f, Path10, true),
                                new WaitCommand(1000),
                                shotPrep(f, Path11, 55, 0.15),
                                new WaitCommand(500),
                                shoot()

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

        public Command shotPrep(Follower f, PathChain path, double turretAngle, double hoodPos){
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
                                    new Pose(26.241, 133.326),

                                    new Pose(58.510, 84.907)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(54))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.510, 84.907),

                                    new Pose(42.239, 62.530)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.239, 62.530),

                                    new Pose(15.670, 61.568)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.670, 61.568),

                                    new Pose(26.768, 70.317)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.768, 70.317),

                                    new Pose(18.524, 69.175)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.524, 69.175),
                                    new Pose(47.700, 68.846),
                                    new Pose(58.447, 84.841)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(54))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.447, 84.841),

                                    new Pose(20.315, 84.098)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.315, 84.098),

                                    new Pose(58.633, 84.777)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.633, 84.777),

                                    new Pose(40.998, 37.914)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.998, 37.914),

                                    new Pose(14.662, 37.434)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.662, 37.434),

                                    new Pose(58.473, 84.777)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))

                    .build();
        }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}

