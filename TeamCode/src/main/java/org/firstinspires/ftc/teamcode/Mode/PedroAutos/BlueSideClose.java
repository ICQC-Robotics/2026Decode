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
        PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, GatePath, GateBack, GatePath1, HitGate;
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



                path(f);

                waitForStart();
                negabot.schedule(
                        new InstantCommand(() -> { negabot.shooter.setVelocity(3900);}),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                        new InstantCommand(() -> {negabot.shooter.setHoodPos(0.15); }),
                        new SequentialCommandGroup(
                                shotPrep(f, Path1, 0, 0.35),
                                new WaitCommand(1000),
                                shoot(),
                                new FollowPathCommand(f, Path2, true),
                                new WaitCommand(1000),
                                shotPrep(f, Path3, 50.5, 0.1),
                                shoot(),
                                new FollowPathCommand(f, Path4, true),
                                new WaitCommand(1000),
                                shotPrep(f, Path5, 71, 0.1),
                                shoot(),
                                new FollowPathCommand(f, Path6, true),
                                new FollowPathCommand(f, Path7, true),
                                new WaitCommand(1000),
                                shotPrep(f, Path8, 73, 0.1),
                                shoot(),
                                new FollowPathCommand(f, Path9, true),
                                shotPrep(f, Path10, 73, 0.1),
                                shoot(),
                                new InstantCommand(() -> {negabot.turret.setTargetDeg(135); })
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
                    new InstantCommand(() -> { negabot.turret.setTargetDeg(turretAngle); }),
                    new InstantCommand(() -> {negabot.shooter.setHoodPos(hoodPos); })

            );
        }

        public void path(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.241, 132.326),

                                    new Pose(58.811, 84.606)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(10))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.811, 84.606),

                                    new Pose(21.009, 84.327)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.009, 84.327),

                                    new Pose(58.857, 84.813)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.857, 84.813),
                                    new Pose(61.800, 55.034),
                                    new Pose(19.108, 60.266)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.108, 60.266),
                                    new Pose(52.366, 67.788),
                                    new Pose(58.890, 84.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(71))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.890, 84.600),
                                    new Pose(71.483, 31.948),
                                    new Pose(16.789, 35.886)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(71), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.789, 35.886),
                                    new Pose(38.064, 72.593),
                                    new Pose(19, 69.408)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19, 69.408),
                                    new Pose(47.001, 76.199),
                                    new Pose(58.499, 84.619)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(71))

                    .build();
            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.499, 84.619),
                                    new Pose(0.288, 35.425),
                                    new Pose(10.200, 21.994),
                                    new Pose(8.579, 9.362)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(90))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.579, 9.362),

                                    new Pose(58.394, 84.308)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72))

                    .build();

            GatePath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.766, 88.326),
                                    new Pose(39.925, 67),
                                    new Pose(gateIntake.getX(), gateIntake.getY())
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(71), Math.toRadians(-30))

                    .build();

            GateBack = follower.pathBuilder().addPath(
                            new BezierLine(

                                    new Pose(gateIntake.getX(), gateIntake.getY()),
                                    new Pose(58.85695, 84.81285)
                            )
                    ).setLinearHeadingInterpolation(gateIntake.getHeading(), Math.toRadians(71))

            .build();
            GatePath1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.766, 88.326),
                                    new Pose(39.925, 67),
                                    new Pose(gateIntake1.getX(), gateIntake1.getY())
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(71), Math.toRadians(-30))

                    .build();
            HitGate =  follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.108, 60.266),

                                    new Pose(15.809, 70.234)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}

