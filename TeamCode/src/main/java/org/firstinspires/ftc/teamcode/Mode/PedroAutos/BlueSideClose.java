package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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
        PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;
        Pose startPose = new Pose(26.241, 133.326, Math.toRadians(54));
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
                        new ShooterStandBy(negabot.shooter),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                        new InstantCommand(() -> {negabot.shooter.setHoodPos(0.3); }),
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

                                    new Pose(21.670, 83.997)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.670, 83.997),

                                    new Pose(58.857, 84.813)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.857, 84.813),
                                    new Pose(61.800, 55.034),
                                    new Pose(15.140, 60.101)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.140, 60.101),
                                    new Pose(52.366, 67.788),
                                    new Pose(58.890, 84.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(71))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.890, 84.600),
                                    new Pose(66.689, 31.452),
                                    new Pose(15.631, 35.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(71), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.962, 35.721),
                                    new Pose(46.670, 54.045),
                                    new Pose(58.499, 84.619)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(71))

                    .build();
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.499, 84.619),

                                    new Pose(69.706, 100.489)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(71), Math.toRadians(71))

                    .build();

        }

        public void run() {
            super.run();
            Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        }
}

