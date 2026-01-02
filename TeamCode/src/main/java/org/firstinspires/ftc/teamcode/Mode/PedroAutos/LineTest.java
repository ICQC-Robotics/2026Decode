package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;

@Autonomous
public class LineTest extends CommandOpMode {
        Robot negabot;
        PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;
        Pose startPose = new Pose(26.241, 133.326, Math.toRadians(54));
        Pose endPose = new Pose(72, 120, Math.toRadians(90));

        @Override
        public void initialize() {
                negabot = new Robot(hardwareMap, telemetry, startPose);

                Drive d = negabot.drive;
                Follower f = d.follower;

                path(f);

                negabot.schedule(
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, Path1, true),
                                new FollowPathCommand(f, Path2, true),
                                new FollowPathCommand(f, Path3, true),
                                new FollowPathCommand(f, Path4, true),
                                new FollowPathCommand(f, Path5, true),
                                new FollowPathCommand(f, Path6, true),
                                new FollowPathCommand(f, Path7, true),
                                new FollowPathCommand(f, Path8, true))
                );
        }

        public void path(Follower follower) {
                Path1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(26.241, 132.326),

                                        new Pose(48.726, 101.139)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))

                        .build();

                Path2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(48.726, 101.139),

                                        new Pose(40.649, 84.403)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(40.649, 84.403),

                                        new Pose(21.670, 83.997)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(21.670, 83.997),

                                        new Pose(48.772, 101.346)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))

                        .build();

                Path5 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(48.772, 101.346),

                                        new Pose(42.659, 76.633)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(90))

                        .build();

                Path6 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(42.659, 76.633),

                                        new Pose(38.356, 61.006)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                        .build();

                Path7 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(38.356, 61.006),

                                        new Pose(21.423, 60.928)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path8 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(21.423, 60.928),

                                        new Pose(48.639, 101.464)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(71))

                        .build();
        }
}

