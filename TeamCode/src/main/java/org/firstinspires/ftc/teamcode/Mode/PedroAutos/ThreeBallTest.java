package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;

@Autonomous(name = "chudAuto", group = "Auto")
public class ImAChud extends OpMode {

    private Robot negabot;
    private Follower follower;
    private Paths paths;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        negabot = new Robot(hardwareMap, telemetry);
        follower = negabot.drive.follower;

        paths = new Paths(follower);

        follower.setStartingPose(
                new Pose(18.577, 122.938, Math.toRadians(234.5))
        );

        telemetry.addLine("Pedro Paths Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, paths.Path1),
                        new FollowPathCommand(follower, paths.Path2),
                        new FollowPathCommand(follower, paths.Path3),
                        new FollowPathCommand(follower, paths.Path4),
                        new FollowPathCommand(follower, paths.Path5)
                )
        );
    }

    @Override
    public void loop() {
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.577, 122.938),
                                    new Pose(36.000, 107.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(234.5))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(36.000, 107.000),
                                    new Pose(43.070, 83.655)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(234.5),
                            Math.toRadians(180)
                    )
                    .setReversed(true)
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(43.070, 83.655),
                                    new Pose(19.168, 83.773)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(19.168, 83.773),
                                    new Pose(35.970, 107.083)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(234.5)
                    )
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(35.970, 107.083),
                                    new Pose(17.867, 107.083)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(234.5),
                            Math.toRadians(180)
                    )
                    .build();
        }
    }
}
