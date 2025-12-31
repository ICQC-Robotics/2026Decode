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
public class ChudAuto extends OpMode {

    private Robot negabot;
    private Follower follower;
    private Paths paths;

    private static final Pose START_POSE = new Pose(21.000, 126.000, Math.toRadians(234.6));

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        negabot = new Robot(hardwareMap, telemetry);
        follower = negabot.drive.follower;

        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);
    }

    @Override
    public void start() {
        if (paths == null) {
            follower.setStartingPose(START_POSE);
            paths = new Paths(follower);
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, paths.Path1, false, 1.0),
                        new FollowPathCommand(follower, paths.Path2, false, 1.0),
                        new FollowPathCommand(follower, paths.Path3, false, 1.0)
                )
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public static class Paths {
        public final PathChain Path1;
        public final PathChain Path2;
        public final PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            START_POSE,
                            new Pose(37.000, 106.000)
                        )
                    )
                    .setConstantHeadingInterpolation(START_POSE.getHeading())
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(37.000, 106.000),
                            new Pose(42.685, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(234.6), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.685, 84.000),
                            new Pose(18.685, 84.000)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}

