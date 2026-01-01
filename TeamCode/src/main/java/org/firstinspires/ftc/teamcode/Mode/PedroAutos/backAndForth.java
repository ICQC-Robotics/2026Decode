package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;


@Autonomous(group = "!",name = "back and forth testing")
public class backAndForth extends OpMode {
    private Robot negabot;
    public PathChain Path1, Path2, Path3, Path4;

    public void buildPaths(Follower follower) {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.000, 72.000, 90),
                                new Pose(72.000, 120.000, 90)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.000, 120.000, 90),
                                new Pose(72.000, 72.000, 90)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.000, 72.000, 90),
                                new Pose(72.000, 120.000, 90)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.000, 120.000, 90),
                                new Pose(72.000, 72.000, 90)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, telemetry);
        Follower follower = negabot.drive.follower;

        negabot.reset();

        buildPaths(follower);
        follower.setStartingPose(new Pose(72, 72, 90));

        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Path1, true),
                new FollowPathCommand(follower, Path2, true),
                new FollowPathCommand(follower, Path3, true),
                new FollowPathCommand(follower, Path4, true)
        );

        negabot.schedule(autoSequence);
    }

    @Override
    public void loop() {
        negabot.run();
    }
}