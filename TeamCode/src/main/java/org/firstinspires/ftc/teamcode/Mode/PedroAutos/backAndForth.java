package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;


@Autonomous(group = "!",name = "Pedro back and forth")
public class backAndForth extends OpMode {
    private Follower follower;
    private Robot negabot;
    private SequentialCommandGroup autoSequence;
    public PathChain Path1, Path2, Path3, Path4;

    public void buildPaths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 72.000), new Pose(72.000, 120.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 120.000), new Pose(72.000, 72.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 72.000), new Pose(72.000, 120.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 120.000), new Pose(72.000, 72.000))
                )
                .setTangentHeadingInterpolation()
                .build();
    }


    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        negabot = new Robot(hardwareMap, telemetry);
        follower = negabot.drive.follower;
        buildPaths(follower);
        follower.setStartingPose(new Pose(23.007, 125.959));
    }

    public void start(){
        // build a sequential group of commands
        autoSequence = new SequentialCommandGroup(
                // Path 1
                new FollowPathCommand(follower, Path1),
                new FollowPathCommand(follower, Path2),
                new FollowPathCommand(follower, Path3),
                new FollowPathCommand(follower, Path4)
        );

        autoSequence.initialize();  // prepare all commands
        CommandScheduler.getInstance().schedule(autoSequence);

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}