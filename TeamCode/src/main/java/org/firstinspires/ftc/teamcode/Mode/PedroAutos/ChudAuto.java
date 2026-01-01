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

@Autonomous(name = "chudAuto", group = "Auto")
public class ChudAuto extends OpMode {

    private Robot negabot;
    private Follower follower;

    private PathChain p1, p2, p3;
    private SequentialCommandGroup autoSequence;

    private static final double START_HEADING = Math.toRadians(234.6);

    private final Pose startPose =
            new Pose(21.000, 126.000, START_HEADING);

    public void buildPaths() {

        Pose p1End = new Pose(
                37.000, 106.000, START_HEADING
        );

        Pose p2End = new Pose(
                42.685, 84.000, Math.toRadians(180)
        );

        Pose p3End = new Pose(
                18.685, 84.000, Math.toRadians(180)
        );

        p1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1End))
                .setConstantHeadingInterpolation(START_HEADING)
                .build();

        p2 = follower.pathBuilder()
                .addPath(new BezierLine(p1End, p2End))
                .setLinearHeadingInterpolation(
                        p1End.getHeading(),
                        p2End.getHeading()
                )
                .build();

        p3 = follower.pathBuilder()
                .addPath(new BezierLine(p2End, p3End))
                .setConstantHeadingInterpolation(p3End.getHeading())
                .build();
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        negabot = new Robot(hardwareMap, telemetry);
        follower = negabot.drive.follower;

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().cancelAll();

        autoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, p1, false),
                new FollowPathCommand(follower, p2, false),
                new FollowPathCommand(follower, p3, false)
        );

        autoSequence.initialize();
        CommandScheduler.getInstance().schedule(autoSequence);
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
}
