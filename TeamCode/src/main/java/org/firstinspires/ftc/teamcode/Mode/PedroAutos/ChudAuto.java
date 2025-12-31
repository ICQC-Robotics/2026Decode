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

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;

    private static final Pose START_POSE =
            new Pose(21.000, 126.000, Math.toRadians(234.6));

    private SequentialCommandGroup autoSequence;

    public void buildPaths(Follower follower) {

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                START_POSE,
                                new Pose(37.000, 106.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(234.6))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(37.000, 106.000),
                                new Pose(37.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(234.6))

                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(37.000, 84.000),
                                new Pose(18, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(234.6), Math.toRadians(180))
                .build();
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        negabot = new Robot(hardwareMap, telemetry);
        follower = negabot.drive.follower;

        follower.setStartingPose(START_POSE);
        buildPaths(follower);

        telemetry.addData("Path1", Path1 == null);
        telemetry.addData("Path2", Path2 == null);
        telemetry.addData("Path3", Path3 == null);
        telemetry.update();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().cancelAll();

        if (Path1 == null || Path2 == null || Path3 == null) {
            follower.setStartingPose(START_POSE);
            buildPaths(follower);
        }

        autoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Path1),
                new FollowPathCommand(follower, Path2),
                new FollowPathCommand(follower, Path3)
        );

        autoSequence.initialize();
        CommandScheduler.getInstance().schedule(autoSequence);
    }

    @Override
    public void loop() {
        follower.update();
        CommandScheduler.getInstance().run();
        telemetry.addData("active", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
