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

@Autonomous(name = "chudAuto", group = "Auto")
public class ChudAuto extends OpMode {

    private Robot negabot;

    public PathChain Path1, Path2, Path3;

    private static final Pose START_POSE = new Pose(21.000, 126.000, Math.toRadians(234.6));

    public void buildPaths(Follower follower) {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                START_POSE,
                                new Pose(23, 111, 234.6)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(234.6))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(23, 111, 234.6),
                                new Pose(23, 100, 234.6)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(234.6))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(23, 100, 234.6),
                                new Pose(20, 84.000, 180)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(234.6),
                        Math.toRadians(180)
                )
                .build();
    }

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, telemetry);
        negabot.reset();

        negabot.drive.setStart(START_POSE);
        buildPaths(negabot.drive.follower);

        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                new FollowPathCommand(negabot.drive.follower, Path1, true),
                new FollowPathCommand(negabot.drive.follower, Path2, true),
                new FollowPathCommand(negabot.drive.follower, Path3, true)
        );

        negabot.schedule(autoSequence);

        telemetry.addData("Path1", Path1 == null);
        telemetry.addData("Path2", Path2 == null);
        telemetry.addData("Path3", Path3 == null);
        telemetry.update();
    }

    @Override
    public void loop() {
        negabot.run();
    }
}
