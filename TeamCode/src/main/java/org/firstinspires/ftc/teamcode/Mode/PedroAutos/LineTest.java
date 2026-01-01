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
    PathChain Path1, Path2, Path3, Path4;

    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry);
        Drive d = negabot.drive;
        Follower f = d.follower;
        d.setStartPose(new Pose(72, 72, Math.toRadians(90)));
        path(f);

        negabot.schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(f, Path1),
                        new FollowPathCommand(f, Path2),
                        new FollowPathCommand(f, Path3),
                        new FollowPathCommand(f, Path4)
                )
        );
    }

    public void path(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 72.000), new Pose(72.000, 120.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 120.000), new Pose(72.000, 72.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 72.000), new Pose(72.000, 120.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 120.000), new Pose(72.000, 72.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }
}
