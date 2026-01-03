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
public class TrueJesterBlue extends CommandOpMode {
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
             //   new ShooterStandBy(negabot.shooter),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new InstantCommand(() -> {negabot.shooter.setHoodPos(0.3); }),
                new SequentialCommandGroup(
                        new FollowPathCommand(f, Path1, true),
                        new FollowPathCommand(f, Path2, true),
                        new FollowPathCommand(f, Path3, true),
                        new FollowPathCommand(f, Path4, true)
                )
        );

    }

    public void path(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(26.241, 132.326),
                                new Pose(88.587, 56.565),
                                new Pose(17.507, 60.009)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.507, 60.009),

                                new Pose(53.766, 88.326)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(71))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.766, 88.326),

                                new Pose(17.625, 62.894)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(71), Math.toRadians(-30))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.625, 62.894),

                                new Pose(53.672, 88.279)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(71))

                .build();
    }

}
