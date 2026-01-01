package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;

/**
* @author
 * In the name of Allah, the Most Compassionate, the Most Merciful
 * True Pedro
 * Brutal
 */

@Autonomous(group="!", name="Auto")
public class Auto extends CommandOpMode {
    Robot negabot;
    Follower follower = negabot.drive.follower;
    PathChain moveToShoot, moveToGrab;

    public void redClose() {
        moveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(0, 0, 0),
                        new Pose(10, 10, 10)
                ))
                .addPath(new BezierCurve(
                        new Pose(10, 10, 10),
                        new Pose(20, 20, 20)
                ))
                .setLinearHeadingInterpolation(0, 20)
                .build();

        moveToGrab = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(20, 20, 20),
                        new Pose(30, 30, 30)
                ))
                .addPath(new BezierCurve(
                        new Pose(30, 30, 30),
                        new Pose(40, 40, 40)
                ))
                .setLinearHeadingInterpolation(20, 40)
                .build();
    }

    public void redFar() {
        moveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(0, 0, 0),
                        new Pose(10, 10, 10)
                ))
                .addPath(new BezierCurve(
                        new Pose(10, 10, 10),
                        new Pose(20, 20, 20)
                ))
                .setLinearHeadingInterpolation(0, 20)
                .build();

        moveToGrab = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(20, 20, 20),
                        new Pose(30, 30, 30)
                ))
                .addPath(new BezierCurve(
                        new Pose(30, 30, 30),
                        new Pose(40, 40, 40)
                ))
                .setLinearHeadingInterpolation(20, 40)
                .build();
    }

    public void blueClose() {
        moveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(0, 0, 0),
                        new Pose(10, 10, 10)
                ))
                .addPath(new BezierCurve(
                        new Pose(10, 10, 10),
                        new Pose(20, 20, 20)
                ))
                .setLinearHeadingInterpolation(0, 20)
                .build();

        moveToGrab = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(20, 20, 20),
                        new Pose(30, 30, 30)
                ))
                .addPath(new BezierCurve(
                        new Pose(30, 30, 30),
                        new Pose(40, 40, 40)
                ))
                .setLinearHeadingInterpolation(20, 40)
                .build();
    }

    public void blueFar() {
        moveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(0, 0, 0),
                        new Pose(10, 10, 10)
                ))
                .addPath(new BezierCurve(
                        new Pose(10, 10, 10),
                        new Pose(20, 20, 20)
                ))
                .setLinearHeadingInterpolation(0, 20)
                .build();

        moveToGrab = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(20, 20, 20),
                        new Pose(30, 30, 30)
                ))
                .addPath(new BezierCurve(
                        new Pose(30, 30, 30),
                        new Pose(40, 40, 40)
                ))
                .setLinearHeadingInterpolation(20, 40)
                .build();
    }

    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, new Pose(72,72));
        telemetry.addLine("X for Red Close, Y for Red Far, A for Blue Close, B for Blue Far");
        if(gamepad1.x) redClose();
        if(gamepad1.y) redFar();
        if(gamepad1.a) blueClose();
        if(gamepad1.b) blueFar();

        negabot.schedule(
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, moveToShoot),
                    new AutoAim(negabot.vision,
                                negabot.shooter,
                                negabot.intake,
                                negabot.wait),
                    new FollowPathCommand(follower, moveToGrab),
                    new AutoIntake(negabot.intake,
                                   negabot.wait).autoAccept(3)
                )
        );
    }

    public void run() {
        negabot.run();
    }
}
