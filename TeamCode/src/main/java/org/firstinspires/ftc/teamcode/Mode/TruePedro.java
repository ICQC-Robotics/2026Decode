package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;

/**
* @author
 * In the name of Allah, the Most Compassionate, the Most Merciful
 * Brutal
 */

@Autonomous(group="!", name="Auto")
public class TruePedro extends CommandOpMode {
    Robot negabot;
    Follower follower = negabot.drive.follower;
    PathChain moveToShoot, moveToGrab;

    public void path() {
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
        negabot = new Robot(hardwareMap, telemetry);
        path();

        negabot.schedule(
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, moveToShoot),
                    new AutoAim(negabot.vision,
                                negabot.shooter,
                                negabot.intake,
                                negabot.drive,
                                negabot.wait),
                    new FollowPathCommand(follower, moveToGrab),
                    new AutoIntake(negabot.intake,
                                   negabot.wait)
                )
        );

    }

    public void run() {
        negabot.run();
    }
}
