package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class ArtifactSeekCommand extends CommandBase {

    private final Robot robot;
    private Pose lastTargetPose;
    private long lastReplanTime;

    private static final double REPLAN_DISTANCE = 3.0; // inches
    private static final long REPLAN_MS = 250;

    public ArtifactSeekCommand(Robot robot) {
        this.robot = robot;
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        lastTargetPose = null;
        lastReplanTime = 0;
    }

    @Override
    public void execute() {
        Pose robotPose =
                robot.drive.follower.getPose();

        Pose artifactPose =
                robot.vision.getArtifactFieldPose(robotPose);

        if (artifactPose == null)
            return;

        long now = System.currentTimeMillis();

        boolean shouldReplan = false;

        if (lastTargetPose == null) {

            shouldReplan = true;

        } else {

            double dx =
                    artifactPose.getX() - lastTargetPose.getX();

            double dy =
                    artifactPose.getY() - lastTargetPose.getY();

            double targetShift = Math.hypot(dx, dy);

            if (targetShift > REPLAN_DISTANCE
                    || now - lastReplanTime > REPLAN_MS) {

                shouldReplan = true;
            }
        }

        if (shouldReplan) {

            lastTargetPose = artifactPose;
            lastReplanTime = now;

            robot.drive.follower.breakFollowing();

            robot.drive.follower.followPath(

                    robot.drive.follower.pathBuilder()

                            .addPath(
                                    new BezierLine(
                                            robotPose,
                                            artifactPose
                                    )
                            )

                            .setLinearHeadingInterpolation(
                                    robotPose.getHeading(),
                                    robotPose.getHeading()
                            )

                            .build(),

                    0.7,
                    true
            );
        }
    }

    @Override
    public boolean isFinished() {
        Pose robotPose = robot.drive.follower.getPose();

        Pose artifactPose = robot.vision.getArtifactFieldPose(robotPose);

        if (artifactPose == null)
            return false;

        double distance =
                Math.hypot(
                        artifactPose.getX() - robotPose.getX(),
                        artifactPose.getY() - robotPose.getY()
                );

        return distance < 4.0;
    }

    @Override
    public void end(boolean interrupted) {
        robot.drive.follower.breakFollowing();
        robot.drive.stop();
    }
}