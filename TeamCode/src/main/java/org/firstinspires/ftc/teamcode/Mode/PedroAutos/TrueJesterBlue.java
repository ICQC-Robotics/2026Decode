package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
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
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

@Autonomous
public class TrueJesterBlue extends CommandOpMode {
    Robot negabot;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;
    Pose startPose = new Pose(55, 10, Math.toRadians(0));
    final double COVER_OPEN = 0.1;
    final double COVER_CLOSE = 1.0;


    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, startPose);

        Drive d = negabot.drive;
        Follower f = d.follower;
        Turret t = negabot.turret;
        Intake intake = negabot.intake;

        t.resetEncoder();

        path(f);

        waitForStart();
        negabot.schedule(
                new InstantCommand(() -> { negabot.shooter.setVelocity(4700);}),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new InstantCommand(() -> {negabot.shooter.setHoodPos(0.06); }),
                new SequentialCommandGroup(
                        new InstantCommand(() -> {negabot.turret.setTargetDeg(15); }),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN); }),
                        new WaitCommand(5000),
                        shoot()
                       // new FollowPathCommand(f, Path1, true)
                )
        );

    }

    public Command shoot(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.intake.setSpeed(-1);} ),
                new InstantCommand(() -> {negabot.shooter.setHoodPos(0.01); }),
                new WaitCommand(500),
                new InstantCommand(() -> {negabot.intake.setSpeed(0);} ),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} )
        );
    }

    public void path(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.992, 7.173),
                                new Pose(68.942, 38.521),
                                new Pose(14.384, 35.545)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}
