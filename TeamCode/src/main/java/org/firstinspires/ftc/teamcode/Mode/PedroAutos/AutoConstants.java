package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;

import java.util.LinkedList;
import java.util.Queue;

public class AutoConstants {
    Queue<PathChain> paths = new LinkedList<>();
    Pose closeShotPose;
    Pose closeStartPose;
    Pose row1StartPose;
    Pose row1EndPose;
    Pose row2StartPose;
    Pose row2EndPose;
    Pose row3StartPose;
    Pose row3EndPose;
    Pose preGatePose;
    Pose gatePressPose;
    Pose farShotPose;
    Pose farStartPose;
    Pose loadingIntake1Pose;
    Pose loadingIntake2Pose;
    int farVelocity;
    int closeVelocity;
    final double COVER_OPEN = 0.1;
    final double COVER_CLOSE = 1.0;

    public Command shoot(Robot negabot){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.intake.setSpeed(-1);} ),
                new WaitCommand(500),
                new InstantCommand(() -> {negabot.intake.setSpeed(-1);} ),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} )
        );
    }
    public Command shotPrep(PathChain path, double turretAngle, Robot negabot, Follower f){
        return new ParallelCommandGroup(

                new FollowPathCommand(f, path, true),
                new InstantCommand(() -> {negabot.intake.setSpeed(0);} ),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);}
                        )),
                new InstantCommand(() -> { negabot.turret.setTargetDeg(turretAngle); })

        );
    }

    private static double mirrorX(double x) {
        return 144.0 - x;
    }
    private static double mirrorHeadingRad(double headingRad) {
        double out = Math.PI - headingRad;
        while (out < 0) out += 2 * Math.PI;
        while (out >= 2 * Math.PI) out -= 2 * Math.PI;
        return out;
    }
    private static double mirrorTurretDeg(double turretDeg) {
        double out = 270.0 - turretDeg;
        while (out < 0) out += 360.0;
        while (out >= 360.0) out -= 360.0;
        return out;
    }
    private static Pose mirrorPose(Pose p){
        return new Pose(mirrorX(p.getX()), p.getY(), mirrorHeadingRad(p.getHeading()));
    }
    /**
        @param f Follower to be used
        @param path Array of poses used to create the path
        @param mirror true when the path should be mirrored
        @param reverse true when the path should be reversed
     **/
    public PathChain path(Follower f, boolean mirror, boolean reverse, Pose... path){
        if(mirror){
            PathBuilder p = f.pathBuilder();
            for(int i = 0; i < path.length - 1; i++){
                p.addPath(
                        new BezierLine(
                                mirrorPose(path[i]),
                                mirrorPose(path[i + 1])
                        )
                );
            }
            p.setLinearHeadingInterpolation(mirrorHeadingRad(path[0].getHeading()), mirrorHeadingRad(path[path.length - 1].getHeading()));
            if(reverse) p.setReversed();
            return p.build();
        }
        PathBuilder p = f.pathBuilder();
        for(int i = 0; i < path.length - 1; i++){

            p.addPath(
                    new BezierLine(
                            path[i],
                            path[i + 1]
                    )
            );
        }
        p.setLinearHeadingInterpolation(path[0].getHeading(), path[path.length - 1].getHeading());
        if(reverse) p.setReversed();
        return p.build();
    }
    public boolean fixPaths(){
        Queue<PathChain> temp = new LinkedList<>();
        while(true) {
            PathChain p = paths.poll();
            temp.add(p);
            if (paths.isEmpty()) break;
        }
        return false;
    }
}
