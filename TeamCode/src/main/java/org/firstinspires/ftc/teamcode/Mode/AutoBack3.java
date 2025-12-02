package org.firstinspires.ftc.teamcode.Mode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitTilCommand;

@Autonomous(group = "Zayan's Autons")
public class AutoBack3 extends OpMode {
    Robot negabot;
    MecanumDrive mD;


    private static final Pose2d START_POSE = new Pose2d(0.0, 0.0, 0.0);
    private static final Pose2d FORWARD_POSE = new Pose2d(3.0, 0.0, 0.0);

    private static final Pose2d LEAVE_POSE = new Pose2d(
            15,
            0,
            Math.toRadians(0)
    );


    private static final double SHOOT_RPM = 2990;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, telemetry);
        mD = negabot.drive.mD;

        mD.localizer.setPose(START_POSE);

        negabot.shooter.setPIDF(0.095, 0.0, 0.005, 0.57);
        negabot.shooter.setMagazineCover(0.24);
    }

    @Override
    public void start() {
        Command autoSeq = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .strafeToLinearHeading(
                                            new Vector2d(
                                                    FORWARD_POSE.position.x,
                                                    FORWARD_POSE.position.y
                                            ),
                                            LEAVE_POSE.heading
                                    )
                                    .build()
                    );
                }),

                shoot(),
                new InstantCommand(() -> {
                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .strafeToLinearHeading(
                                            new Vector2d(
                                                    LEAVE_POSE.position.x,
                                                    LEAVE_POSE.position.y
                                            ),
                                            LEAVE_POSE.heading
                                    )
                                    .build()
                    );
                })
        );

        negabot.schedule(autoSeq);
    }

    @Override
    public void loop() {
        negabot.run();
        telemetry.addData("Shooter Target RPM", SHOOT_RPM);
        telemetry.addData("Shooter Actual RPM", negabot.shooter.getVelocity());
        telemetry.update();
    }

    private Command shoot() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_RPM)),
                new WaitTilCommand(negabot.shooter, SHOOT_RPM, 50),
                new AutoIntake(negabot.intake, negabot.wait).acceptSlow(),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.03)),
                new WaitCommand(negabot.wait, 5),
                new AutoIntake(negabot.intake, negabot.wait).finish(),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.24)),
                new InstantCommand(() -> negabot.shooter.setVelocity(0))
        );
    }
}

