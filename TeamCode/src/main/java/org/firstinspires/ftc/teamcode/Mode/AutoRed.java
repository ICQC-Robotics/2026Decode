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

@Autonomous(group = ".")
public class AutoRed extends OpMode {
    Robot negabot;
    MecanumDrive mD;


    private static final Pose2d START_POSE = new Pose2d(0.0, 0.0, 0.0);

    private static final double FIRST_SHOT_HEADING_DEG = 65;

    private static final Pose2d FIRST_SHOT_POSE = new Pose2d(
            0.0,
            10,
            Math.toRadians(FIRST_SHOT_HEADING_DEG)
    );

    private static final Pose2d STACK_PRE_POSE = new Pose2d(
            10,
            37,
            Math.toRadians(180)

    );

    private static final Pose2d STACK_INTAKE_POSE = new Pose2d(
            45.0,
            37,
            Math.toRadians(180)

    );

    private static final double SHOOT_RPM = 3150;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, null, null);
        mD = negabot.drive.mD;

        mD.localizer.setPose(START_POSE);

        negabot.shooter.setPIDF(0.095, 0.0, 0.005, 0.57);
        negabot.shooter.setMagazineCover(0.24);
    }

    @Override
    public void start() {
        Command autoSeq = new SequentialCommandGroup(

                //1
                new InstantCommand(() -> {
                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .strafeToLinearHeading(
                                            new Vector2d(
                                                    FIRST_SHOT_POSE.position.x,
                                                    FIRST_SHOT_POSE.position.y
                                            ),
                                            FIRST_SHOT_POSE.heading
                                    )
                                    .build()
                    );
                }),

                //2
                shoot(),

                //3
                new InstantCommand(() -> {
                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .strafeToLinearHeading(
                                            new Vector2d(
                                                    STACK_PRE_POSE.position.x,
                                                    STACK_PRE_POSE.position.y
                                            ),
                                            STACK_PRE_POSE.heading
                                    )
                                    .build()
                    );
                }),

                //3
                new InstantCommand(() -> {
                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .strafeToLinearHeading(
                                            new Vector2d(
                                                    STACK_PRE_POSE.position.x,
                                                    STACK_PRE_POSE.position.y
                                            ),
                                            STACK_PRE_POSE.heading
                                    )
                                    .build()
                    );
                }),

                //4
                new SequentialCommandGroup(
                        new AutoIntake(negabot.intake, negabot.wait).accept(),

                        new InstantCommand(() -> {
                            Actions.runBlocking(
                                    mD.actionBuilder(mD.localizer.getPose())
                                            .strafeToLinearHeading(
                                                    new Vector2d(
                                                            STACK_INTAKE_POSE.position.x,
                                                            STACK_INTAKE_POSE.position.y
                                                    ),
                                                    STACK_INTAKE_POSE.heading
                                            )
                                            .build()
                            );
                        }),

                        new WaitCommand(negabot.wait, .5),
                        new AutoIntake(negabot.intake, negabot.wait).finish()
                ),

                //5
                new InstantCommand(() -> {
                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .strafeToLinearHeading(
                                            new Vector2d(
                                                    FIRST_SHOT_POSE.position.x,
                                                    FIRST_SHOT_POSE.position.y
                                            ),
                                            Math.toRadians(85)
                                    )
                                    .build()
                    );
                }),

                //6
                shoot()
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
                new AutoIntake(negabot.intake, negabot.wait).acceptSlow(),
                shootHelper(),
                shootHelper(),
                shootHelper(),
                new AutoIntake(negabot.intake, negabot.wait).finish(),
                new InstantCommand(() -> negabot.shooter.setVelocity(0))
        );
    }
    private Command shootHelper() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_RPM)),
                new WaitTilCommand(negabot.shooter, SHOOT_RPM, 100), //Last num is tolerance
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.03)),
                new WaitCommand(negabot.wait, 0.5), //Might Need to Tune
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.24))
        );
    }
}

