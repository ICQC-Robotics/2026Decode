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

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitCommand;

@Autonomous(group=".")
public class AutoRed extends OpMode {
    Robot negabot;
    MecanumDrive mD;

    private static final double STRAFE_IN = 10.0;
    private static final double TURN_DEG_RIGHT = -80;
    private static final double BURST_TIME_S = 5;
    private static final double SHOOT_RPM = 3150;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, null, null, true);
        mD = negabot.drive.mD;

        negabot.shooter.setMagazineCover(0.24);
        negabot.shooter.setPIDF(0.05, 0.0, 0.0, 0.75);
    }

    @Override
    public void start() {
        Command autoSeq = new SequentialCommandGroup(
                // 1) strafe right
                new InstantCommand(() -> {
                    Pose2d p = mD.localizer.getPose();
                    Actions.runBlocking(
                            mD.actionBuilder(p)
                                    .strafeToLinearHeading(
                                            p.position.plus(new Vector2d(0.0, STRAFE_IN)),
                                            p.heading
                                    )
                                    .build()
                    );
                }),

                // 2) turn right
                new InstantCommand(() -> {
                    Pose2d p = mD.localizer.getPose();
                    Actions.runBlocking(
                            mD.actionBuilder(p)
                                    .turn(Math.toRadians(TURN_DEG_RIGHT))
                                    .build()
                    );
                }),

                // 3) spin shooter
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            negabot.shooter.setPIDF(0.05, 0.0, 0.0, 0.58);
                            negabot.shooter.setVelocity(SHOOT_RPM);
                        }),
                        new SequentialCommandGroup(
                                new WaitCommand(negabot.wait, 5),

                                // then open the cover and start feeding
                                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.03)),
                                new WaitCommand(negabot.wait, 1),
                                new AutoIntake(negabot.intake, negabot.shooter).acceptSlow(),
                                new WaitCommand(negabot.wait, 5),
                                new AutoIntake(negabot.intake, negabot.shooter).stopShoot(negabot.wait),
                                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.24))
                        )
                )
        );

        negabot.schedule(autoSeq);
    }

    @Override
    public void loop() {
        negabot.run();
    }
}
