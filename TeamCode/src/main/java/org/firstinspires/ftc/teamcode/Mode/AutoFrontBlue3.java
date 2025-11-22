package org.firstinspires.ftc.teamcode.Mode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitTilCommand;

@Autonomous(group = "Zayan's Autos")
public class AutoFrontBlue3 extends OpMode {

    Robot negabot;
    MecanumDrive mD;

    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);
    private static final double BACK_DIST = 24.0;
    private static final double FORWARD_DIST = 30.0;
    private static final double SHOOT_RPM = 1900;

    private Pose2d firstShotPose;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, null, null);
        mD = negabot.drive.mD;

        mD.localizer.setPose(START_POSE);

        negabot.shooter.setPIDF(0.095, 0.0, 0.005, 0.54);
        negabot.shooter.setMagazineCover(0.24);
    }

    @Override
    public void start() {
        Command autoSeq = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d p = mD.localizer.getPose();

                    double heading = p.heading.toDouble();
                    double dx = -BACK_DIST * Math.cos(heading);
                    double dy = -BACK_DIST * Math.sin(heading);

                    Vector2d targetPos = new Vector2d(
                            p.position.x + dx,
                            p.position.y + dy
                    );



                    Actions.runBlocking(
                            mD.actionBuilder(p)
                                    .strafeToLinearHeading(
                                            targetPos,
                                            p.heading
                                    )
                                    .build()
                    );
                }),

                shoot(),

                new InstantCommand(() -> {
                    Pose2d p = mD.localizer.getPose();
                    Vector2d targetPos = new Vector2d(
                            p.position.x,
                            p.position.y + 20
                    );

                    Actions.runBlocking(
                            mD.actionBuilder(p)
                                    .strafeToLinearHeading(
                                            targetPos,
                                            p.heading
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
                new WaitTilCommand(negabot.shooter, SHOOT_RPM, 100),
                new AutoIntake(negabot.intake, negabot.wait).acceptSlowish(),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.03)),
                new WaitCommand(negabot.wait, 3),
                new AutoIntake(negabot.intake, negabot.wait).finish(),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.24)),
                new InstantCommand(() -> negabot.shooter.setVelocity(0))
        );
    }
}
