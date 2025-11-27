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

@Autonomous(name = "Close Red Auto", group = "Autos")
public class CloseRedAuto extends OpMode {

    Robot goonbot;
    MecanumDrive mD;

    private static final Pose2d START_POSE = new Pose2d(0, 0, Math.toRadians(0));

    private static final double DROP_PURPLE_FORWARD = 20;
    private static final double MOVE_TO_BACKDROP_X = 48;
    private static final double MOVE_TO_BACKDROP_Y = -24;

    private static final double SHOOT_RPM = 1900;

    @Override
    public void init() {
        goonbot = new Robot(hardwareMap, null, null);
        mD = goonbot.drive.mD;

        mD.localizer.setPose(START_POSE);

        goonbot.shooter.setPIDF(0.095, 0, 0.005, 0.54);
        goonbot.shooter.setMagazineCover(0.24);
    }

    @Override
    public void start() {

        Command auto = new SequentialCommandGroup(

                new InstantCommand(() -> {
                    Pose2d p = mD.localizer.getPose();
                    Vector2d target = new Vector2d(
                            p.position.x + DROP_PURPLE_FORWARD,
                            p.position.y
                    );

                    Actions.runBlocking(
                            mD.actionBuilder(p)
                                    .strafeToLinearHeading(target, p.heading)
                                    .build()
                    );
                }),

                new AutoIntake(goonbot.intake, goonbot.wait).dropPurple(),

                new InstantCommand(() -> {
                    Pose2d p = mD.localizer.getPose();
                    Actions.runBlocking(
                            mD.actionBuilder(p)
                                    .strafeToLinearHeading(
                                            new Vector2d(MOVE_TO_BACKDROP_X, MOVE_TO_BACKDROP_Y),
                                            Math.toRadians(180)
                                    )
                                    .build()
                    );
                }),

                shoot(),

                new InstantCommand(() -> {
                    Pose2d p = mD.localizer.getPose();
                    Vector2d park = new Vector2d(
                            p.position.x,
                            p.position.y - 12
                    );

                    Actions.runBlocking(
                            mD.actionBuilder(p)
                                    .strafeToLinearHeading(park, p.heading)
                                    .build()
                    );
                })
        );

        goonbot.schedule(auto);
    }

    @Override
    public void loop() {
        goonbot.run();
        telemetry.addData("Shooter Target RPM", SHOOT_RPM);
        telemetry.addData("Shooter Actual RPM", goonbot.shooter.getVelocity());
        telemetry.update();
    }

    private Command shoot() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> goonbot.shooter.setVelocity(SHOOT_RPM)),
                new WaitTilCommand(goonbot.shooter, SHOOT_RPM, 100),
                new AutoIntake(goonbot.intake, goonbot.wait).acceptSlowish(),
                new InstantCommand(() -> goonbot.shooter.setMagazineCover(0.03)),
                new WaitCommand(goonbot.wait, 3),
                new AutoIntake(goonbot.intake, goonbot.wait).finish(),
                new InstantCommand(() -> goonbot.shooter.setMagazineCover(0.24)),
                new InstantCommand(() -> goonbot.shooter.setVelocity(0))
        );
    }
}
