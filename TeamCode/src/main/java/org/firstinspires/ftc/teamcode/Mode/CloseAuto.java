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

@Autonomous(name = "AutoBackShoot3_1900", group = ".")
public class CloseAuto extends OpMode {

    Robot negabot;
    MecanumDrive mD;

    // Starting pose at (0,0) heading 0
    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);

    // Distance to go backward (inches)
    private static final double BACK_DIST = 24.0;

    // Shooter speed
    private static final double SHOOT_RPM = 1900;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, null, null);
        mD = negabot.drive.mD;

        // Set starting RoadRunner pose
        mD.localizer.setPose(START_POSE);

        // Default shooter config
        negabot.shooter.setPIDF(0.095, 0.0, 0.005, 0.57);
        negabot.shooter.setMagazineCover(0.24); // closed
    }

    @Override
    public void start() {
        Command autoSeq = new SequentialCommandGroup(

                // 1) Drive backward first using RoadRunner
                new InstantCommand(() -> {
                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .strafeToLinearHeading(
                                            new Vector2d(0, -BACK_DIST), // backward = negative Y
                                            0.0                           // keep heading
                                    )
                                    .build()
                    );
                }),

                // 2) Spin up shooter + shoot 3 notes
                shootThreeNotes()
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

    // Spins up shooter, fires 3, then stops shooter
    private Command shootThreeNotes() {
        return new SequentialCommandGroup(
                // Make sure intake is off before starting
                new AutoIntake(negabot.intake, negabot.wait).finish(),

                // Spin up shooter
                new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_RPM)),

                // Three individual shots
                shootOne(),
                shootOne(),
                shootOne(),

                // Stop shooter & close mag at the end
                new InstantCommand(() -> negabot.shooter.setVelocity(0)),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.24))
        );
    }

    // Fires 1 note at current RPM
    private Command shootOne() {
        return new SequentialCommandGroup(
                // Let shooter stabilize at 1900
                new WaitCommand(negabot.wait, 1.0),

                // Open mag
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.03)),

                // Start feeding
                new AutoIntake(negabot.intake, negabot.wait).acceptSlow(),

                // Time to send 1 note through – tune this
                new WaitCommand(negabot.wait, 0.7),

                // Stop feeding and close mag
                new AutoIntake(negabot.intake, negabot.wait).finish(),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(0.24))
        );
    }
}
