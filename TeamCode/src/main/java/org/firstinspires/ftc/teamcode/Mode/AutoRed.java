package org.firstinspires.ftc.teamcode.Mode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;

@Autonomous(group=".")
public class AutoRed extends OpMode {
    Robot negabot;
    MecanumDrive mD;

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, null, null, false);
        mD = negabot.drive.mD;

        Command AutoRed = new SequentialCommandGroup(
                new InstantCommand(() ->
                        Actions.runBlocking(
                                mD.actionBuilder(mD.localizer.getPose())
                                        .lineToYConstantHeading(10.0)
                                        .build()
                        )
                ),

                new AutoAim(negabot.vision,
                        negabot.shooter,
                        negabot.intake,
                        negabot.drive,
                        negabot.wait,
                        negabot.isBlueAlliance()
                )
        );

        negabot.schedule(AutoRed);
    }

    @Override
    public void loop() {
        negabot.run();
    }
}
