package org.firstinspires.ftc.teamcode.Mode;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;

public class AutoBlue extends OpMode {
    Robot negabot = new Robot(hardwareMap, null, null, true);
    MecanumDrive mD = negabot.drive.mD;

    @Override
    public void init() {
        CommandScheduler.getInstance().schedule(
            new InstantCommand(() ->
                Actions.runBlocking(
                    mD.actionBuilder(mD.localizer.getPose())
                            .lineToYConstantHeading(10.0)
                            .build()
                )
            ),

            new AutoAim(negabot.vision,
                    negabot.shooter,
                    negabot.drive,
                    negabot.wait,
                    negabot.isBlueAlliance()
            )
        );
    }

    @Override
    public void loop() {

    }
}
