package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoTracking;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

@TeleOp(group=".")
public class TurretTrackingTest extends CommandOpMode {

    private GamepadEx g;
    private Robot negabot;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, telemetry);
        negabot.turret.holdCurrentAngle();

        //auto tracking
        negabot.Action(
                g,
                GamepadKeys.Button.A,
                new AutoTracking(negabot.turret, negabot.vision),
                null
        );

        //zero the turret
        negabot.Action(
                g,
                GamepadKeys.Button.X,
                new InstantCommand(() -> negabot.turret.setCurrentAsZero()),
                null
        );

        //cancel (just stop the code idk if this is actually effective)
        negabot.Action(
                g,
                GamepadKeys.Button.Y,
                new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()),
                null
        );

        //stop tracking and hold angle
        negabot.Action(
                g,
                GamepadKeys.Button.B,
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                    negabot.turret.holdCurrentAngle();
                }),
                null
        );

        //set to blue
        negabot.Action(
                g,
                GamepadKeys.Button.DPAD_LEFT,
                new InstantCommand(() -> negabot.vision.setAlliance(Vision.Alliance.BLUE)),
                null
        );

        //set to red
        negabot.Action(
                g,
                GamepadKeys.Button.DPAD_RIGHT,
                new InstantCommand(() -> negabot.vision.setAlliance(Vision.Alliance.RED)),
                null
        );
    }

    @Override
    public void run() {
        negabot.run();

        telemetry.addData("Alliance", negabot.vision.getAlliance());
        telemetry.addData("Has Target", negabot.vision.hasTarget());
        telemetry.addData("Tag ID", negabot.vision.getCurrentTagID());
        telemetry.addData("Tx", negabot.vision.getTx());
        telemetry.addData("Turret Target (deg)", negabot.turret.getTargetDeg());
        telemetry.addData("Turret Angle (deg)", negabot.turret.getAngleDeg());
        telemetry.update();
    }
}
