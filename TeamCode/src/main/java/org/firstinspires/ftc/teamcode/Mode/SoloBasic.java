package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;

@TeleOp(group=".")
public class SoloBasic extends CommandOpMode {
    GamepadEx g;
    Robot negabot;

    private boolean shooterStandby = false;
    private double turretTargetDeg;
    private static final double TURRET_STEP_DEG = .067; //TODO: adjust if needed
    private double targetRpm = 5000;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0, 0));
        negabot.reset();
        turretTargetDeg = negabot.turret.getAngleDeg();


        negabot.drive.setDefaultCommand(new DriveCommand(negabot.drive, g));

        negabot.Action(g,
                       GamepadKeys.Button.RIGHT_BUMPER,
                       new AutoIntake(negabot.intake, negabot.wait).accept(),
                       new AutoIntake(negabot.intake, negabot.wait).finish()
        );

        negabot.Action(g,
                GamepadKeys.Button.LEFT_BUMPER,
                new AutoIntake(negabot.intake, negabot.wait).reject(),
                new AutoIntake(negabot.intake, negabot.wait).finish()
        );

        //shoot close-mid
        negabot.Action(g,
                        GamepadKeys.Button.A,

                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    targetRpm = 4000;
                                }),
                                new InstantCommand(() -> {
                                    negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                                }),
                                new WaitCommand(500),
                                new AutoIntake(negabot.intake, negabot.wait).accept(),
                                new WaitCommand(500),
                                new AutoIntake(negabot.intake, negabot.wait).finish(),
                            new InstantCommand(() -> {
                                negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                            })

                        ),
                null

        );

        //shoot far
        negabot.Action(g,
                GamepadKeys.Button.B,

                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            targetRpm = 6000;
                        }),

                        new InstantCommand(() -> {
                            negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                        }),
                        new WaitCommand(500),
                        new AutoIntake(negabot.intake, negabot.wait).accept(),
                        new WaitCommand(500),
                        new AutoIntake(negabot.intake, negabot.wait).finish(),
                        new InstantCommand(() -> {
                            negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                        })

                ),
                null

        );

        g.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(new RunCommand(() -> {
                    turretTargetDeg += TURRET_STEP_DEG;
                    negabot.turret.setTargetDeg(turretTargetDeg);
                }, negabot.turret));

        g.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new RunCommand(() -> {
                    turretTargetDeg -= TURRET_STEP_DEG;
                    negabot.turret.setTargetDeg(turretTargetDeg);
                }, negabot.turret));
    }

    public void run() {
        negabot.shooter.setVelocity(targetRpm);
        negabot.run();
    }
}



