package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;

/**
 * Backup teleop for when the turret/auto-aim path is unavailable. No turret, no distance-based
 * hood/RPM lookup -- just a static flywheel speed, manually toggled between a close and far
 * preset (A), defaulting to close. Right trigger feeds (opens the magazine, spins the intake)
 * for as long as it's held, instead of running a fixed-duration sequence. Intake accept/reject
 * bumper bindings match SoloRed/SoloBlue.
 */
@TeleOp(name="Backup")
public class ChudTele extends CommandOpMode {
    GamepadEx g;
    Robot negabot;

    static final double CLOSE_VELOCITY = 3000;
    static final double CLOSE_HOOD     = 0.5;
    static final double FAR_VELOCITY   = 4200;
    static final double FAR_HOOD       = 0.7;
    static final double TRIGGER_THRESHOLD = 0.5;

    double flywheelVelocity = CLOSE_VELOCITY;
    double flywheelHood     = CLOSE_HOOD;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0, 0));
        negabot.reset();
        negabot.shooter.setHood(flywheelHood);
        negabot.shooter.setDefaultCommand(new RunCommand(() -> negabot.shooter.setVelocity(flywheelVelocity), negabot.shooter));
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

        // A toggles between the two close/far presets (velocity + hood together) -- no auto-aim,
        // no distance lookup.
        negabot.Action(g,
                GamepadKeys.Button.A,
                new InstantCommand(() -> {
                    boolean toFar = flywheelVelocity == CLOSE_VELOCITY;
                    flywheelVelocity = toFar ? FAR_VELOCITY : CLOSE_VELOCITY;
                    flywheelHood     = toFar ? FAR_HOOD     : CLOSE_HOOD;
                    negabot.shooter.setHood(flywheelHood);
                }),
                null
        );

        // Right trigger feeds for as long as it's held: opens the magazine and spins the intake
        // on pull, stops and closes it the instant it's released.
        new Trigger(() -> g.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD)
                .whileActiveOnce(feedWhileHeld());

        // cancelAll() must run from execute(), not initialize(): button-triggered commands are
        // initialized during button polling, before the scheduler's run-loop guard is up, so
        // cancelAll() canceling the already-scheduled default commands at that point mutates the
        // scheduler's command map while it's mid-iteration and throws ConcurrentModificationException.
        // Deferring the call to execute() lets it run inside the guarded loop, where cancellations
        // are queued and applied safely afterward.
        negabot.Action(
                g,
                GamepadKeys.Button.Y,
                new CommandBase() {
                    @Override
                    public void execute() {
                        CommandScheduler.getInstance().cancelAll();
                    }

                    @Override
                    public boolean isFinished() {
                        return true;
                    }
                },
                null
        );
    }

    private Command feedWhileHeld() {
        return new CommandBase() {
            {
                addRequirements(negabot.intake);
            }

            @Override
            public void initialize() {
                negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                negabot.intake.set(AutoIntake.Positions.LOWER_INTAKE.getPos());
                negabot.intake.setSpeed(-1);
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public void end(boolean interrupted) {
                negabot.intake.setSpeed(0);
                negabot.intake.set(AutoIntake.Positions.LOWER_INTAKE.getPos());
                negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
            }
        };
    }

    public void run() {
        telemetry.addData("Preset", flywheelVelocity == CLOSE_VELOCITY ? "CLOSE" : "FAR");
        telemetry.addData("Flywheel target", flywheelVelocity);
        telemetry.addData("Hood target", flywheelHood);
        negabot.run();
    }
}
