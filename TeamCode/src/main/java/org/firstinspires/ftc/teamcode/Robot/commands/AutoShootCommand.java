package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoShootCommand extends SequentialCommandGroup {

    private static final double FEED_POWER = -0.75;
    private static final double MIN_COVER_OPEN_S = 0.1;
    private static final double FEED_TIME_S = 0.6;
    private static final double RPM_TOLERANCE = 50;

    public AutoShootCommand(Intake intake, Shooter shooter, Wait wait) {
        addCommands(
                new InstantCommand(() ->
                        shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos()), shooter),

                // Wait until RPM is on-target AND the cover has had time to open
                new CommandBase() {
                    {
                        addRequirements(shooter);
                    }

                    @Override
                    public void initialize() {
                        wait.start();
                    }

                    @Override
                    public boolean isFinished() {
                        return wait.elapsed() >= MIN_COVER_OPEN_S
                                && shooter.isAtTargetVelocity(RPM_TOLERANCE);
                    }
                },

                new InstantCommand(() -> intake.setSpeed(FEED_POWER), intake),

                new CommandBase() {
                    {
                        addRequirements(shooter, intake);
                    }

                    @Override
                    public void initialize() {
                        wait.start();
                    }

                    @Override
                    public boolean isFinished() {
                        return wait.elapsed() >= FEED_TIME_S;
                    }

                    @Override
                    public void end(boolean interrupted) {
                        intake.setSpeed(0);
                    }
                },

                new InstantCommand(() -> {
                    shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                    intake.setSpeed(0);
                }, shooter, intake)
        );
        addRequirements(shooter, intake);
    }
}
