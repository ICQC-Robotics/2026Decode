package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoShootCommand extends SequentialCommandGroup {

    private static final double FEED_POWER = -0.75;
    private static final double COVER_WAIT_S = 1.0;
    private static final double FEED_TIME_S = 1.0;

    public AutoShootCommand(Intake intake, Shooter shooter, Wait wait) {
        addCommands(
                new InstantCommand(() ->
                        shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos()), shooter),
                new WaitCommand(wait, COVER_WAIT_S),
                new InstantCommand(() -> intake.setSpeed(FEED_POWER), intake),
                new WaitCommand(wait, FEED_TIME_S),
                new InstantCommand(() -> {
                    shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                    intake.setSpeed(0);
                }, shooter, intake)
        );
    }
}
