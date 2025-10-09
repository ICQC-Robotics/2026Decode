package org.firstinspires.ftc.teamcode.Robot.commands;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoAim extends SequentialCommandGroup {
    enum Positions {
        OPEN_COVER(0),
        CLOSED_COVER(1);

        private final double pos;
        Positions(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }
    }

    private final Vision vision;
    private final Shooter shooter;
    private final Drive drive;
    private final MecanumDrive mD;
    private final Wait wait;

    private static final double MAX_VELOCITY = 2400;
    private static final double MIN_VELOCITY = 1800;

    private final int targetTagId;
    private final ElapsedTime timer = new ElapsedTime();

    public AutoAim(Vision vision, Shooter shooter, Drive drive, Wait wait, boolean isBlueAlliance) {
        this.vision = vision;
        this.shooter = shooter;
        this.drive = drive;
        this.wait = wait;
        mD = drive.getMecanumDrive();
        targetTagId = isBlueAlliance ? 20 : 24;

        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> {
                    if (!vision.hasTarget()) return;
                    double botX = vision.getBotX();
                    double botY = vision.getBotY();

                    double tagX = (targetTagId == 20) ? 0 : 3.6576;
                    double tagY = (targetTagId == 20) ? 3.6576 : 0;

                    double dx = tagX - botX;
                    double dy = tagY - botY;
                    double desiredHeading = Math.atan2(dy, dx);

                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .turnTo(desiredHeading)
                                    .build()
                    );
                }),

                new InstantCommand(() ->
                        shooter.setMagazineCover(Positions.OPEN_COVER.getPos())
                ),

                new WaitCommand(wait, 300),

                new InstantCommand(() -> {
                    double botX = vision.getBotX();
                    double botY = vision.getBotY();

                    double tagX = (targetTagId == 20) ? 0 : 144;
                    double tagY = (targetTagId == 20) ? 144 : 0;

                    double dx = tagX - botX;
                    double dy = tagY - botY;
                    double distance = Math.sqrt(dx * dx + dy * dy);

                    double velocity = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * (distance / 67.0);
                    velocity = Math.min(MAX_VELOCITY, Math.max(MIN_VELOCITY, velocity));

                    shooter.setVelocity(velocity);
                }),

                new WaitCommand(wait, 1000),

                new InstantCommand(() ->
                        shooter.setMagazineCover(Positions.CLOSED_COVER.getPos())
                )
        );
    }
}
