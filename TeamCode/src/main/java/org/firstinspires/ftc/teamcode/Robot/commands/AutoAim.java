package org.firstinspires.ftc.teamcode.Robot.commands;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
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
    private final MecanumDrive mD;

    private final int targetTagId;

    public AutoAim(Vision vision, Shooter shooter, Intake intake, Drive drive, Wait wait, boolean isBlueAlliance) {
        mD = drive.getMecanumDrive();
        targetTagId = isBlueAlliance ? 20 : 24;

        addCommands(
                new ParallelCommandGroup(
                        intake(intake, shooter),
                        shoot(vision, shooter, wait)
                )
        );
    }

    private Command intake(Intake intake, Shooter shooter) {
        return new AutoIntake(intake, shooter).accept();
    }

    private SequentialCommandGroup shoot(Vision vision, Shooter shooter, Wait wait) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (!vision.hasTarget()) return;

                    double botX = mD.localizer.getPose().position.x;
                    double botY = mD.localizer.getPose().position.y;

                    double tagX, tagY;

                    if (targetTagId == 20) {
                        tagX = 0.0;
                        tagY = 3.6576;
                    } else if (targetTagId == 24) {
                        tagX = 3.6576;
                        tagY = 0.0;
                    } else {
                        return;
                    }

                    double dx = tagX + botX;
                    double dy = tagY - botY;

                    double desiredHeading = Math.atan2(dy, dx);

                    Actions.runBlocking(
                            mD.actionBuilder(mD.localizer.getPose())
                                    .turnTo(Math.PI + desiredHeading)
                                    .build()
                    );
                }),


                new InstantCommand(() ->
                        shooter.setMagazineCover(Positions.OPEN_COVER.getPos())
                ),

                new WaitCommand(wait, 5),

                new InstantCommand(() ->
                        shooter.setMagazineCover(Positions.CLOSED_COVER.getPos())
                )
        );
    }
}
