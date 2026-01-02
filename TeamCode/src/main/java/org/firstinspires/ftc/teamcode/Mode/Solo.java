package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.PPTracking;
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;

@TeleOp(group=".")
public class Solo extends CommandOpMode {
    GamepadEx g;
    Robot negabot;
    private boolean shooterStandby = false;

    private enum Alliance { BLUE, RED }
    private Alliance alliance;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0, 0));
        negabot.reset();

        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.x) alliance = Alliance.BLUE;

        Robot.ALLIANCE = (alliance == Alliance.BLUE)? Robot.Alliance.BLUE: Robot.Alliance.RED;
        //W ternary operator i had to look up how to do it thank you

        telemetry.addLine("X = blue, B = red");
        telemetry.addData("Alliance", alliance);
        telemetry.update();

        negabot.drive.follower.setPose(Robot.LAST_POSE); //to test just set a pose

        negabot.drive.setDefaultCommand(new DriveCommand(negabot.drive, g));
        negabot.turret.setDefaultCommand(new PPTracking(negabot.turret, negabot.drive));

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


        negabot.Action(g,
                       GamepadKeys.Button.A,
                       new AutoAim(negabot.vision,
                                   negabot.shooter,
                                   negabot.intake,
                                   negabot.wait
                       ),
                       null
        );

        negabot.Action(
                g,
                GamepadKeys.Button.Y,
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                }),
                null
        );


    }
    public void run() {
        if (!shooterStandby && opModeIsActive()) {
            negabot.shooter.setDefaultCommand(new ShooterStandBy(negabot.shooter));
            shooterStandby = true;
        }

        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        negabot.run();
    }
}



