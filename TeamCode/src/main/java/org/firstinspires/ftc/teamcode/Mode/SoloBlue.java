package org.firstinspires.ftc.teamcode.Mode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.TurretTracking;
import org.firstinspires.ftc.teamcode.Robot.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;

@TeleOp(group=".")
public class SoloBlue extends CommandOpMode {
    GamepadEx g;
    Robot negabot;

    private boolean shooterStandby = false;
    private boolean turretTracking = false;

    private boolean poseLocked = false;

    private Pose blueReset = new Pose(32.54237288135593, 134.56271186440677, Math.toRadians(90));

    private Robot.Alliance alliance = Robot.Alliance.BLUE;

    // make this a field so your DPAD buttons can access it
    private TurretTracking ppTracking;

    @Override
    public void initialize() {
        g = new GamepadEx(gamepad1);
        GamepadEx g2 = new GamepadEx(gamepad2);
        // resetTurret = false: keep the encoder count from auto so restoreFromAuto() can
        // re-anchor the saved turret angle instead of zeroing it.
        negabot = new Robot(hardwareMap, telemetry, new Pose(0,0, 0), false);
        negabot.reset();

        negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
        negabot.shooter.setHoodPosition(0.25);

        Robot.ALLIANCE = Robot.Alliance.BLUE;

        // Restore the pose + turret angle saved at the end of auto (no encoder reset).
        poseLocked = negabot.restoreFromAuto();

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

        // Gamepad 2: standby pre-spin zone — RB = FAR, LB = CLOSE
        negabot.Action(g2, GamepadKeys.Button.RIGHT_BUMPER,
                new InstantCommand(() -> negabot.shooter.setStandbyZone(Shooter.StandbyZone.FAR)), null);
        negabot.Action(g2, GamepadKeys.Button.LEFT_BUMPER,
                new InstantCommand(() -> negabot.shooter.setStandbyZone(Shooter.StandbyZone.CLOSE)), null);

        /*

        negabot.Action(
                g,
                GamepadKeys.Button.X,
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                }),
                null
        );
*/
        negabot.Action(
                g,
                GamepadKeys.Button.Y,
                new InstantCommand(() -> {
                    Pose corner = blueReset;
                    negabot.drive.follower.setPose(corner);
                    poseLocked = true;
                    ppTracking.resetDegOffset();
                }),

                null
        );

        // create TurretTracking ONCE (do not set as default)
        ppTracking = new TurretTracking(negabot.turret, negabot.drive, alliance);
        ppTracking.resetDegOffset();

        // Right trigger: shoot. ShootCommand only requires intake so the drive,
        // turret-tracking (TurretTracking), and shooter standby keep running — the
        // driver can still translate freely while the sequence waits for ready.
        new Trigger(() -> g.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new ShootCommand(
                        negabot.shooter,
                        negabot.turret,
                        negabot.intake
                ));


        // Keep your DPAD offset buttons here exactly like before
        negabot.Action(
                g,
                GamepadKeys.Button.DPAD_RIGHT,
                new InstantCommand(() -> {
                    ppTracking.incDeg();  // adds +1 degree
                    telemetry.addData("Offset", ppTracking.offset);
                    telemetry.update();
                }),
                null
        );

        negabot.Action(
                g,
                GamepadKeys.Button.DPAD_LEFT,
                new InstantCommand(() -> {
                    ppTracking.decDeg();  // subtracts 1 degree
                    telemetry.addData("Offset", ppTracking.offset);
                    telemetry.update();
                }),
                null
        );
    }

    //this is so then these default commands are activated on run
    public void run() {

        if (!shooterStandby && opModeIsActive() && !turretTracking) {
            negabot.shooter.setDefaultCommand(new ShooterStandBy(negabot.shooter, negabot.drive));
            negabot.turret.setDefaultCommand(ppTracking);
            turretTracking = true;
            shooterStandby = true;
        }

        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        negabot.run();
    }
}
