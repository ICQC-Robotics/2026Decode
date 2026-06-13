package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;

/**
 * Default command for the Shooter subsystem.
 *
 * Runs continuously every loop: computes distance from current pose to the
 * alliance goal, looks up the correct hood angle and flywheel RPM via the
 * ShooterAimingModel, and applies both immediately. The bang-bang controller
 * in Shooter.periodic() then drives the motors toward that velocity.
 *
 * Because this is the shooter's default command, it is automatically replaced
 * (and later restored) whenever another command explicitly requires the shooter.
 */
public class ShooterStandBy extends CommandBase {

    private final Shooter shooter;
    private final Drive drive;

    public ShooterStandBy(Shooter shooter, Drive drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double distIn = AutoAim.calculateDistanceIn(drive);
        if (shooter.isRefineActive()) {
            shooter.aimForDistance(distIn);     // exact distance while a shot is firing
        } else {
            shooter.applyStandbyPreset();        // pre-spin to the operator-selected zone
        }

        drive.telemetry.addData("Standby Zone",       shooter.getStandbyZone());
        drive.telemetry.addData("Standby Dist (in)",  distIn);
        drive.telemetry.addData("Standby Profile",    shooter.getLastProfileName());
        drive.telemetry.addData("Standby Target RPM", shooter.getTargetVelocity());
        drive.telemetry.addData("Standby Actual RPM", shooter.getVelocity());
        drive.telemetry.addData("Standby Hood",       shooter.getTargetHoodPosition());
        drive.telemetry.addData("Standby At Speed",   shooter.isAtTargetVelocity(75));
    }

    @Override
    public boolean isFinished() {
        return false; // runs forever as default
    }
}
