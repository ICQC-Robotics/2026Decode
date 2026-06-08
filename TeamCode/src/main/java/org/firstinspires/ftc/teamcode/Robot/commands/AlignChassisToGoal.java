package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;

/**
 * The turret can only swing within ~135 deg of the chassis's forward direction -
 * a 90 deg dead zone directly behind the robot is physically unreachable (cable
 * routing, see Turret.MIN_DEG/MAX_DEG). When the goal falls in that dead zone, no
 * amount of turret motion will aim at it, so PPTracking pins the turret at the
 * nearest edge and shots go nowhere near the target.
 *
 * Run this right before a shot: it spins the chassis the short way until the goal
 * is back within the turret's reachable arc, then hands off immediately. If the
 * goal is already reachable it finishes instantly, so normal shots aren't delayed.
 * Driver translation passes through untouched - only rotation is taken over.
 */
public class AlignChassisToGoal extends CommandBase {

    private static final double REACHABLE_MARGIN_DEG = 120.0;
    private static final double ALIGN_TURN_POWER = 0.45;

    private final Drive drive;
    private final GamepadEx gamepad;
    private final Robot.Alliance alliance;

    public AlignChassisToGoal(Drive drive, GamepadEx gamepad, Robot.Alliance alliance) {
        this.drive = drive;
        this.gamepad = gamepad;
        this.alliance = alliance;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double deflectionDeg = deflectionDeg();

        // Positive deflection = goal is to the left of heading, which requires
        // heading to increase (CCW) to close the gap. Positive chassis turn power
        // spins the robot CW (heading decreases - same mapping the driver's right
        // stick uses), so we turn opposite the sign of the deflection.
        double turnPower = (deflectionDeg > 0 ? -1 : 1) * ALIGN_TURN_POWER;

        double forward = -gamepad.getLeftY();
        double strafe = gamepad.getLeftX();
        drive.driveRobotCentric(forward, strafe, turnPower);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(deflectionDeg()) <= REACHABLE_MARGIN_DEG;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private double deflectionDeg() {
        Pose robot = drive.follower.getPose();
        Pose goal = FieldConstants.goalAimPointForAlliance(alliance == Robot.Alliance.BLUE);

        double bearingDeg = Math.toDegrees(Math.atan2(goal.getY() - robot.getY(), goal.getX() - robot.getX()));
        double headingDeg = Math.toDegrees(robot.getHeading());
        return wrap180(bearingDeg - headingDeg);
    }

    private static double wrap180(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }
}
