package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.subsystems.*;

public class Robot {
    public Drive drive;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Wait wait;
    public Vision vision;


    public Indicator indicator;
    public static Pose LAST_POSE;
    public static double LAST_TURRET_DEG;
    Telemetry t;

    public enum Alliance { BLUE, RED }
    public static Alliance ALLIANCE;

    public Robot(HardwareMap h, Telemetry t, Pose startPose) {
        this(h, t, startPose, true);
    }

    /**
     * @param resetTurret false in teleop so the turret encoder is NOT zeroed at
     *        construction and the angle saved at the end of auto can be restored
     *        (see {@link #restoreFromAuto()}). true (default) zeroes it for auto.
     */
    public Robot(HardwareMap h, Telemetry t, Pose startPose, boolean resetTurret) {
        this.t = t;
        drive = new Drive(
                h, t, startPose,
                h.get(DcMotorEx.class, "fR"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "fL"), DcMotorSimple.Direction.REVERSE,
                h.get(DcMotorEx.class, "bR"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "bL"), DcMotorSimple.Direction.REVERSE
        );

        intake = new Intake(
                h.get(DcMotorEx.class, "intake"),
                h.get(Servo.class, "servo3"),
                DcMotorSimple.Direction.FORWARD
        );

        shooter = new Shooter(
                h.get(DcMotorEx.class, "shooter1"), DcMotorSimple.Direction.REVERSE,
                h.get(DcMotorEx.class, "shooter2"), DcMotorSimple.Direction.REVERSE,
                h.get(Servo.class, "servo1"),
                h.get(Servo.class, "servo0")
        );

        turret = new Turret(
                h.get(DcMotorEx.class, "turret"),
                DcMotorSimple.Direction.REVERSE,
                new PIDFCoefficients(35, 0, 0, 0),
                resetTurret
        );

        vision = new Vision(
                h.get(Limelight3A.class, "ll"),
                h, "webcam"
        );

        wait = new Wait();
    }

    /**
     * Restore the drive pose and turret angle saved at the end of auto. Call once at the
     * start of teleop, after constructing the Robot with resetTurret = false.
     *
     * <p>On carryover the turret encoder is NOT reset — {@link Turret#setCurrentAngleDeg}
     * re-anchors the saved angle onto the existing count. If no auto pose was saved (auto
     * never ran), the robot is placed at (0,0,0) and the turret is homed, which assumes it
     * is physically at FORWARD_DEG.
     *
     * @return true if a saved auto pose was restored, false if a fresh start was used.
     */
    public boolean restoreFromAuto() {
        if (LAST_POSE != null) {
            drive.follower.setPose(LAST_POSE.copy());
            turret.setCurrentAngleDeg(LAST_TURRET_DEG);   // restore turret without resetting the encoder
            return true;
        }
        drive.follower.setPose(new Pose(0, 0, 0));
        turret.resetEncoder();
        return false;
    }

    public void Action(GamepadEx g, GamepadKeys.Button b, Command Press, Command Release) {
        if(Release == null) {
            new GamepadButton(g, b).whenPressed(Press);
        } else {
            new GamepadButton(g, b).whenPressed(Press).whenReleased(Release);
        }
    }

    public void schedule(Command... command) {
        CommandScheduler.getInstance().schedule(command);
    }

    public void run() {
        CommandScheduler.getInstance().run();
        t.update();
    }

    public void reset() {
        CommandScheduler.getInstance().reset();
    }
}
