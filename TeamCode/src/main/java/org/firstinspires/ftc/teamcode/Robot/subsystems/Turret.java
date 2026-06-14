package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Turret extends SubsystemBase {

    // 0° = full left, 135° = forward, 270° = full right. Increasing encoder = CW.
    // Encoder is zeroed with the turret physically at FORWARD_DEG before each match.
    public static final double MIN_DEG = 0.0;
    public static final double MAX_DEG = 270.0;
    public static final double FORWARD_DEG = 135.0;

    // Turret pivot offset from robot center, in inches. Dashboard-tunable.
    //   FORWARD_OFFSET_IN: + toward the front of the robot (along heading)
    //   LATERAL_OFFSET_IN: + to the robot's LEFT
    public static double FORWARD_OFFSET_IN = 3.0;
    public static double LATERAL_OFFSET_IN = 0.0;

    private static final double GEAR_RATIO    = 2.77272727;
    private static final double TICKS_PER_REV = 384.5;
    public  static final double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0; // ≈ 2.96

    private final DcMotorEx motor;
    private double targetDeg = FORWARD_DEG;
    // Angle the current encoder zero represents. Normally FORWARD_DEG; changed by
    // setCurrentAngleDeg() to restore the turret angle across auto -> teleop.
    private double angleOffsetDeg = FORWARD_DEG;

    // setPower(1.0) is sent once per enable cycle. Calling it on every loop in
    // RUN_TO_POSITION mode briefly re-initialises the REV Hub PIDF, causing the
    // random bidirectional jitter we want to eliminate.
    private boolean powered = false;

    public Turret(DcMotorEx motor, DcMotorSimple.Direction direction, PIDFCoefficients pidf) {
        this(motor, direction, pidf, true);
    }

    /**
     * @param resetEncoder true to zero the encoder at construction — the turret must be
     *        physically at FORWARD_DEG (used by auto). false to keep the existing encoder
     *        count so an angle saved at the end of auto survives into teleop; the caller
     *        re-anchors the reference afterwards with {@link #setCurrentAngleDeg(double)}.
     */
    public Turret(DcMotorEx motor, DcMotorSimple.Direction direction, PIDFCoefficients pidf,
                  boolean resetEncoder) {
        this.motor = motor;

        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (resetEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
        } else {
            // Preserve the carried-over count; hold the current position until
            // setCurrentAngleDeg()/setTargetDeg() re-anchors the angle reference.
            motor.setTargetPosition(motor.getCurrentPosition());
        }
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
    }

    // ── targeting ──────────────────────────────────────────────────────────────

    /** Set the desired turret angle. Values outside [MIN_DEG, MAX_DEG] are clamped. */
    public void setTargetDeg(double deg) {
        targetDeg = clampDeg(deg);
        motor.setTargetPosition((int) Math.round((targetDeg - angleOffsetDeg) * TICKS_PER_DEG));
        if (!powered) {
            motor.setPower(1.0);
            powered = true;
        }
    }

    public void holdCurrentAngle() {
        setTargetDeg(getAngleDeg());
    }

    // ── state queries ──────────────────────────────────────────────────────────

    public double getAngleDeg() {
        return motor.getCurrentPosition() / TICKS_PER_DEG + angleOffsetDeg;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public boolean atTarget(double toleranceDeg) {
        return Math.abs(targetDeg - getAngleDeg()) <= toleranceDeg;
    }

    public int getMotorTicks() {
        return motor.getCurrentPosition();
    }

    // ── calibration ────────────────────────────────────────────────────────────

    /** Re-home the encoder. Only call this when the turret is physically at FORWARD_DEG. */
    public void resetEncoder() {
        powered = false;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);
        targetDeg = FORWARD_DEG;
        angleOffsetDeg = FORWARD_DEG;
    }

    /** Restore a known turret angle (e.g. saved from auto) without moving the turret. */
    public void setCurrentAngleDeg(double deg) {
        angleOffsetDeg = deg - motor.getCurrentPosition() / TICKS_PER_DEG;
        setTargetDeg(deg);
    }

    public void setPIDF(double p, double i, double d, double f) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(p, i, d, f));
        // 1° ≈ 3 ticks. Math.round avoids the (int) truncation that gives 2 ticks instead.
        motor.setTargetPositionTolerance((int) Math.round(TICKS_PER_DEG));
    }

    // ── zone-based aim correction ──────────────────────────────────────────────
    // When the robot is inside the rectangle below AND its heading falls in
    // [ZONE_HEADING_MIN_DEG, ZONE_HEADING_MAX_DEG], ZONE_OFFSET_DEG is added to
    // the turret target.  All values are defined in the BLUE alliance frame.
    // For RED the rectangle is mirrored about X = 72 and the heading is mirrored
    // about the vertical axis (θ → 180°−θ); the offset sign is also negated.
    // Set ZONE_OFFSET_DEG = 0 to disable.  Tune everything else on the field.
    public static double ZONE_X_MIN           =  21.5;   // field inches, blue frame
    public static double ZONE_X_MAX           = 120;
    public static double ZONE_Y_MIN           =  0.0;
    public static double ZONE_Y_MAX           = 72;
    public static double ZONE_HEADING_MIN_DEG = 100;  // robot heading range that triggers
    public static double ZONE_HEADING_MAX_DEG = 360;  //   (Pedro convention, −180 to 180)
    public static double ZONE_OFFSET_DEG      =  7;   // degrees added when in zone (+ = CW)

    // ── geometry ───────────────────────────────────────────────────────────────

    /**
     * Clamps a turret angle to [MIN_DEG, MAX_DEG].
     * Values outside the range snap to whichever endpoint is closest (short circular arc).
     * Public so TurretTracking (and callers that compute raw angles) can use it.
     */
    public static double clampDeg(double deg) {
        deg = ((deg % 360.0) + 360.0) % 360.0;   // normalise to [0, 360)
        if (deg <= MAX_DEG) return deg;            // in [0°, 270°] — in range
        // deg is in (270°, 360°): closer to MIN (0°) or MAX (270°)?
        return (360.0 - deg) <= (deg - MAX_DEG) ? MIN_DEG : MAX_DEG;
    }

    @Override
    public void periodic() {}
}
