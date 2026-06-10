package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Turret extends SubsystemBase {

    public static final double MIN_DEG = 0.0;
    public static final double MAX_DEG = 270.0;
    public static final double GEAR_RATIO = 2.77272727;

    private static final double TICKS_PER_REV = 384.5;
    private static final double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private final DcMotorEx motor;
    private final PIDFCoefficients pidf;

    private double angleOffsetDeg = 135.0;
    private double targetDeg = 135.0;

    public Turret(DcMotorEx motor, DcMotorSimple.Direction direction, PIDFCoefficients pidf) {
        this.motor = motor;
        this.pidf = pidf;

        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);

        setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
        setTargetDeg(clamp(getAngleDeg(), MIN_DEG, MAX_DEG));
    }

    public void setPIDF(double p, double i, double d, double f) {
        pidf.p = p; pidf.i = i; pidf.d = d; pidf.f = f;
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(p, i, d, f));
        motor.setTargetPositionTolerance((int) (1.0 * TICKS_PER_DEG));
    }

    public void setTargetDeg(double deg) {
        targetDeg = nearestReachableDeg(deg);
        motor.setTargetPosition(degToTicks(targetDeg - angleOffsetDeg));
        motor.setPower(1.0);
    }

    public double getAngleDeg() {
        return (motor.getCurrentPosition() / TICKS_PER_DEG) + angleOffsetDeg;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public boolean atTarget(double toleranceDeg) {
        return Math.abs(targetDeg - getAngleDeg()) <= toleranceDeg;
    }

    public void holdCurrentAngle() {
        setTargetDeg(getAngleDeg());
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);
        angleOffsetDeg = 135.0;
        targetDeg = 135.0;
        setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
        setTargetDeg(clamp(getAngleDeg(), MIN_DEG, MAX_DEG));
    }

    public void setCurrentAsZeroButStartAtAngleDeg(double startupAngleDeg) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);
        angleOffsetDeg = clamp(startupAngleDeg, MIN_DEG, MAX_DEG);
        targetDeg = angleOffsetDeg;
        setTargetDeg(targetDeg);
    }

    @Override
    public void periodic() {}

    // ── helpers ────────────────────────────────────────────────────────────────

    private int degToTicks(double deg) {
        return (int) Math.round(deg * TICKS_PER_DEG);
    }

    private static double nearestReachableDeg(double desired) {
        double d = wrap360(desired);
        if (d >= MIN_DEG && d <= MAX_DEG) return d;
        return circularDistanceDeg(d, MIN_DEG) <= circularDistanceDeg(d, MAX_DEG)
                ? MIN_DEG : MAX_DEG;
    }

    private static double circularDistanceDeg(double a, double b) {
        double diff = Math.abs(wrap360(a) - wrap360(b)) % 360.0;
        return Math.min(diff, 360.0 - diff);
    }

    private static double wrap360(double a) {
        a %= 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
