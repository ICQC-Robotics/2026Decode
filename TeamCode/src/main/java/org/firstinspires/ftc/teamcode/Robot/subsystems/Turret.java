package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Turret extends SubsystemBase {

    public static final double MIN_DEG = 0;
    public static final double MAX_DEG = 270;
    public static final double GEAR_RATIO = 2.9047619048;

    private final DcMotorEx turretMotor;

    private double kP = 0.02;
    private double kD = 0.001;
    private double kF = 0.05;

    private final double ticksPerDeg;

    private double targetDeg = 0;
    private double visionTxDeg = Double.NaN;
    private static final double TX_DEADBAND_DEG = 0.1;

    private double angleOffsetDeg = 135.0;

    private double lastError = 0;
    private long lastTimeNs = 0;

    private static final double MAX_POWER = 0.6;

    public Turret(DcMotorEx turretMotor,
                  DcMotorSimple.Direction direction) {

        this.turretMotor = turretMotor;

        turretMotor.setDirection(direction);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerDeg = (537.7 * GEAR_RATIO) / 360.0;

        targetDeg = clamp(getAngleDeg(), MIN_DEG, MAX_DEG);
    }

    public void setPIDF(double p, double d, double f) {
        this.kP = p;
        this.kD = d;
        this.kF = f;
    }

    public void setTargetDeg(double deg) {
        targetDeg = clamp(deg, MIN_DEG, MAX_DEG);
    }

    public void holdCurrentAngle() {
        setTargetDeg(getAngleDeg());
    }

    public void setVisionTxDeg(double txDeg) {
        visionTxDeg = txDeg;
    }

    public void clearVisionTx() {
        visionTxDeg = Double.NaN;
    }

    public double getAngleDeg() {
        return (-turretMotor.getCurrentPosition() / ticksPerDeg) + angleOffsetDeg;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public boolean atTarget(double toleranceDeg) {
        return Math.abs(targetDeg - getAngleDeg()) <= toleranceDeg;
    }

    public void setCurrentAsZeroButStartAtAngleDeg(double startupAngleDeg) {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleOffsetDeg = clamp(startupAngleDeg, MIN_DEG, MAX_DEG);
        targetDeg = angleOffsetDeg;

        lastError = 0;
        lastTimeNs = 0;
    }

    @Override
    public void periodic() {
        if (!Double.isNaN(visionTxDeg)) {
            double tx = Math.abs(visionTxDeg) < TX_DEADBAND_DEG ? 0 : visionTxDeg;
            setTargetDeg(getAngleDeg() + tx);
            visionTxDeg = Double.NaN;
        }

        double currentDeg = getAngleDeg();
        double error = targetDeg - currentDeg;

        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0 : (now - lastTimeNs) * 1e-9;
        lastTimeNs = now;

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double output =
                kP * error +
                        kD * derivative +
                        kF * Math.signum(error);

        output = Range.clip(output, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(output);
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
