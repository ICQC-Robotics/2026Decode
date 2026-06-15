package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Turret extends SubsystemBase {

    public static final double MIN_DEG = 0;
    public static final double MAX_DEG = 270;
    public static final double GEAR_RATIO = 2.77272727;

    private final DcMotorEx turretMotor;
    private final PIDFCoefficients pidf;

    private final double ticksPerDeg;

    private double targetDeg = 0;
    private double visionTxDeg = Double.NaN;
    private static final double TX_DEADBAND_DEG = 0.1;
    private double angleOffsetDeg = 135.0;

    // ── zone-based aim correction (BLUE frame) ──
    public static double ZONE_X_MIN      =  21.5;
    public static double ZONE_X_MAX      = 120;
    public static double ZONE_Y_MIN      =  0.0;
    public static double ZONE_Y_MAX      = 72;
    public static double ZONE_HEADING_MIN_DEG = 100;
    public static double ZONE_HEADING_MAX_DEG = 360;
    public static double ZONE_OFFSET_DEG =  7;   // + = CW. Set 0 to disable.

    public Turret(DcMotorEx turretMotor,
                  DcMotorSimple.Direction direction,
                  PIDFCoefficients pidf) {

        this.turretMotor = turretMotor;
        this.pidf = pidf;

        turretMotor.setDirection(direction);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.0);

        ticksPerDeg = (384.5 * GEAR_RATIO) / 360.0;

        this.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
        this.setTargetDeg(clamp(getAngleDeg(), MIN_DEG, MAX_DEG));
    }

    public void resetEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.0);

        //ticksPerDeg = (384.5 * GEAR_RATIO) / 360.0;

        this.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
        this.setTargetDeg(clamp(getAngleDeg(), MIN_DEG, MAX_DEG));
    }

    public void setCurrentAsZeroButStartAtAngleDeg(double startupAngleDeg) {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.0);

        // ticks is now 0, so angleOffsetDeg becomes the startup angle in your global system
        angleOffsetDeg = clamp(startupAngleDeg, MIN_DEG, MAX_DEG);

        targetDeg = angleOffsetDeg;
        visionTxDeg = Double.NaN;

        setTargetDeg(targetDeg);
    }



    public void assumeCurrentAngleDeg(double angleDeg) {
        double clamped = clamp(angleDeg, MIN_DEG, MAX_DEG);

        // Keep the SAME global coordinate system (135 is still forward).
        // We are only anchoring encoder ticks to that coordinate system.
        angleOffsetDeg = clamped - (turretMotor.getCurrentPosition() / ticksPerDeg);

        // Hold this angle so it doesn't jump
        targetDeg = clamped;
        turretMotor.setTargetPosition(degToTicks(targetDeg - angleOffsetDeg));
        turretMotor.setPower(1);
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.pidf.p = p;
        this.pidf.i = i;
        this.pidf.d = d;
        this.pidf.f = f;

        turretMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(p, i, d, f)
        );
        turretMotor.setTargetPositionTolerance((int) (1.0 * ticksPerDeg));
    }

    public void setVisionTxDeg(double txDeg) {
        visionTxDeg = txDeg;
    }

    public void clearVisionTx() {
        visionTxDeg = Double.NaN;
    }

    public void setTargetDeg(double deg) {
        targetDeg = clamp(deg, MIN_DEG, MAX_DEG);
        int ticks = degToTicks(targetDeg - angleOffsetDeg);
        turretMotor.setTargetPosition(ticks);
        turretMotor.setPower(1);
    }

    public void holdCurrentAngle() {
        setTargetDeg(getAngleDeg());
    }

    public void setCurrentAsZero() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.0);

        targetDeg = angleOffsetDeg;
        visionTxDeg = Double.NaN;
    }

    public double getAngleDeg() {
        return (turretMotor.getCurrentPosition() / ticksPerDeg) + angleOffsetDeg;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public boolean atTarget(double toleranceDeg) {
        return Math.abs(targetDeg - getAngleDeg()) <= toleranceDeg;
    }

    @Override
    public void periodic() {
        if (!Double.isNaN(visionTxDeg)) {

            double tx = visionTxDeg;
            if (Math.abs(tx) < TX_DEADBAND_DEG) tx = 0;

            double current = getAngleDeg();
            double desired = current + tx;

            desired = clamp(desired, MIN_DEG, MAX_DEG);
            setTargetDeg(desired);
            visionTxDeg = Double.NaN;
        }
    }

    private int degToTicks(double deg) {
        return (int) Math.round(deg * ticksPerDeg);
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    public void restoreAngleDeg(double savedAngleDeg) {
        angleOffsetDeg = savedAngleDeg - (turretMotor.getCurrentPosition() / ticksPerDeg);
        targetDeg = savedAngleDeg;
        setTargetDeg(savedAngleDeg);
    }
}