package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Turret extends SubsystemBase {

    public static final double MIN_ANGLE_DEG = 0;
    public static final double MAX_ANGLE_DEG = 270;
    public static final double SOFT_MIN_DEG = 2;
    public static final double SOFT_MAX_DEG = 268;

    public static final double GEAR_RATIO = 2.9047619048;

    private final DcMotorEx turretMotor;
    private final PIDFCoefficients pidc;
    private final PIDController pid;

    private final double ticksPerDeg;

    private double targetAngleDeg = 135;
    private double zeroOffsetTicks;

    //TODO: tune and adjust below
    private static final double ANGLE_TOLERANCE_DEG = 1.0;
    private static final double MIN_POWER = 0.06;
    private static final double MAX_POWER = 0.67;

    public Turret(DcMotorEx turretMotor, DcMotorSimple.Direction turretDir, PIDFCoefficients pidc) {
        this.turretMotor = turretMotor;
        this.pidc = pidc;
        this.pid = new PIDController(pidc.p, pidc.i, pidc.d);

        this.turretMotor.setDirection(turretDir);
        this.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.ticksPerDeg = (this.turretMotor.getMotorType().getTicksPerRev() * GEAR_RATIO) / 360.0;

        this.pid.setTolerance(ANGLE_TOLERANCE_DEG);

        this.zeroOffsetTicks = this.turretMotor.getCurrentPosition();
        this.targetAngleDeg = getAngleDeg();
    }

    public void setPID(double p, double i, double d) {
        this.pidc.p = p;
        this.pidc.i = i;
        this.pidc.d = d;
        this.pid.setPID(p, i, d);
    }

    public void setCurrentAsZero() {
        this.zeroOffsetTicks = turretMotor.getCurrentPosition();
        this.targetAngleDeg = 0.0;
        this.pid.reset();
    }

    public double getAngleDeg() {
        double ticks = turretMotor.getCurrentPosition() - zeroOffsetTicks;
        double deg = ticks / ticksPerDeg;
        return clamp(deg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    public void setTargetAngleDeg(double targetDeg) {
        this.targetAngleDeg = clamp(targetDeg, SOFT_MIN_DEG, SOFT_MAX_DEG);
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public boolean atTarget() {
        return pid.atSetPoint();
    }

    public void aimWithTx(double txDeg) {
        if (Double.isNaN(txDeg)) {
            setTargetAngleDeg(getAngleDeg());
            return;
        }
        setTargetAngleDeg(getAngleDeg() + txDeg);
    }

    @Override
    public void periodic() {
        double current = getAngleDeg();
        double out = pid.calculate(current, targetAngleDeg);

        if (Math.abs(out) > 0 && Math.abs(out) < MIN_POWER) {
            out = Math.signum(out) * MIN_POWER;
        }

        if (out > MAX_POWER) out = MAX_POWER;
        if (out < -MAX_POWER) out = -MAX_POWER;

        if (pid.atSetPoint()) out = 0.0;

        setPowerWithLimits(out);
    }

    private void setPowerWithLimits(double power) {
        double angle = getAngleDeg();

        if (angle <= SOFT_MIN_DEG && power < 0) power = 0;
        if (angle >= SOFT_MAX_DEG && power > 0) power = 0;

        turretMotor.setPower(power);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
