package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends SubsystemBase {

    private final DcMotorEx rightShooter, leftShooter;
    public final Servo leftCover, rightCover;
    private final PIDFCoefficients pidf; // not used in bang-bang, but kept in case you want PID later

    private double targetVelocityRPM;

    // -----------------------
    // Bang-bang tuning knobs
    // -----------------------
    private boolean bangBangEnabled = true;

    // How close you allow RPM to be before switching states (prevents jitter)
    private double rpmTolerance = 10; // tune this (50-150 is common)

    // Motor powers used by bang-bang
    private double fullPower = 1.0;      // power when under speed
    private double offPower  = 0.0;      // power when over speed (set to 0.0 for pure bang-bang)

    // Optional: a small hold power can reduce drop-off oscillation (not "pure" bang-bang, but practical)
    // Example: 0.05 - 0.15
    // If you want PURE bang-bang, keep this at 0.0 (same as offPower).
    // private double holdPower = 0.08;

    public Shooter(
            DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
            DcMotorEx leftShooter, DcMotorSimple.Direction leftDir,
            Servo leftCover, Servo rightCover,
            PIDFCoefficients pidf
    ) {
        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;
        this.leftCover = leftCover;
        this.rightCover = rightCover;
        this.pidf = pidf;

        this.rightShooter.setDirection(rightDir);
        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.leftShooter.setDirection(leftDir);
        this.leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // PIDF setup is not used for bang-bang, but leaving it here doesn’t hurt
        this.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);

        this.setMagazineCover(1);     // TODO: change accordingly to new cover
        targetVelocityRPM = 0;
    }

    public double getTargetVelocity() {
        return targetVelocityRPM;
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.pidf.p = p;
        this.pidf.i = i;
        this.pidf.d = d;
        this.pidf.f = f;

        this.rightShooter.setVelocityPIDFCoefficients(p, i, d, f);
        this.leftShooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setMagazineCover(double pos) {
        leftCover.setPosition(pos);
        rightCover.setPosition(1 - pos);
    }



    /**
     * Sets target RPM for bang-bang control.
     * Bang-bang will be applied automatically in periodic().
     */
    public void setVelocity(double rpm) {
        targetVelocityRPM = rpm;

        // If target is 0, stop immediately
        if (rpm <= 0) {
            rightShooter.setPower(0);
            leftShooter.setPower(0);
        }
    }

    /** Directly stop shooter (power = 0, target = 0) */
    public void stop() {
        targetVelocityRPM = 0;
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    /** Turn bang-bang on/off */
    public void setBangBangEnabled(boolean enabled) {
        bangBangEnabled = enabled;
    }

    /** Tune bang-bang tolerance (RPM band) */
    public void setRpmTolerance(double tol) {
        rpmTolerance = tol;
    }

    /** Tune bang-bang powers */
    public void setBangBangPowers(double fullPower, double offPower) {
        this.fullPower = fullPower;
        this.offPower = offPower;
    }

    /**
     * Current velocity in RPM (from encoder)
     */
    public double getVelocity() {
        double velocityInTPS = this.rightShooter.getVelocity();
        double ticksPerRev = 28;
        return (velocityInTPS * 60.0) / ticksPerRev;
    }

    @Override
    public void periodic() {
        if (!bangBangEnabled) return;

        // If no target speed, keep it off
        if (targetVelocityRPM <= 0) {
            rightShooter.setPower(0);
            leftShooter.setPower(0);
            return;
        }

        double currentRPM = getVelocity();

        // Bang-bang with hysteresis band
        if (currentRPM < (targetVelocityRPM - rpmTolerance)) {
            // below target -> full send
            rightShooter.setPower(fullPower);
            leftShooter.setPower(fullPower);
        } else if (currentRPM > (targetVelocityRPM + rpmTolerance)) {
            // above target -> cut power
            rightShooter.setPower(offPower);
            leftShooter.setPower(offPower);
        }
        // else: inside tolerance band -> keep last power state (prevents rapid toggling)
    }
}
