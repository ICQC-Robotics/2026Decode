package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends SubsystemBase {

    private final DcMotorEx rightShooter, leftShooter;
    public final Servo Cover;
    public final Servo Hood;

    // Keep pidf if you still want the motor controller's internal velocity PIDF (optional)
    private final PIDFCoefficients pidf;

    private double targetVelocityRPM;
    private double rpmTolerance = 0;

    // Bang-bang outputs
    private double fullPower = 1;
    private double offPower  = 0.0;

    // Optional: if you want it to "coast" at a low power when above target instead of fully off
    // private double holdPower = 0.0;

    // Bang-bang state (for hysteresis / keeping last output within tolerance band)
    private boolean bangHigh = false;

    public Shooter(
            DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
            DcMotorEx leftShooter, DcMotorSimple.Direction leftDir,
            Servo Cover, Servo Hood,
            PIDFCoefficients pidf
    ) {
        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;
        this.Cover = Cover;
        this.Hood = Hood;
        this.pidf = pidf;

        this.rightShooter.setDirection(rightDir);
        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        this.leftShooter.setDirection(leftDir);
        this.leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Optional: keep this if you're relying on the built-in velocity PIDF when using setVelocity(),
        // but since we're doing pure bang-bang on setPower(), this doesn't really matter.
        this.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);

        this.setMagazineCover(1);
        targetVelocityRPM = 0;
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.pidf.p = p;
        this.pidf.i = i;
        this.pidf.d = d;
        this.pidf.f = f;
        this.rightShooter.setVelocityPIDFCoefficients(p, i, d, f);
        this.leftShooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setVelocity(double rpm) {
        targetVelocityRPM = rpm;
        bangHigh = false; // reset state on new target
        if (rpm <= 0) {
            stop();
        }
    }

    public void stop() {
        targetVelocityRPM = 0;
        bangHigh = false;
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    public double getVelocity() {
        double velocityInTPS = this.rightShooter.getVelocity();
        double ticksPerRev = 28; // adjust if not a 28 CPR motor/encoder setup
        return (velocityInTPS * 60.0) / ticksPerRev;
    }

    @Override
    public void periodic() {
        if (targetVelocityRPM <= 0) return;

        double currentRPM = getVelocity();

        // Pure bang-bang with hysteresis band
        double low = targetVelocityRPM - rpmTolerance;
        double high = targetVelocityRPM + rpmTolerance;

        if (currentRPM < low) {
            bangHigh = true;   // go full
        } else if (currentRPM > high) {
            bangHigh = false;  // go off
        }
        // else: within band → keep previous bangHigh (prevents chatter)

        double out = bangHigh ? fullPower : offPower;
        rightShooter.setPower(out);
        leftShooter.setPower(out);
    }

    public void setMagazineCover(double pos) {
        Cover.setPosition(pos);
    }

    public double getTargetVelocity(){
        return targetVelocityRPM;
    }
}