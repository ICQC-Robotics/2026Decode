package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter extends SubsystemBase {

    public static final double BASELINE_RPM = 2600;

    private final DcMotorEx rightShooter, leftShooter;
    public final Servo Cover;
    private final Servo Hood;
    private final ShooterAimingModel aimingModel = new ShooterAimingModel();
    private final ElapsedTime hoodSettleTimer = new ElapsedTime();

    // Keep pidf if you still want the motor controller's internal velocity PIDF (optional)
    private final PIDFCoefficients pidf;

    private double targetVelocityRPM;
    private double rpmTolerance = 25;
    private double targetHoodPosition = -1.0;
    private static final double TARGET_RPM_CHANGE_RESET = 50.0;
    private static final double HOOD_POSITION_EPSILON = 0.002;
    private static final double HOOD_SETTLE_TIME_S = 0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MIN_VALID_RPM = 1.0;
    private ShooterAimingModel.Solution lastSolution;
    private double lastDistanceIn = 0.0;

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
            Servo Cover,
            Servo Hood,
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
        this.setHoodPosition(0.2);
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
        if (rpm <= 0) {
            stop();
            return;
        }
        if (Math.abs(rpm - targetVelocityRPM) > TARGET_RPM_CHANGE_RESET) {
            bangHigh = false;
        }
        targetVelocityRPM = rpm;
    }

    public void stop() {
        targetVelocityRPM = 0;
        bangHigh = false;
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    public double getVelocity() {
        double rightRPM = Math.abs(getRightVelocity());
        double leftRPM = Math.abs(getLeftVelocity());

        if (rightRPM < MIN_VALID_RPM) return leftRPM;
        if (leftRPM < MIN_VALID_RPM) return rightRPM;
        return (rightRPM + leftRPM) / 2.0;
    }

    public double getRightVelocity() {
        return ticksPerSecondToRPM(rightShooter.getVelocity());
    }

    public double getLeftVelocity() {
        return ticksPerSecondToRPM(leftShooter.getVelocity());
    }

    public boolean isAtTargetVelocity(double toleranceRPM) {
        return targetVelocityRPM > 0
                && Math.abs(getVelocity() - targetVelocityRPM) <= toleranceRPM;
    }

    private static double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
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

    public void setHoodPosition(double pos) {
        double clipped = Math.max(0.0, Math.min(1.0, pos));
        if (Math.abs(clipped - targetHoodPosition) > HOOD_POSITION_EPSILON) {
            hoodSettleTimer.reset();
            targetHoodPosition = clipped;
            Hood.setPosition(clipped);
        }
    }

    public ShooterAimingModel.Solution aimForDistance(double distanceIn) {
        ShooterAimingModel.Solution solution = aimingModel.update(distanceIn);
        lastDistanceIn = distanceIn;
        lastSolution = solution;
        setHoodPosition(solution.hoodPosition);
        setVelocity(solution.rpm);
        return solution;
    }

    public ShooterAimingModel.Solution standbyForDistance(double distanceIn) {
        ShooterAimingModel.Solution solution = aimingModel.update(distanceIn);
        lastDistanceIn = distanceIn;
        lastSolution = solution;
        setHoodPosition(solution.hoodPosition);
        setVelocity(solution.rpm);
        return solution;
    }

    public boolean isHoodSettled() {
        return hoodSettleTimer.seconds() >= HOOD_SETTLE_TIME_S;
    }

    public double getTargetHoodPosition() {
        return targetHoodPosition;
    }

    public double getTargetVelocity(){
        return targetVelocityRPM;
    }

    public double getLastDistanceIn() {
        return lastDistanceIn;
    }

    public ShooterAimingModel.Solution getLastSolution() {
        return lastSolution;
    }

    public String getLastProfileName() {
        return lastSolution == null ? "NONE" : lastSolution.profileName;
    }
}
