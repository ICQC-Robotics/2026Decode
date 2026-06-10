package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter extends SubsystemBase {

    // Bang-bang hysteresis band (±RPM around target). Tune via FTC Dashboard.
    public static double BANG_BAND_RPM = 25.0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double MIN_VALID_RPM = 1.0;
    private static final double TARGET_CHANGE_RESET_RPM = 50.0;
    private static final double HOOD_POSITION_EPSILON = 0.002;
    private static final double HOOD_SETTLE_TIME_S = 0.0;

    private final DcMotorEx rightShooter, leftShooter;
    public final Servo Cover;
    private final Servo Hood;
    private final ShooterAimingModel aimingModel = new ShooterAimingModel();
    private final ElapsedTime hoodSettleTimer = new ElapsedTime();

    private double targetVelocityRPM = 0.0;
    private double targetHoodPosition = -1.0;
    private boolean bangHigh = false;

    // Velocity cached once per loop to avoid redundant I2C reads
    private double cachedRightRPM = 0.0;
    private double cachedLeftRPM = 0.0;
    private double cachedVelocityRPM = 0.0;

    private double lastDistanceIn = 0.0;
    private ShooterAimingModel.Solution lastSolution;

    public Shooter(
            DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
            DcMotorEx leftShooter, DcMotorSimple.Direction leftDir,
            Servo Cover,
            Servo Hood
    ) {
        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;
        this.Cover = Cover;
        this.Hood = Hood;

        this.rightShooter.setDirection(rightDir);
        // FLOAT (coast) so the flywheel spins down naturally on bang-bang "off" phase
        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        this.leftShooter.setDirection(leftDir);
        this.leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        setMagazineCover(1.0);
        setHoodPosition(0.2);
    }

    // ── velocity control ───────────────────────────────────────────────────────

    public void setVelocity(double rpm) {
        if (rpm <= 0) {
            stop();
            return;
        }
        if (Math.abs(rpm - targetVelocityRPM) > TARGET_CHANGE_RESET_RPM) {
            bangHigh = false; // reset bang state on large target jumps
        }
        targetVelocityRPM = rpm;
    }

    public void stop() {
        targetVelocityRPM = 0;
        bangHigh = false;
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    /** Returns the averaged flywheel velocity (RPM), cached from the last periodic() call. */
    public double getVelocity() {
        return cachedVelocityRPM;
    }

    /** Signed right-motor velocity (RPM), cached from the last periodic() call. */
    public double getRightVelocity() {
        return cachedRightRPM;
    }

    /** Signed left-motor velocity (RPM), cached from the last periodic() call. */
    public double getLeftVelocity() {
        return cachedLeftRPM;
    }

    public boolean isAtTargetVelocity(double toleranceRPM) {
        return targetVelocityRPM > 0
                && Math.abs(cachedVelocityRPM - targetVelocityRPM) <= toleranceRPM;
    }

    // ── aiming model ───────────────────────────────────────────────────────────

    /**
     * Looks up the hood angle and RPM for {@code distanceIn}, applies them immediately,
     * and returns the solution. Call every loop while in standby or mid-shot.
     */
    public ShooterAimingModel.Solution aimForDistance(double distanceIn) {
        ShooterAimingModel.Solution solution = aimingModel.update(distanceIn);
        lastDistanceIn = distanceIn;
        lastSolution = solution;
        setHoodPosition(solution.hoodPosition);
        setVelocity(solution.rpm);
        return solution;
    }

    /** Identical to {@link #aimForDistance} — kept for call-site readability. */
    public ShooterAimingModel.Solution standbyForDistance(double distanceIn) {
        return aimForDistance(distanceIn);
    }

    // ── servo control ──────────────────────────────────────────────────────────

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

    // ── state queries ──────────────────────────────────────────────────────────

    public boolean isHoodSettled() {
        return hoodSettleTimer.seconds() >= HOOD_SETTLE_TIME_S;
    }

    public double getTargetHoodPosition() {
        return targetHoodPosition;
    }

    public double getTargetVelocity() {
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

    // ── periodic (bang-bang) ───────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Read both motors once per loop — all getters return these cached values
        cachedRightRPM = ticksPerSecondToRPM(rightShooter.getVelocity());
        cachedLeftRPM  = ticksPerSecondToRPM(leftShooter.getVelocity());

        double rightAbs = Math.abs(cachedRightRPM);
        double leftAbs  = Math.abs(cachedLeftRPM);
        if (rightAbs < MIN_VALID_RPM)      cachedVelocityRPM = leftAbs;
        else if (leftAbs < MIN_VALID_RPM)  cachedVelocityRPM = rightAbs;
        else                               cachedVelocityRPM = (rightAbs + leftAbs) / 2.0;

        if (targetVelocityRPM <= 0) return;

        // Hysteresis band prevents chatter at the boundary
        if (cachedVelocityRPM < targetVelocityRPM - BANG_BAND_RPM)      bangHigh = true;
        else if (cachedVelocityRPM > targetVelocityRPM + BANG_BAND_RPM) bangHigh = false;

        double out = bangHigh ? 1.0 : 0.0;
        rightShooter.setPower(out);
        leftShooter.setPower(out);
    }

    private static double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }
}
