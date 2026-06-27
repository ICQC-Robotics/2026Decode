package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;

/**
 * Empirically tune the intake anti-jam values used by the far autos. Drives the
 * intake at a set power and prints the live motor current so you can find the
 * current spike a jam produces, then pick a threshold below it. The anti-jam
 * behaves exactly like the far autos so you can also watch the reverse kick.
 * Edit the values live in Dashboard.
 */
@Configurable
@Config
@TeleOp(name = "Intake Tuner")
public class IntakeTuner extends OpMode {
    // Same anti-jam knobs as the far autos.
    public static boolean INTAKE_ANTI_JAM_ENABLED      = true;
    public static double  INTAKE_JAM_CURRENT_THRESHOLD = 7.0; // amps
    public static double  INTAKE_REVERSE_TIME          = 0.5; // seconds

    // Power to run the intake at while tuning (negative = intaking, matches accept()).
    public static double INTAKE_POWER = -1.0;

    private Robot negabot;
    private DcMotorEx intakeMotor;

    private boolean intakeReversing = false;
    private double  intakeSavedPower = 0;
    private final ElapsedTime intakeReverseTimer = new ElapsedTime();

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, telemetry, new Pose(72, 72));
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    @Override
    public void loop() {
        // Drive the intake, unless the anti-jam is currently holding a reverse kick.
        if (!intakeReversing) {
            negabot.intake.setSpeed(INTAKE_POWER);
        }
        intakeAntiJam();

        // ── telemetry ────────────────────────────────────────────────────────
        telemetry.addData("Intake Current (A)", intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current Threshold (A)", INTAKE_JAM_CURRENT_THRESHOLD);
        telemetry.addData("Anti-Jam Enabled", INTAKE_ANTI_JAM_ENABLED);
        telemetry.addData("Reverse Time (s)", INTAKE_REVERSE_TIME);
        telemetry.addData("Intake Power", INTAKE_POWER);
        telemetry.addData("Reversing", intakeReversing);
        telemetry.update();
    }

    /** Reverse the intake briefly when its current spikes from a jam. */
    private void intakeAntiJam() {
        if (!INTAKE_ANTI_JAM_ENABLED) return;
        if (intakeReversing) {
            if (intakeReverseTimer.seconds() >= INTAKE_REVERSE_TIME) {
                intakeReversing = false;
                intakeMotor.setPower(intakeSavedPower);   // resume what we interrupted
            } else {
                intakeMotor.setPower(-intakeSavedPower);  // hold reverse vs. command writes
            }
        } else {
            double power = intakeMotor.getPower();
            if (power != 0 && intakeMotor.getCurrent(CurrentUnit.AMPS) > INTAKE_JAM_CURRENT_THRESHOLD) {
                intakeSavedPower = power;
                intakeReversing = true;
                intakeReverseTimer.reset();
                intakeMotor.setPower(-power);
            }
        }
    }

    @Override
    public void stop() {
        if (negabot != null) {
            negabot.intake.setSpeed(0);
        }
    }
}
