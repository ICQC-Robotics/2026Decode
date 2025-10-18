package org.firstinspires.ftc.teamcode.Mode.raihancode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

class DriverControls {

    private static final GamepadKeys.Button RIGHT_TRIGGER = DriverControls.RIGHT_TRIGGER;
    public ChassisSubsystem chassis;
    public ActiveIntakeSubsystem intake;
    public ShooterSystem shooter;

    private GamepadEx driver;

    public DriverControls(HardwareMap hardwareMap, com.qualcomm.robotcore.hardware.Gamepad gamepad) {

        // Initialize subsystems
        chassis = new ChassisSubsystem(hardwareMap);
        intake = new ActiveIntakeSubsystem(hardwareMap);
        shooter = new ShooterSystem(hardwareMap.get(DcMotorEx.class, "shooter"));

        driver = new GamepadEx(gamepad);

        // Chassis default: arcade drive
        chassis.setDefaultCommand(
                new ArcadeDriveCommand(chassis,
                        () -> -driver.getLeftY(),
                        () -> driver.getRightX())
        );

        // Button bindings
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new IntakeCommand(intake, 1));   // intake in
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new IntakeCommand(intake, -1));  // intake out
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ShooterCommand(shooter, 1500)); // spin up
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ShooterCommand(shooter, 0));    // stop
        driver.getGamepadButton(RIGHT_TRIGGER)
                .whileHeld(new ShooterCommand(shooter, 1800));   // shoot one
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }

    private class RIGHT_TRIGGER {
    }
}
