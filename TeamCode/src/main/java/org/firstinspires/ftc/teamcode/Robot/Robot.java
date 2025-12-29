package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.subsystems.*;

public class Robot {
    public Drive drive;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Vision vision;
    public Wait wait;

    public Robot(HardwareMap h, Telemetry t) {
        drive = new Drive(
                h, t,
                h.get(DcMotorEx.class, "fR"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "fL"), DcMotorSimple.Direction.REVERSE,
                h.get(DcMotorEx.class, "bR"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "bL"), DcMotorSimple.Direction.REVERSE
        );

        intake = new Intake(
                h.get(DcMotorEx.class, "intake"),
                h.get(Servo.class, "servo3"),
                DcMotorSimple.Direction.FORWARD
        );

        shooter = new Shooter(
                h.get(DcMotorEx.class, "shooter1"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "shooter2"), DcMotorSimple.Direction.REVERSE,
                h.get(Servo.class, "servo0"),
                h.get(Servo.class, "servo2"),
                h.get(Servo.class, "servo1"),
                new PIDFCoefficients(0.075, 0.0, 0.005, 0.5)
        );

        turret = new Turret(
                h.get(DcMotorEx.class, "turret"),
                DcMotorSimple.Direction.REVERSE,
                new PIDFCoefficients(15, 0, 0, 0)
        );

        vision = new Vision(
                h
        );

        wait = new Wait();
    }

    public void Action(GamepadEx g, GamepadKeys.Button b, Command Press, Command Release) {
        if(Release == null) {
            new GamepadButton(g, b).whenPressed(Press);
        } else {
            new GamepadButton(g, b).whenPressed(Press).whenReleased(Release);
        }
    }

    public void schedule(Command... command) {
        CommandScheduler.getInstance().schedule(command);
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }
}
