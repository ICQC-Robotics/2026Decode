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

import org.firstinspires.ftc.teamcode.Robot.subsystems.*;

public class Robot {
    public Drive drive;
    public Intake intake;
    public Shooter shooter;
    public Vision vision;
    public Wait wait;

    public Robot(HardwareMap h, GamepadEx g1, GamepadEx g2) {
        drive = new Drive(
                h, g1,
                h.get(DcMotorEx.class, "m2"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "m1"), DcMotorSimple.Direction.REVERSE,
                h.get(DcMotorEx.class, "m3"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "m4"), DcMotorSimple.Direction.REVERSE
        );

        intake = new Intake(
                h.get(DcMotorEx.class, "m5"),
                h.get(DcMotorEx.class, "m6"),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD
        );

        shooter = new Shooter(
                h.get(DcMotorEx.class, "m7"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "m8"), DcMotorSimple.Direction.FORWARD,
                new PIDFCoefficients(0.075, 0.0, 0.005, 0.5)
        );

        vision = new Vision(h,
                drive
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

    public void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }
}
