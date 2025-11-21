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
                h.get(DcMotorEx.class, "fR"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "fL"), DcMotorSimple.Direction.REVERSE,
                h.get(DcMotorEx.class, "bR"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "bL"), DcMotorSimple.Direction.REVERSE
        );

        //m5 = intake
        //m6 = shooter

        intake = new Intake(
                h.get(DcMotorEx.class, "5"),
                h.get(Servo.class, "intake"),
                DcMotorSimple.Direction.FORWARD
        );

        shooter = new Shooter(
                h.get(DcMotorEx.class, "6"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "7"), DcMotorSimple.Direction.FORWARD,
                h.get(Servo.class, "shooter"),
                new PIDFCoefficients(0.075, 0.0, 0.005, 0.5)
        );

        vision = new Vision(h,
                drive
        );

        wait = new Wait();
        shooter.setMagazineCover(0.24);
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
