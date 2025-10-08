package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.subsystems.*;

public class Robot {
    public Drive drive;
    public Intake intake;
    public Shooter shooter;
    public Vision vision;

    public Robot(HardwareMap h, GamepadEx g1, GamepadEx g2) {
        drive = new Drive(
                g1,
                h.get(DcMotorEx.class, "fR"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "fL"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "bR"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "bL"), DcMotorSimple.Direction.FORWARD
        );

        intake = new Intake(
                h.get(DcMotorEx.class, "intake"),
                h.get(Servo.class, "intakeServo")
        );

        shooter = new Shooter(
                h.get(DcMotorEx.class, "rightShooter"), DcMotorSimple.Direction.FORWARD,
                h.get(DcMotorEx.class, "leftShooter"), DcMotorSimple.Direction.FORWARD,
                new PIDFCoefficients(0.0, 0.0, 0.0, 0.0)
        );

        vision = new Vision(h);
    }

    public void Action(GamepadEx g, GamepadKeys.Button b, Command Press, Command Release) {
        if(Release == null) {
            new GamepadButton(g, b).whenPressed(Press);
        } else {
            new GamepadButton(g, b).whenPressed(Press).whenReleased(Release);
        }
    }

    public Command AutoAim() {
        return new AutoAim(vision, shooter);
    }
}
