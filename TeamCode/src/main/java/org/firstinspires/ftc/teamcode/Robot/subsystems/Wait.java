package org.firstinspires.ftc.teamcode.Robot.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait extends SubsystemBase {
    private ElapsedTime t;
    public Wait() {
        t = new ElapsedTime();
    }

    public void start(){
        t.reset();
    }

    public double elapesd(){
        return t.milliseconds();
    }
}
