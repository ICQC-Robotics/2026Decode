package org.firstinspires.ftc.teamcode.Mode.juniorsubsystemteleop;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    DcMotorEx intake;

    public IntakeSubsystem(HardwareMap hMap) {}

    public double getVelocity() {
        return intake.getVelocity();
    }
}


