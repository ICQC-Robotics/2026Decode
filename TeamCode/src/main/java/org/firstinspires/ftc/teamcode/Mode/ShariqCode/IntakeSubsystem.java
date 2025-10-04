package org.firstinspires.ftc.teamcode.Mode.ShariqCode.;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActiveIntakeSubsystem extends SubsystemBase{

    DcMotorEx intake;

    public  ActiveIntakeSubsystem(HardwareMap hMap){}

    public double getVelocity(){
        return intake.getVelocity();
    }
}



