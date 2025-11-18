package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait extends SubsystemBase {
    private ElapsedTime t;
    public Wait() {
        t = new ElapsedTime();
    }

    public void start(){
        t.reset();
    }

    public double elapsed(){
        return t.seconds();
    }
}
