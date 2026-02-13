package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySubsystem extends SubsystemBase {
    Robot robot;
    MultipleTelemetry telemetry;
    public TelemetrySubsystem(Telemetry t, Robot r){
        telemetry = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());
        robot = r;

    }
    @Override
    public void periodic(){
        telemetry.addLine("----------Shooter---------");
        telemetry.addLine("----------Turret---------");
        telemetry.addLine("----------Drive---------");
        telemetry.addLine("----------Intake---------");
        //telemetry.addLine("----------Limelight---------");
        telemetry.update();
    }
}
