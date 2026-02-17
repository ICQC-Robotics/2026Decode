package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.commands.IntakeColorProcessing;
import org.firstinspires.ftc.vision.VisionPortal;

public class Webcam extends SubsystemBase {

    private VisionPortal portal;
    private IntakeColorProcessing processor;

    public Webcam(HardwareMap hardwareMap) {

        processor = new IntakeColorProcessing();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(processor)
                .build();
    }

    public IntakeColorProcessing.Zone getZone() {
        return processor.getDetectedZone();
    }

    public void stop() {
        portal.stopStreaming();
    }

    public void close() {
        portal.close();
    }
}
