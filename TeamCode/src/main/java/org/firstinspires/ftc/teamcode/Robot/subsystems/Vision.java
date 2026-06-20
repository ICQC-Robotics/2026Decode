package org.firstinspires.ftc.teamcode.Robot.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Vision.ArtifactZoneProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class Vision extends SubsystemBase {

    private final VisionPortal visionPortal;
    private final ArtifactZoneProcessor zoneProcessor;

    // A zone needs at least this much box coverage (%) to be considered "has artifacts".
    // CALIBRATE on robot alongside ArtifactZoneProcessor.HORIZONTAL_FOV_DEG / HSV thresholds.
    private static final double MIN_DENSITY_PCT = 1.0;
    // Shoot-position -> zone endpoint distances (inches), used only to break ties. Alliance-symmetric.
    // Zones 1/3/5 are the original lanes (y=8/20/32); 2/4 are the new in-between lanes (y=14/26).
    private static final double[] ZONE_DIST = {0, 45.89, 45.50, 45.89, 47.06, 48.93};

    public Vision(WebcamName webcam) {
        zoneProcessor = new ArtifactZoneProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                // Density work doesn't need a big image -- a high default resolution can make
                // processFrame() slower than the camera's frame interval, backing up frames
                // (the "old frames" symptom). Keep this low; check getFps() to confirm it helps.
                .setCameraResolution(new Size(320, 240))
                .addProcessor(zoneProcessor)
                .build();
    }

    /** Actual achieved vision-processing rate (frames/sec). For diagnosing lag/backlog. */
    public float getFps() {
        return visionPortal.getFps();
    }

    /** Flips the on-screen box layout to match. Call as soon as alliance is known/changes. */
    public void setAlliance(boolean isBlue) {
        zoneProcessor.setAlliance(isBlue);
    }

    /**
     * Per-zone artifact density (% box coverage), index 1..5. The processor flips its box
     * layout to match the given alliance, so this is already zone-correct either way.
     */
    public double[] getLastDensity(boolean isBlue) {
        zoneProcessor.setAlliance(isBlue);
        return zoneProcessor.getZoneDensityPct();
    }

    /**
     * @param isBlue current alliance (flips the processor's box layout to match)
     * @return 1-5 for the densest zone; 0 means "no good zone" -> caller runs sweep.
     */
    public int getScannedZone(boolean isBlue) {
        double[] density = getLastDensity(isBlue);

        int best = 0;
        for (int z = 1; z <= 5; z++) {
            if (density[z] < MIN_DENSITY_PCT) continue;
            if (best == 0
                    || density[z] > density[best]
                    || (density[z] == density[best] && ZONE_DIST[z] < ZONE_DIST[best])) {
                best = z;
            }
        }
        return best;
    }
}
