package org.firstinspires.ftc.teamcode.Robot.subsystems;

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
    private static final double[] ZONE_DIST = {0, 36.5, 38.1, 43.1};

    public Vision(WebcamName webcam) {
        zoneProcessor = new ArtifactZoneProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(zoneProcessor)
                .build();
    }

    /**
     * Per-zone artifact density (% box coverage), remapped from the processor's on-screen
     * left/mid/right readout to zone 1/2/3. The field (and camera view) mirrors between
     * alliances, so which on-screen box is zone 1 vs 3 flips with isBlue.
     */
    public double[] getLastDensity(boolean isBlue) {
        double[] region = zoneProcessor.getRegionDensityPct();  // [left, mid, right]
        double[] density = new double[4];
        if (isBlue) {
            density[3] = region[0];
            density[2] = region[1];
            density[1] = region[2];
        } else {
            density[1] = region[0];
            density[2] = region[1];
            density[3] = region[2];
        }
        return density;
    }

    /**
     * @param isBlue current alliance (changes which on-screen box maps to which zone)
     * @return 1, 2, or 3 for the densest zone; 0 means "no good zone" -> caller runs sweep.
     */
    public int getScannedZone(boolean isBlue) {
        double[] density = getLastDensity(isBlue);

        int best = 0;
        for (int z = 1; z <= 3; z++) {
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
