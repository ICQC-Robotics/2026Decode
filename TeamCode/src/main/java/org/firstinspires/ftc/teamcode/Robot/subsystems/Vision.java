package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot.Vision.IntakeColorProcessing;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public class Vision extends SubsystemBase {

    //ll
    private final Limelight3A limelight;
    private LLResult lastResult;
    public final double turretRadius = 2.75;
    public final double turretOffsetX = 0;
    public final double turretOffsetY = 3;

    public Vision(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.pipelineSwitch(GREEN_PIPELINE);
        limelight.start();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
    }
    public static final int GREEN_PIPELINE  = 0;   // tuned green color pipeline
    public static final int PURPLE_PIPELINE = 1;   // tuned purple color pipeline

    // tx bearing (deg) to a SINGLE ball sitting in each zone, read from the shoot pose. CALIBRATE.
    private static final double[] TX_ZONE   = {0, 2.0, -17.0, -32.0};  // [1]=z1 [2]=z2 [3]=z3
    // apparent area (%) of ONE ball in each zone — normalizes density for distance. CALIBRATE.
    private static final double[] BALL_AREA = {0, 0.9,  0.7,  0.5};
    // rough shoot->zone travel cost, used only to break ties. lower = nearer = preferred.
    private static final double[] ZONE_DIST = {0, 37.0, 38.6, 43.6};

    private static final double TX_TOL      = 8.0;   // half-width of each zone's tx band (deg)
    private static final double NOISE_AREA  = 0.15;  // ignore blobs smaller than this (% area)
    private static final double MIN_DENSITY = 0.5;   // a zone needs at least this "ball count" to qualify
    private static final double DENSITY_EPS = 0.6;   // two zones within this are treated as a tie
    private static final int    SCAN_FRAMES = 5;     // frames averaged per pipeline per decision

    private final double[] lastDensity = new double[4];  // [1..3] combined ball count, kept for telemetry

    /**
     * Scan BOTH color pipelines, sum per-zone density, pick the densest zone
     * (nearest one on a tie).
     * @param isBlue current alliance (mirrors tx for red so one calibration set covers both)
     * @return 1, 2, or 3 for the chosen zone; 0 means "no good zone" -> caller runs sweep.
     */
    public int scanArtifactZone(boolean isBlue) {
        double[] green  = scanOnePipeline(GREEN_PIPELINE,  isBlue);
        double[] purple = scanOnePipeline(PURPLE_PIPELINE, isBlue);

        // combined density = green balls + purple balls in each zone
        for (int z = 1; z <= 3; z++) lastDensity[z] = green[z] + purple[z];

        // Pass 1: find the highest density among zones that clear the floor.
        double maxD = -1;
        for (int z = 1; z <= 3; z++) {
            if (lastDensity[z] >= MIN_DENSITY && lastDensity[z] > maxD) maxD = lastDensity[z];
        }
        if (maxD < 0) return 0;   // nothing worth visiting -> sweep

        // Pass 2: among zones tied within DENSITY_EPS of the leader, pick the NEAREST.
        int best = 0;
        for (int z = 1; z <= 3; z++) {
            if (lastDensity[z] < MIN_DENSITY) continue;
            if (maxD - lastDensity[z] <= DENSITY_EPS) {
                if (best == 0 || ZONE_DIST[z] < ZONE_DIST[best]) best = z;
            }
        }
        return best;
    }

    /** Switch to one color pipeline, average several frames, return per-zone ball-count estimate. */
    private double[] scanOnePipeline(int pipeline, boolean isBlue) {
        // Switching pipelines needs a few frames to produce valid data, so wait after a switch.
        if (limelight.getStatus().getPipelineIndex() != pipeline) {
            limelight.pipelineSwitch(pipeline);
            sleepMs(120);
        }

        double[] areaSum = new double[4];
        int valid = 0;

        // Average several frames so one bad frame can't misroute a whole cycle.
        for (int n = 0; n < SCAN_FRAMES; n++) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                for (LLResultTypes.ColorResult c : r.getColorResults()) {
                    double tx = c.getTargetXDegrees();
                    if (!isBlue) tx = -tx;               // mirror to the blue calibration frame
                    double area = c.getTargetArea();
                    if (area < NOISE_AREA) continue;     // drop speckle / noise
                    int z = bucket(tx);                  // which zone's tx band is this blob in?
                    if (z != 0) areaSum[z] += area;      // merged balls sum together -> density survives
                }
                valid++;
            }
        }

        // Convert summed area -> approximate ball count per zone (distance-normalized).
        double[] density = new double[4];
        if (valid > 0) {
            for (int z = 1; z <= 3; z++) {
                density[z] = (areaSum[z] / valid) / BALL_AREA[z];
            }
        }
        return density;
    }

    /** Map a tx bearing to zone 1/2/3, or 0 if it doesn't fall in any zone's band. */
    private int bucket(double tx) {
        for (int z = 1; z <= 3; z++) {
            if (Math.abs(tx - TX_ZONE[z]) < TX_TOL) return z;
        }
        return 0;
    }

    /** Latest per-zone combined density estimate, for telemetry. Index 1..3. */
    public double[] getLastDensity() {
        return lastDensity;
    }

    private static void sleepMs(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    public int debugBucket(double tx) {
        return bucket(tx);
    }
}