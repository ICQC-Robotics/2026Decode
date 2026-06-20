package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

import java.util.function.IntSupplier;

/**
 * Drives out to whichever zone endpoint vision initially favors and then straight back to
 * {@code returnPose}, both legs built as one continuous PathChain. Pedro Pathing's default
 * deceleration only brakes for the chain's last leg, so the robot cruises through the zone
 * waypoint instead of stopping there and restarting -- there's no need to stop since the intake
 * runs continuously.
 *
 * While still approaching the zone (alliance-relative x still dropping to new lows) and within
 * {@code slowXIn} of the wall, caps drive power at {@code slowPower} for better control; full
 * power resumes the instant it turns back for the return leg, even if still under slowXIn.
 *
 * Keeps re-sampling vision while approaching the zone. If the densest zone ever changes to
 * something other than the one currently being driven to, the robot abandons its path and snaps
 * to zone 1 specifically (not whichever zone newly looks best) -- a simple, deterministic
 * fallback for an artifact that rolled out from under the original pick. Once within
 * {@code commitDistanceIn} of the zone, re-targeting is permanently disabled for the rest of this
 * command's run (including the whole return leg) -- otherwise, once the robot passes the zone and
 * heads back, distance-to-zone starts increasing again and could spuriously re-trigger a retarget.
 *
 * Re-targeting rebuilds both legs from the live pose (not the original start), since the robot has
 * already moved by the time a switch happens.
 */
public class VisionGuidedZoneApproach extends CommandBase {
    private static final int FALLBACK_ZONE = 1;

    private final Follower follower;
    private final Vision vision;
    private final boolean isBlue;
    private final Pose[] zoneEndpoints; // index 1..N; index 0 unused
    private final double endHeadingRad;
    private final IntSupplier initialZoneSupplier;
    private final long resampleIntervalMs;
    private final double commitDistanceIn;
    private final Pose returnPose;
    private final double slowXIn;
    private final double slowPower;

    private int currentZone;
    private long lastSampleTime;
    private boolean committed;

    private boolean belowSlowX;
    private double minXSinceEntry;

    public VisionGuidedZoneApproach(Follower follower, Vision vision, boolean isBlue,
                                     Pose[] zoneEndpoints, double endHeadingRad,
                                     IntSupplier initialZoneSupplier, long resampleIntervalMs,
                                     double commitDistanceIn, Pose returnPose,
                                     double slowXIn, double slowPower) {
        this.follower = follower;
        this.vision = vision;
        this.isBlue = isBlue;
        this.zoneEndpoints = zoneEndpoints;
        this.endHeadingRad = endHeadingRad;
        this.initialZoneSupplier = initialZoneSupplier;
        this.resampleIntervalMs = resampleIntervalMs;
        this.commitDistanceIn = commitDistanceIn;
        this.returnPose = returnPose;
        this.slowXIn = slowXIn;
        this.slowPower = slowPower;
    }

    /** Which zone endpoint the robot actually ended up driving to (may differ from the initial pick). */
    public int getCurrentZone() {
        return currentZone;
    }

    @Override
    public void initialize() {
        currentZone = initialZoneSupplier.getAsInt();
        committed = false;
        belowSlowX = false;
        lastSampleTime = System.currentTimeMillis();
        startPathTo(currentZone);
    }

    @Override
    public void execute() {
        updateSpeedCap();

        if (committed || currentZone == FALLBACK_ZONE) return;

        long now = System.currentTimeMillis();
        if (now - lastSampleTime < resampleIntervalMs) return;
        lastSampleTime = now;

        Pose live = follower.getPose();
        Pose target = zoneEndpoints[currentZone];
        double remaining = Math.hypot(target.getX() - live.getX(), target.getY() - live.getY());
        if (remaining < commitDistanceIn) {
            committed = true;
            return;
        }

        int liveBest = vision.getScannedZone(isBlue);
        if (liveBest != 0 && liveBest != currentZone) {
            currentZone = FALLBACK_ZONE;
            startPathTo(currentZone);
        }
    }

    /**
     * Caps drive power while still approaching (alliance-relative x still dropping to new lows)
     * and within slowXIn of the wall; full power otherwise, including the whole return leg.
     * Tracks the lowest x seen since last crossing below slowXIn, same trick as a one-shot
     * crossing detector -- once x climbs back off that low, the robot has turned around.
     */
    private void updateSpeedCap() {
        double x = allianceRelativeX(follower.getPose().getX());

        if (x >= slowXIn) {
            belowSlowX = false;
            follower.setMaxPower(1.0);
            return;
        }

        if (!belowSlowX) {
            belowSlowX = true;
            minXSinceEntry = x;
        }

        boolean stillApproaching = x <= minXSinceEntry;
        if (stillApproaching) minXSinceEntry = x;

        follower.setMaxPower(stillApproaching ? slowPower : 1.0);
    }

    private double allianceRelativeX(double realX) {
        return isBlue ? realX : (144.0 - realX);
    }

    private void startPathTo(int zone) {
        Pose live = follower.getPose();
        Pose zoneEndpoint = zoneEndpoints[zone];
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(live.getX(), live.getY()), zoneEndpoint))
                .setLinearHeadingInterpolation(live.getHeading(), endHeadingRad)
                .addPath(new BezierLine(zoneEndpoint, returnPose))
                .setLinearHeadingInterpolation(endHeadingRad, returnPose.getHeading())
                .build();
        follower.followPath(path, 1.0, true);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
