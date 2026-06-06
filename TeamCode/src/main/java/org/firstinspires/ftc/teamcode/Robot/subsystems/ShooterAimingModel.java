package org.firstinspires.ftc.teamcode.Robot.subsystems;

public class ShooterAimingModel {

    public static class Solution {
        public final String profileName;
        public final double hoodPosition;
        public final double rpm;
        public final double holdMinIn;
        public final double holdMaxIn;

        public Solution(String profileName, double hoodPosition, double rpm,
                        double holdMinIn, double holdMaxIn) {
            this.profileName = profileName;
            this.hoodPosition = hoodPosition;
            this.rpm = rpm;
            this.holdMinIn = holdMinIn;
            this.holdMaxIn = holdMaxIn;
        }
    }

    private static class Profile {
        final String name;
        final double hoodPosition;
        final double holdMinIn;
        final double holdMaxIn;
        final double centerIn;
        final double[][] rpmLut;

        Profile(String name, double hoodPosition, double holdMinIn, double holdMaxIn,
                double[][] rpmLut) {
            this.name = name;
            this.hoodPosition = hoodPosition;
            this.holdMinIn = holdMinIn;
            this.holdMaxIn = holdMaxIn;
            this.centerIn = (holdMinIn + holdMaxIn) / 2.0;
            this.rpmLut = rpmLut;
        }

        boolean contains(double distanceIn) {
            return distanceIn >= holdMinIn && distanceIn <= holdMaxIn;
        }

        double distanceOutside(double distanceIn) {
            if (distanceIn < holdMinIn) return holdMinIn - distanceIn;
            if (distanceIn > holdMaxIn) return distanceIn - holdMaxIn;
            return 0.0;
        }

        double distanceToNearestSample(double distanceIn) {
            double best = Double.POSITIVE_INFINITY;
            for (double[] sample : rpmLut) {
                best = Math.min(best, Math.abs(distanceIn - sample[0]));
            }
            return best;
        }

        Solution solve(double distanceIn) {
            return new Solution(name, hoodPosition, lookupInterpolated(distanceIn, rpmLut),
                    holdMinIn, holdMaxIn);
        }
    }

    private static final Profile[] DEFAULT_PROFILES = new Profile[] {
            new Profile("HOOD_0_20", 0.2, 35.0, 52.0, new double[][] {
                    {40.0, 2600.0},
                    {50.0, 2800.0}
            }),
            new Profile("HOOD_0_25", 0.35, 50.0, 60.0, new double[][] {
                    {54.5, 2700.0},
                    {56.0, 2850.0},
                    {58.0, 2900.0},
                    {62.5, 2800.0},
            }),
            new Profile("HOOD_0_70", 0.7, 70.0, 145.0, new double[][] {
                    {67, 3160.0},
                    {80.0, 3260.0},
                    {97.5, 3400.0},
                    {115.5, 3700.0},
                    {133.5, 4000.0}
            })
    };

    private final Profile[] profiles;
    private Profile currentProfile;

    public ShooterAimingModel() {
        this(DEFAULT_PROFILES);
    }

    private ShooterAimingModel(Profile[] profiles) {
        this.profiles = profiles;
    }

    public Solution update(double distanceIn) {
        currentProfile = selectProfile(distanceIn);
        return currentProfile.solve(distanceIn);
    }

    public Solution preview(double distanceIn) {
        return selectProfile(distanceIn).solve(distanceIn);
    }

    public void reset() {
        currentProfile = null;
    }

    private Profile selectProfile(double distanceIn) {
        if (currentProfile != null && currentProfile.contains(distanceIn)) {
            return currentProfile;
        }

        Profile best = null;
        double bestScore = Double.POSITIVE_INFINITY;

        for (Profile profile : profiles) {
            double score = profile.contains(distanceIn)
                    ? profile.distanceToNearestSample(distanceIn)
                    : 1000.0 + profile.distanceOutside(distanceIn);
            if (score < bestScore) {
                best = profile;
                bestScore = score;
            }
        }

        return best;
    }

    public static double previewDefaultRpm(double distanceIn) {
        return new ShooterAimingModel().preview(distanceIn).rpm;
    }

    private static double lookupInterpolated(double x, double[][] table) {
        if (table == null || table.length == 0) return 0.0;
        if (table.length == 1) return table[0][1];

        if (x <= table[0][0]) return table[0][1];
        int last = table.length - 1;
        if (x >= table[last][0]) return table[last][1];

        for (int i = 0; i < last; i++) {
            double x0 = table[i][0];
            double y0 = table[i][1];
            double x1 = table[i + 1][0];
            double y1 = table[i + 1][1];

            if (x >= x0 && x <= x1) {
                double span = x1 - x0;
                if (span <= 1e-9) return y0;
                double t = (x - x0) / span;
                return y0 + t * (y1 - y0);
            }
        }

        return table[last][1];
    }
}
