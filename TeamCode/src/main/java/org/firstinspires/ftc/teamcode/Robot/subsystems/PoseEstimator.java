package org.firstinspires.ftc.teamcode.Robot.subsystems;

public final class PoseEstimator {
    private static final double Q_X = 0;
    private static final double Q_Y = 0;
    private static final double P0 = 0;
    private static final double R = 0;

    private static final double LL_OFFSET_X = 0; //in forward from robot center
    private static final double LL_OFFSET_Y = 0; //in left from robot center

    private final KalmanFiltering kx = new KalmanFiltering(Q_X, R, P0);
    private final KalmanFiltering ky = new KalmanFiltering(Q_Y, R, P0);
    private boolean init = false;
    private double lastX, lastY;
    private double fusedX = Double.NaN;
    private double fusedY = Double.NaN;

    public void reset() {
        init = false;
        fusedX = Double.NaN;
        fusedY = Double.NaN;
    }

    public void init(double pedroX, double pedroY, double heading) {
        init = true;
        fusedX = pedroX;
        fusedY = pedroY;

        kx.reset(pedroX, P0);
        ky.reset(pedroY, P0);

        lastX = pedroX;
        lastY = pedroY;
    }

    public void update(
            double pedroX, double pedroY, double heading,
            double turretYawRad,
            boolean hasVision,
            double camFieldX, double camFieldY
    ) {

        if (!init) {
            init(pedroX, pedroY, heading);
            return;
        }

        double dx = pedroX - lastX;
        double dy = pedroY - lastY;

        kx.predict(dx);
        ky.predict(dy);

        fusedX = kx.getX();
        fusedY = ky.getX();

//        if (hasVision) {
//            double[] meas = cameraFieldToRobotCenter(camFieldX, camFieldY, heading, turretYawRad);
//            double measX = meas[0];
//            double measY = meas[1];
//
//            kx.setR(R);
//            ky.setR(R);
//
//            kx.update(measX);
//            ky.update(measY);
//
//            fusedX = kx.getX();
//            fusedY = ky.getX();
//        }

        lastX = pedroX;
        lastY = pedroY;
    }

    public double getX() { return fusedX; }
    public double getY() { return fusedY; }


//    private static double[] cameraFieldToRobotCenter(double camFieldX, double camFieldY,
//                                                     double robotHeading, double turretYaw) {
//        double cy = Math.cos(turretYaw);
//        double sy = Math.sin(turretYaw);
//
//        double offRx = LL_OFFSET_X * cy - LL_OFFSET_Y * sy;
//        double offRy = LL_OFFSET_X * sy + LL_OFFSET_Y * cy;
//
//        double ch = Math.cos(robotHeading);
//        double sh = Math.sin(robotHeading);
//
//        double offFx = offR_x * ch - offR_y * sh;
//        double offFy = offR_x * sh + offR_y * ch;
//
//        double measX = camFieldX - offF_x;
//        double measY = camFieldY - offF_y;
//
//        return new double[]{measX, measY};
//    }
}
