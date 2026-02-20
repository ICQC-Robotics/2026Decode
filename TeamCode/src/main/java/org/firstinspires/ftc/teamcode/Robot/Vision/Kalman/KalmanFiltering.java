//package org.firstinspires.ftc.teamcode.Robot.Vision.Kalman;
//
//import com.pedropathing.geometry.Pose;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.Robot.subsystems.Limelight;
//
//public final class KalmanFiltering {
//
//    private final KalmanBias1D bx;
//    private final KalmanBias1D by;
//    private final KalmanBias1D bh;
//    private double lastTimeSec = Double.NaN;
//
//    private Pose fusedPose = null;
//    private Pose lastVisionPedroPose = null;
//
//    private double baseRxy;
//    private double baseRh;
//
//    public KalmanFiltering(
//            double qRateXY, double rXY, double p0XY,
//            double qRateHeading, double rHeading, double p0Heading
//    )
//    {
//        this.bx = new KalmanBias1D(0.0, p0XY, qRateXY, rXY);
//        this.by = new KalmanBias1D(0.0, p0XY, qRateXY, rXY);
//        this.bh = new KalmanBias1D(0.0, p0Heading, qRateHeading, rHeading);
//
//        this.baseRxy = rXY;
//        this.baseRh = rHeading;
//    }
//
//    public void reset() {
//        lastTimeSec = Double.NaN;
//        fusedPose = null;
//        lastVisionPedroPose = null;
//
//        bx.reset(0.0, bx.getP());
//        by.reset(0.0, by.getP());
//        bh.reset(0.0, bh.getP());
//    }
//
//    public void update(double nowSec, Pose pedroPose, Limelight ll) {
//        if (pedroPose == null) return;
//
//        double dt = Double.isFinite(lastTimeSec) ? (nowSec - lastTimeSec) : 0.0;
//        if (dt < 0) dt = 0;
//        lastTimeSec = nowSec;
//
//        bx.predict(dt);
//        by.predict(dt);
//        bh.predict(dt);
//
//        fusedPose = new Pose(
//                pedroPose.getX() - bx.getBias(),
//                pedroPose.getY() - by.getBias(),
//                wrapRad(pedroPose.getHeading() - bh.getBias())
//        );
//
//        if (ll == null) return;
//
//        Pose3D mt2 = ll.getBotposeMT2();
//        Pose visionPedro = LimelightToPedro.toPedroPose(mt2);
//        if (visionPedro == null) return;
//
//        lastVisionPedroPose = visionPedro;
//        double zH = wrapRad(pedroPose.getHeading() - visionPedro.getHeading());
//        bh.setR(baseRh);
//        bh.update(zH);
//
//        double zx = pedroPose.getX() - visionPedro.getX();
//        double zy = pedroPose.getY() - visionPedro.getY();
//
//        bx.setR(baseRxy);
//        by.setR(baseRxy);
//
//        bx.update(zx);
//        by.update(zy);
//
//        fusedPose = new Pose(
//                pedroPose.getX() - bx.getBias(),
//                pedroPose.getY() - by.getBias(),
//                wrapRad(pedroPose.getHeading() - bh.getBias())
//        );
//    }
//
//    public Pose getFusedPose() { return fusedPose; }
//    public Pose getLastVisionPedroPose() { return lastVisionPedroPose; }
//
//    public double getBiasX() { return bx.getBias(); }
//    public double getBiasY() { return by.getBias(); }
//    public double getBiasHeading() { return bh.getBias(); }
//
//    public double getKx() { return bx.getK(); }
//    public double getKy() { return by.getK(); }
//    public double getKh() { return bh.getK(); }
//
//    private static double wrapRad(double a) {
//        while (a > Math.PI) a -= 2.0 * Math.PI;
//        while (a < -Math.PI) a += 2.0 * Math.PI;
//        return a;
//    }
//}
