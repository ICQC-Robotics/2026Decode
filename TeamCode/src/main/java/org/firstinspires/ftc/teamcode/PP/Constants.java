package org.firstinspires.ftc.teamcode.PP;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.515)
            .forwardZeroPowerAcceleration(-32.671)
            .lateralZeroPowerAcceleration(-68.032)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, .04, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0.0,0.0001,0.6,0.01))
            .centripetalScaling(0.0004);

    public static PathConstraints pathConstraints = new PathConstraints(1.0, 50, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fR")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("fL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.835)
            .yVelocity(57.5325);



    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.45)
            .strafePodX(6.94)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}