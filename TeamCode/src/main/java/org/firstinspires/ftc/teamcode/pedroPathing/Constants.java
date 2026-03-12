package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.odoX;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.odoY;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .mass(13.7)//8.9
            .forwardZeroPowerAcceleration(-39.8)
            .lateralZeroPowerAcceleration(-74.125)
            //.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.15, 0.00199, 0.0514))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.037, 0.015))  //0.037  0.015
            .headingPIDFCoefficients(new PIDFCoefficients(1.7, 0, 0.06, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.0009, 0.4, 0.01)) //0.0047
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0001);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(76.45)
            .yVelocity(61.64)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .useVoltageCompensation(true)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.579) //128.953  130  4.579
            .strafePodX(4.469) //34.477 11.5
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 0, 1.5, 1); //1.8, 1

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
