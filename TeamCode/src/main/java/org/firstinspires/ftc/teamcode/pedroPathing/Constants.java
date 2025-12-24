package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;


import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-49.7)
            .lateralZeroPowerAcceleration(-59.94349)
            .centripetalScaling(0.0003)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07,0.000075,0.002,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5,0.2,0.05,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0018,0,0.00001,0.6,0));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(73.92097787589588)
            .yVelocity(79.29183977);

    public static ThreeWheelConstants localizerConstants =
            new ThreeWheelConstants()
                    .forwardTicksToInches(0.0010609088278141003)
                    .strafeTicksToInches(0.0010486712643076903)
                    .turnTicksToInches(0.001428505931999635)
                    .leftPodY(2.75)
                    .rightPodY(-3.625)
                    .strafePodX(-7)
                    .leftEncoder_HardwareMapName("frontLeft")
                    .rightEncoder_HardwareMapName("frontRight")
                    .strafeEncoder_HardwareMapName("backRight")
                    .leftEncoderDirection(Encoder.REVERSE)
                    .rightEncoderDirection(Encoder.REVERSE)
                    .strafeEncoderDirection(Encoder.REVERSE);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.8,
            0.1,
            0.1,
            0.007,
            500,
            5,
            10,
            3
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}