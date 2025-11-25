package org.firstinspires.ftc.teamcode;

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
            .mass(9.5)// mass du robot
            .forwardZeroPowerAcceleration(-42.22) //valeur trouvé par Pedro pour la décéleration
            .lateralZeroPowerAcceleration(-59.2)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01)); //valeur trouvé par Pedro pour la décéleration

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.15,
            0.30);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RightFront")
            .rightRearMotorName("RightBack")
            .leftRearMotorName("LeftBack")
            .leftFrontMotorName("LeftFront")

            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(45.19)
            .yVelocity(34.4);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.7)
            .strafePodX(4.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)

                .pinpointLocalizer(localizerConstants)
                /* other builder steps */
                .build();
    }


}
