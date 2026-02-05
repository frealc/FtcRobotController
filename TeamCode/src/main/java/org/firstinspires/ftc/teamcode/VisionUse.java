package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
@Disabled
public class VisionUse extends LinearOpMode {
    VisionTest vision = new VisionTest();


    @Override
    public void runOpMode(){
        vision.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()){

            vision.update();
            AprilTagDetection id21 = VisionTest.getTagBySpecificId(21);
            vision.updateTelemetry();
        }
    }


}
