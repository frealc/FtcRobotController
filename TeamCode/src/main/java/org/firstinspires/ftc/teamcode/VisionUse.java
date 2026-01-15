package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Autonomous
public class VisionUse extends OpMode {
    VisionTest visionTest = new VisionTest();


    @Override
    public void init(){
    visionTest.init(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
    visionTest.update();
    AprilTagDetection id21 = VisionTest.getTagBySpecificId(21);
    visionTest.DisplayDetectionTelemetry(id21);
    }
}
