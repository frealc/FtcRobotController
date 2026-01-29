package org.firstinspires.ftc.teamcode;

import static java.lang.String.*;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


public class VisionTest {
    public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap,Telemetry telemetry){
        this.telemetry = telemetry;

        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    public void update(){
        detectedTags = aprilTag.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }


    public void updateTelemetry() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addData("Nb Tags", detections.size());

        for (AprilTagDetection tag : detections) {
            telemetry.addLine("----------------");
            telemetry.addData("ID", tag.id);
            telemetry.addData("Distance (cm)", tag.ftcPose.range);
            telemetry.addData("Bearing (deg)", tag.ftcPose.bearing);
            telemetry.addData("Yaw (deg)", tag.ftcPose.yaw);
        }

        telemetry.update();
    }

    public static AprilTagDetection getTagBySpecificId(int id){
        for (AprilTagDetection detection : detectedTags){
            if (detection.id==id){
                return detection;
            }
        }
        return null;
    }

    public void stop(){
        if (visionPortal != null){
            visionPortal.close();
        }
    }

}





/*
VisionTest vision = new VisionTest();


            vision.update();
            AprilTagDetection tag = VisionTest.getTagBySpecificId(24);
*if (tag != null) {
                vision.update();

                yaw = tag.ftcPose.yaw;
                range = tag.ftcPose.range;

                telemetry.addData("Yaw", yaw);

                if (range <= 100){
                    Power = -0.5;
                }else if (range >= 200){
                    Power = 0.5;
                }else {
                    Power = 0;
                }

                if (yaw >= 6.5){
                    tgtpowerRota = -0.5;
                } else if (yaw <= -6.5){
                    tgtpowerRota = 0.5;
                } else {
                    tgtpowerRota = 0;
                }

            }
*/