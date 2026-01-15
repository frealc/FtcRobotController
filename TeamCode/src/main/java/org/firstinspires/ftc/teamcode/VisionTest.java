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
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private static List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap,Telemetry telemetry){
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update(){
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }


    public void DisplayDetectionTelemetry(AprilTagDetection detectedId){

        if (detectedId==null){return;}

        if (detectedId.metadata != null) {

            telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));

            telemetry.addLine(String.format(Locale.US,"XYZ %.1f %.1f %.1f (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));

            telemetry.addLine(String.format(Locale.US,"PRY %.1f %.1f %.1f (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));

            telemetry.addLine(String.format(Locale.US,"RBE %.1f %.1f %.1f (inch, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));

        } else {
            telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) Unknown", detectedId.id));

            telemetry.addLine(String.format(Locale.US,"Center %.0f %.0f (pixels)", detectedId.center.x, detectedId.center.y));
        }
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