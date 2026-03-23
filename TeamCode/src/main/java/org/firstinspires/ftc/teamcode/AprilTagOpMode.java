package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
@Disabled

@TeleOp(name = "AprilTag Test", group = "Vision")
public class AprilTagOpMode extends LinearOpMode {


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // Step 1: créer le processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Step 2: créer le VisionPortal
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addData("Nb Tags", detections.size());

            for (AprilTagDetection detection : detections) {

                telemetry.addData("ID", detection.id);

                if (detection.ftcPose != null) {
                    telemetry.addData("X", detection.ftcPose.x);
                    telemetry.addData("Y", detection.ftcPose.y);
                    telemetry.addData("Z", detection.ftcPose.z);
                    telemetry.addData("Yaw", detection.ftcPose.yaw);
                }
            }

            telemetry.update();
        }
    }
}