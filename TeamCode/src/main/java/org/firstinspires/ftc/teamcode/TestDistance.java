package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test capteur de distance")
public class TestDistance extends LinearOpMode {
    private DistanceSensor distance0;
    private DistanceSensor distance1;
    @Override
    public void runOpMode() {
        distance0 = hardwareMap.get(DistanceSensor.class, "distance0");
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance 0", distance0.getDistance(DistanceUnit.CM));
            telemetry.addData("distance 1", distance1.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}