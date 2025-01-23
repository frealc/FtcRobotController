package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled

public class DistanceSensorDebbugger extends OpMode {
    private final String sensorId = "";
    private DistanceSensor distanceSensor;
    ColorSensor color1;


    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, sensorId);
        distanceSensor.resetDeviceConfigurationForOpMode();
        color1 = hardwareMap.get(ColorSensor.class, "color1");
    }

    @Override
    public void loop() {
        telemetry.addLine("--- DEBUG INFOS ---");
        telemetry.addData("ConnectionInfo", distanceSensor.getConnectionInfo());
        telemetry.addData("DeviceName", distanceSensor.getDeviceName());
        telemetry.addData("Manufacturer", distanceSensor.getManufacturer());
        telemetry.addData("Version", distanceSensor.getVersion());

        // La ligne "réellement" intéressante
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));

        telemetry.addData("Light Detected", ((OpticalDistanceSensor) color1).getLightDetected());
        telemetry.addData("Red", color1.red());
        telemetry.addData("Green", color1.green());
        telemetry.addData("Blue", color1.blue());
        telemetry.update();
    }
}
