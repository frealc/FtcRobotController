package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled

public class DistanceSensorDebbugger extends OpMode {
    private final String sensorId = "";
    private DistanceSensor distanceSensor;
    ColorSensor test_color;

    TouchSensor test_touch;  // Touch sensor Object


    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor.resetDeviceConfigurationForOpMode();
        test_color = hardwareMap.get(ColorSensor.class, "test_color");
        test_touch = hardwareMap.get(TouchSensor.class, "test_touch");
    }

    @Override
    public void loop() {

//-------------------------------distance sensor part (not working)------------------------------------
       telemetry.addLine("--- DEBUG INFOS ---");
        telemetry.addData("ConnectionInfo", distanceSensor.getConnectionInfo());
        telemetry.addData("DeviceName", distanceSensor.getDeviceName());
        telemetry.addData("Manufacturer", distanceSensor.getManufacturer());
        telemetry.addData("Version", distanceSensor.getVersion());
        //telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));

//-------------------------------color sensor part (working)--------------------------------------------
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
        telemetry.addData("Red", test_color.red());
        telemetry.addData("Green", test_color.green());
        telemetry.addData("Blue", test_color.blue());



//-------------------------------touch sensor part (have some issues)-----------------------------------------
        if (test_touch.isPressed()){
            //Touch Sensor is pressed.
            telemetry.addData("Touch Sensor", "Is Pressed");
        } else {
            //Touch Sensor is not pressed

            telemetry.addData("Touch Sensor", "Is Not Pressed");
        }
        telemetry.update();
    }
}
