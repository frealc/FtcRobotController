package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Test encodeur")
public class testregulateur extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx m = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "rouelanceur");

        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        m.setVelocity(1700);
        m2.setVelocity(1700);

        while (opModeIsActive()) {
            telemetry.addData("Velocity", m.getVelocity());
            telemetry.addData("Position", m.getCurrentPosition());
            telemetry.update();
        }
    }
}
