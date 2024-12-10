package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="org.firstinspires.ftc.teamcode.AutonomeV1")
public class AutonomeV1 extends LinearOpMode {
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;

    public void avancer(double vitesse, long temps) {
        motorA.setPower(-vitesse);
        motorB.setPower(vitesse);
        motorC.setPower(-vitesse);
        motorD.setPower(vitesse);
        sleep(temps);
    }

    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorB = hardwareMap.get(DcMotor.class, "motorB");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        motorD = hardwareMap.get(DcMotor.class, "motorD");
        // Put initialization blocks here

        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);

        waitForStart();
        telemetry.addData("z :", "mode autonome initialisé");
        telemetry.update();

        while (opModeIsActive()) {

            avancer(1, 1000);

        }

    }

}
