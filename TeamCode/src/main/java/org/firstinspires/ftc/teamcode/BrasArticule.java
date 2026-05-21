package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class BrasArticule extends LinearOpMode {

    private DcMotorEx motorA;

    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotorEx.class, "moteur1");
        //motorB = hardwareMap.get(DcMotorEx.class, "moteur2");




        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double coudeZero = 0.521;
        double coudeX = coudeZero;
        int brasA = 0;
        int brasB = 0;
        double trigger = 0;
        double varRY = 0;


        while (opModeIsActive()) {
            varRY = this.gamepad1.right_stick_y;
            trigger = this.gamepad1.right_trigger;


            if (this.gamepad1.dpad_up) {
                coudeX += 0.002;
                if (coudeX > 1) {
                    coudeX = 1;
                }
            } else if (this.gamepad1.dpad_down) {
                coudeX -= 0.002;
                if (coudeX<0) {
                    coudeX = 0;
                }
            } else {
                coudeX = coudeZero;
            }

            motorA.setPower(0.5);

            telemetry.addData("Var Y", varRY);
            telemetry.update();

        }
    }

}
