package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servotest extends LinearOpMode {
    private Servo coudeA;




    @Override
    public void runOpMode() {
        coudeA = hardwareMap.get(Servo.class, "coudeA");

        waitForStart();


        double brasZero = 0.91;
        double brasX = brasZero;
        double braspas = 0.003;

        Gamepad manette1 = this.gamepad1;

        while (opModeIsActive()) {


            if (manette1.left_trigger > 0) {

                    brasX = -0.80;

            } else if (manette1.right_trigger > 0) {

                brasX = 0.80;
            }

                coudeA.setPosition(brasX);

                telemetry.addData("Position Actuelle Bras", coudeA.getPosition());
                telemetry.update();
            }
        }
    }
