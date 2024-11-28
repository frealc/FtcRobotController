package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class conduiteunemanetteV1 extends LinearOpMode {


    private DcMotorEx motorA;
    private DcMotorEx motorB;
    private DcMotorEx bras1;
    private DcMotorEx bras2;
    private Servo coudeA;

    private Servo pince;



    @Override
    public void runOpMode() {

        motorA = hardwareMap.get(DcMotorEx.class, "motorA");
        motorB = hardwareMap.get(DcMotorEx.class, "motorB");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coudeA = hardwareMap.get(Servo.class, "coudeA");
        pince = hardwareMap.get(Servo.class, "pince");


        double brasZero = 0.91;
        double brasX = brasZero;
        double tgtPowerA = 0;
        double tgtPowerB = 0;
        double tgtBras = 0;
        int maPosBras = 0;
        double varY = 0;
        double varX = 0;
        double varYpos = 0;
        double varXpos = 0;
        double varRY = 0;
        double debugTkt = 0;
        int brasA = 0;
        int bras0 = 0;


        Gamepad manette1 = this.gamepad1;
        Gamepad manette2 = this.gamepad2;

        waitForStart();


        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bras2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;

            // Convertion pour Moteurs
            varYpos = Math.abs(varY);
            varXpos = Math.abs(varX);

            // Récupération valeur joystick gauche
            varRY = manette2.right_stick_y;


            //prise des donné des joystick pour les moteur
            if (varY > 0) {
                tgtPowerA = varYpos;
                tgtPowerB = varYpos;
                if (varX < 0) {
                    tgtPowerB = tgtPowerB - varXpos;
                } else if (varX > 0) {
                    tgtPowerA = tgtPowerA - varXpos;
                }
            } else if (varY < 0) {
                tgtPowerA = -varYpos;
                tgtPowerB = -varYpos;

                debugTkt = -1;

                if (varX < 0) {
                    tgtPowerA = tgtPowerA + varXpos;
                } else if (varX > 0) {
                    tgtPowerB = tgtPowerB + varXpos;
                }
            } else if (varY == 0) {
                tgtPowerA = 0;
                tgtPowerB = 0;
            }

            if (varX > 0 && varY == 0) {
                tgtPowerA = -varXpos;
                tgtPowerB = varXpos;
            }

            if (varX < 0 && varY == 0) {
                tgtPowerA = varXpos;
                tgtPowerB = -varXpos;
            }


            if (manette1.left_bumper) {
                motorA.setPower(tgtPowerA);
                motorB.setPower(-tgtPowerB);
            } else {
                motorA.setPower((tgtPowerA / 2));
                motorB.setPower(-(tgtPowerB / 2));
            }


            //mouvement de bras a glissiere dans l'axe y
            if (varRY < 0) {
                tgtBras = varRY/3;
                bras0 = brasA;

            } else if (varRY > 0) {
                tgtBras = varRY/3;
                bras0 = brasA;
            } else {
                if (brasA > bras0) {
                    tgtBras = -0.1;
                } else if (brasA < bras0) {
                    tgtBras = 0.1;
                } else {
                    tgtBras = 0;
                }
            }
            bras1.setPower(tgtBras * 1.2);
            bras2.setPower(tgtBras);
            //temps pour monter :
            //temps pour descendre :


            if (manette2.left_trigger > 0) {

                brasX = 0.05;

            } else if (manette2.right_trigger > 0) {

                brasX = 0.90;
            }

            coudeA.setPosition(brasX);

            if (pince.getPosition() > 0.10) {
                while (manette2.a) {
                    pince.setPosition(0);
                }}
            if (pince.getPosition() < 0.2){
                while (manette2.a) {
                    pince.setPosition(1);
                }
            }

            telemetry.addData("Position Actuelle Bras", coudeA.getPosition());
            telemetry.update();
        }
    }
}