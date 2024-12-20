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
    private DcMotorEx motorC;
    private DcMotorEx motorD;
    private DcMotorEx bras1;
    private DcMotorEx bras2;
    private DcMotorEx coudeA;
    private Servo boite;

    private Servo pince;

    private Servo pinceP;
    private Servo rotapinceP;



    @Override
    public void runOpMode() {

        motorA = hardwareMap.get(DcMotorEx.class, "motorA");
        motorB = hardwareMap.get(DcMotorEx.class, "motorB");
        motorC = hardwareMap.get(DcMotorEx.class, "motorC");
        motorD = hardwareMap.get(DcMotorEx.class, "motorD");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coudeA = hardwareMap.get(DcMotorEx.class, "coudeA");
        pince = hardwareMap.get(Servo.class, "pince");
        pinceP = hardwareMap.get(Servo.class, "pinceP");
        boite = hardwareMap.get(Servo.class, "boite");
        rotapinceP = hardwareMap.get(Servo.class, "rotapinceP");


        double brasZero = 0.91;
        double brasX = brasZero;
        double tgtPowerA = 0;
        double tgtPowerB = 0;
        double tgtPowerC = 0;
        double tgtPowerD = 0;
        double tgtPowerA2 = 0;
        double tgtPowerB2 = 0;
        double tgtPowerC2 = 0;
        double tgtPowerD2 = 0;
        double tgtBras = 0;
        int maPosBras = 0;
        double varY = 0;
        double varX = 0;
        double varRX = 0;
        double varRXpos = 0;
        double varYbras = 0;
        double varYpos = 0;
        double varXpos = 0;
        double varYbraspos = 0;
        double varRY = 0;
        double debugTkt = 0;
        int brasA = 0;
        int bras0 = 0;
        double maPosCoude = 0;


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
            varRXpos = Math.abs(varRX);


            // Récupération valeur joystick gauche
            varRY = manette2.left_stick_y;
            brasA= bras1.getCurrentPosition();
            varYbras = manette2.left_stick_y;
            varYbraspos = Math.abs(varYbras);


            /// Mouvements
            if (varY > 0) {
                tgtPowerA = varYpos;
                tgtPowerB = varYpos;
                tgtPowerC = varYpos;
                tgtPowerD = varYpos;
                if (varX < 0) {
                    tgtPowerB = tgtPowerB - varXpos;
                    tgtPowerD = tgtPowerD - varXpos;
                } else if (varX > 0) {
                    tgtPowerA = tgtPowerA - varXpos;
                    tgtPowerC = tgtPowerC - varXpos;
                }
            } else if (varY < 0) {
                tgtPowerA = -varYpos;
                tgtPowerB = -varYpos;
                tgtPowerC = -varYpos;
                tgtPowerD = -varYpos;

                debugTkt = -1;

                if (varX < 0) {
                    tgtPowerA = tgtPowerA + varXpos;
                    tgtPowerC = tgtPowerC + varXpos;
                } else if (varX > 0) {
                    tgtPowerB = tgtPowerB + varXpos;
                    tgtPowerD = tgtPowerD + varXpos;
                }
            } else if (varY == 0) {
                tgtPowerA = 0;
                tgtPowerB = 0;
                tgtPowerC = 0;
                tgtPowerD = 0;
            }

            if (varX > 0 && varY == 0) {
                tgtPowerA = -varXpos;
                tgtPowerB = varXpos;
                tgtPowerC = -varXpos;
                tgtPowerD = varXpos;
            }

            if (varX < 0 && varY == 0) {
                tgtPowerA = varXpos;
                tgtPowerB = -varXpos;
                tgtPowerC = varXpos;
                tgtPowerD = -varXpos;
            }

            if (manette1.left_trigger > 0 && manette1.right_trigger > 0) {
                tgtPowerA2 = 0;
                tgtPowerB2 = 0;
                tgtPowerC2 = 0;
                tgtPowerD2 = 0;
            }
            else if (manette1.right_trigger > 0) {
                tgtPowerA2 = 0.56;
                tgtPowerB2 = -0.56;
                tgtPowerC2 = -0.56;
                tgtPowerD2 = 0.56;//strafe gauche
            } else if (manette1.left_trigger > 0) {
                tgtPowerA2 = -0.56;
                tgtPowerB2 = 0.56;
                tgtPowerC2 = 0.56;
                tgtPowerD2 = -0.56; //strafe droit
            }
            //prise des valeur du joystick gauche pour faire les strafe et avancer reculé
            else {
                tgtPowerA2 = 0;
                tgtPowerB2 = 0;
                tgtPowerC2 = 0;
                tgtPowerD2 = 0;
            }

            if (manette1.a) {
                motorA.setPower(-tgtPowerA * 1.2);
                motorB.setPower(tgtPowerB * 1.2);
                motorC.setPower(-tgtPowerC * 1.2);
                motorD.setPower(tgtPowerD * 1.2);

                motorA.setPower(-tgtPowerA2);
                motorB.setPower(tgtPowerB2);
                motorC.setPower(-tgtPowerC2);
                motorD.setPower(tgtPowerD2);
            } else {
                motorA.setPower(-(tgtPowerA / 2));
                motorB.setPower((tgtPowerB / 2));
                motorC.setPower(-(tgtPowerC / 2));
                motorD.setPower((tgtPowerD / 2));

                motorA.setPower(-(tgtPowerA2 / 2));
                motorB.setPower((tgtPowerB2 / 2));
                motorC.setPower(-(tgtPowerC2 / 2));
                motorD.setPower((tgtPowerD2 / 2));
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

            if (manette2.left_bumper){
            bras1.setPower(tgtBras * 3);
            bras2.setPower(tgtBras * 3);
            } else {
                bras1.setPower(tgtBras * 1.2);
                bras2.setPower(tgtBras * 1.2);
            }//code glissiere


            if (manette2.right_trigger > 0) {

                brasX = -0.5;

            } else if (manette2.left_trigger > 0) {

                brasX = 0.5;
            }
            else {
                brasX = 0;
            } //regler le probleme de gravité du bras


            coudeA.setPower(brasX); //code coude


            if (pince.getPosition() > 0.1) {
                while (manette2.a) {
                    pince.setPosition(0);
                }
            }

            if (pince.getPosition() < 0.20){
                while (manette2.a) {
                    pince.setPosition(1);
                }
            }


            if (boite.getPosition() > 0.1) {
                while (manette2.b) {
                    boite.setPosition(0);
                }
            }
            if (boite.getPosition() < 0.2){
                while (manette2.b) {
                    boite.setPosition(1);
                }
            }


            if (rotapinceP.getPosition() > 0.5) {
                while (manette2.x) {
                    rotapinceP.setPosition(0);
                }
            }
            if (rotapinceP.getPosition() < 0.5){
                while (manette2.x) {
                    rotapinceP.setPosition(1);
                }
            }


            if (pinceP.getPosition() > 0.7) {
                while (manette2.y) {
                    pinceP.setPosition(0.5);
                }
            }
            if (pinceP.getPosition() < 0.7){
                while (manette2.y) {
                    pinceP.setPosition(0.98);
                }
            }


            telemetry.addData("Position Actuelle Bras", brasX);
            telemetry.addData("Position Actuelle pince", pince.getPosition());
            telemetry.addData("Position Actuelle boite", boite.getPosition());
            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Target Power C", tgtPowerC);
            telemetry.addData("Target Power D", tgtPowerD);
            telemetry.addData("Joystick Gauche : VarY", varRY);
            telemetry.update();
        }
    }
}
