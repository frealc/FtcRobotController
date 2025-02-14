package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class conduite2manette extends LinearOpMode {
    private DcMotorEx motorA;
    private DcMotorEx motorB;
    private DcMotorEx motorC;
    private DcMotorEx motorD;
    private DcMotorEx bras1;
    private DcMotorEx coudeA;
    private DcMotorEx ascent;
    private Servo boite;
    private DcMotorEx coudeB;
    private Servo pince;

    private Servo pinceP;
    private Servo rotapinceP;

    private Servo verin;

    private double brasA=0;


    @Override
    public void runOpMode() {

        motorA = hardwareMap.get(DcMotorEx.class, "motorA");
        motorB = hardwareMap.get(DcMotorEx.class, "motorB");
        motorC = hardwareMap.get(DcMotorEx.class, "motorC");
        motorD = hardwareMap.get(DcMotorEx.class, "motorD");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        coudeA = hardwareMap.get(DcMotorEx.class, "coudeA");
        coudeB = hardwareMap.get(DcMotorEx.class, "coudeB");
        ascent = hardwareMap.get(DcMotorEx.class, "ascent");
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
        double tgtBrasG = 0;
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
        double varRYcoude = 0;
        double varRYcoudepos = 0;
        double debugTkt = 0;
        int brasA = 0;
        int bras0 = 0;
        int coudeApos = 0;
        int coude0 = 0;
        boolean PrecisionMode = false; //precision mis en faut quand initialisé
        double red = 0;
        double blue=0;

        Gamepad manette1 = this.gamepad1;
        Gamepad manette2 = this.gamepad2;


        waitForStart();



        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coudeA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coudeB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

            varRYcoude = manette2.right_stick_y;
            coudeApos = coudeA.getCurrentPosition();


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


            if (PrecisionMode){
                while (manette1.b){
                    PrecisionMode = false;
                }
            } else {
                while (manette1.b){
                    PrecisionMode = true;
                }
            }//active le mode precision quand B est appuyé et le desactive quand B est re appuyé

            if (manette1.a) {
                motorA.setPower(-tgtPowerA * 5);
                motorB.setPower(tgtPowerB * 5);
                motorC.setPower(-tgtPowerC * 5);
                motorD.setPower(tgtPowerD * 5);

                motorA.setPower(-tgtPowerA2 * 2);
                motorB.setPower(tgtPowerB2 * 2);
                motorC.setPower(-tgtPowerC2 * 2);
                motorD.setPower(tgtPowerD2 * 2);
            } else if(PrecisionMode) {
                motorA.setPower(-(tgtPowerA / 3.5));
                motorB.setPower((tgtPowerB / 3.5));
                motorC.setPower(-(tgtPowerC / 3.5));
                motorD.setPower((tgtPowerD / 3.5));

                motorA.setPower(-(tgtPowerA2 / 3.5));
                motorB.setPower((tgtPowerB2 / 3.5));
                motorC.setPower(-(tgtPowerC2 / 3.5));
                motorD.setPower((tgtPowerD2 / 3.5));//en mode precision, reduit la vitesse par 4
            }
            else {
                motorA.setPower(-(tgtPowerA / 1.4));
                motorB.setPower((tgtPowerB / 1.4));
                motorC.setPower(-(tgtPowerC / 1.4));
                motorD.setPower((tgtPowerD / 1.4));

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
                    tgtBras = -0.07;
                } else if (brasA < bras0) {
                    tgtBras = 0.07;
                } else {
                    tgtBras = 0;
                }
            }

            if (manette2.left_trigger > 0){
            bras1.setPower(tgtBras * 6);
            ascent.setPower((varRY/3) * 6.4);
            } else {
                bras1.setPower(tgtBras * 1.2);
                ascent.setPower((varRY/3) * 1.4);
            }//code glissiere


            if (varRYcoude > 0) {

                varRYcoudepos = -Math.abs(varRYcoude);
                coude0 = coudeA.getCurrentPosition();


            } else if (varRYcoude < 0) {

                varRYcoudepos = Math.abs(varRYcoude);
                coude0 = coudeA.getCurrentPosition();

            }
            /*else {
                if  (coude0 < coudeA.getCurrentPosition()) {
                    coudeA.setPower(0.01);
                    coudeB.setPower(0.01);


                } else if (coude0 > coudeA.getCurrentPosition()) {
                    coudeA.setPower(-0.01);
                    coudeB.setPower(-0.01);

                }*/ else {
                varRYcoudepos = 0;
            } //regler le probleme de gravité du bras


            coudeA.setPower(varRYcoudepos); //code coude
            coudeB.setPower(varRYcoudepos);


            if (manette1.left_bumper) {
                ascent.setPower(-9);
            }
            else if (manette1.right_bumper){
                ascent.setPower(9);
            }
            else {
                ascent.setPower(0);
            }

            // Pas de capteur de couleur
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
                while (manette2.right_bumper) {

                    rotapinceP.setPosition(0);
                }
            }
            if (rotapinceP.getPosition() < 0.5){
                while (manette2.left_bumper) {

                    rotapinceP.setPosition(0.85);
                }
            }


            if (pinceP.getPosition() > 0.7) {
                while (manette2.y) {
                    pinceP.setPosition(0.4);
                }
            }
            if (pinceP.getPosition() < 0.7){
                while (manette2.y) {
                    pinceP.setPosition(1);
                }
            }


            /*if (manette2.dpad_down && brasA < -1135) {
                brasA = bras1.getCurrentPosition();
                while (brasA < -1130){
                    brasA = bras1.getCurrentPosition();
                    bras1.setPower(0.7 * 1.3);
                    telemetry.addData("Position Actuelle BRASA (test)", brasA);
                    telemetry.update();
                }
                pince.setPosition(0);
            }*/




            telemetry.addData("Position Actuelle Bras", brasX);
            telemetry.addData("Position Actuelle Coude", coudeApos);
            telemetry.addData("Position Actuelle BRASA (test)", brasA);
            telemetry.addData("Position Actuelle pince", pince.getPosition());
            telemetry.addData("Position Actuelle boite", boite.getPosition());
            telemetry.addData("Position fil",ascent.getCurrentPosition());
            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Target Power C", tgtPowerC);
            telemetry.addData("Target Power D", tgtPowerD);
            telemetry.addData("Joystick Gauche : VarY", varRY);
            telemetry.addData("ma pos coude", coude0);
            telemetry.addData("coude a pos", coudeApos);
            telemetry.update();
        }
    }
}
