package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class DECODEmanette extends LinearOpMode {
    private DcMotorEx LeftFront;
    private DcMotorEx LeftBack;
    private DcMotorEx RightFront;
    private DcMotorEx RightBack;
    private DcMotorEx roueTest;



    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");
        roueTest = hardwareMap.get(DcMotorEx.class, "roueTest");


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


        double varY = 0;
        double varX = 0;
        double varRX = 0;
        double varRXpos = 0;
        double varYbras = 0;
        double varYpos = 0;
        double varXpos = 0;
        double varRY = 0;
        double debugTkt = 0;

        boolean PrecisionMode = false; //precision mis en faut quand initialisé
        double red = 0;
        double blue = 0;
        double dpad_up = 0;

        Gamepad manette1 = this.gamepad1;
        Gamepad manette2 = this.gamepad2;


        waitForStart();




        while (opModeIsActive()) {

            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;



            // Convertion pour Moteurs
            varYpos = Math.abs(varY);
            varXpos = Math.abs(varX);
            varRXpos = Math.abs(varRX);






            /// Mouvements
            if (varY > 0) {
                tgtPowerA = varYpos;
                tgtPowerB = -varYpos;
                tgtPowerC = varYpos;
                tgtPowerD = -varYpos;
                if (varX < 0) {
                    tgtPowerA = tgtPowerC * 2 + varXpos;
                    tgtPowerC = tgtPowerD * 2 + varXpos;
                } else if (varX > 0) {
                    tgtPowerB = tgtPowerA * 2 - varXpos;
                    tgtPowerD = tgtPowerB * 2 - varXpos;
                }
            } else if (varY < 0) {
                tgtPowerA = -varYpos;
                tgtPowerB = varYpos;
                tgtPowerC = -varYpos;
                tgtPowerD = varYpos;

                debugTkt = -1;

                if (varX < 0) {
                    tgtPowerA = tgtPowerD * 2 - varXpos;
                    tgtPowerC = tgtPowerC * 2 - varXpos;
                } else if (varX > 0) {
                    tgtPowerB = tgtPowerA * 2 + varXpos;
                    tgtPowerD = tgtPowerB * 2 + varXpos;
                }
            } else {
                tgtPowerA = 0;
                tgtPowerB = 0;
                tgtPowerC = 0;
                tgtPowerD = 0;
            }

            if (varX > 0 && varY == 0) {
                tgtPowerA = -varXpos*2;
                tgtPowerB = -varXpos*2;
                tgtPowerC = varXpos*2;
                tgtPowerD = varXpos*2;
            }

            if (varX < 0 && varY == 0) {
                tgtPowerA = varXpos*2;
                tgtPowerB = varXpos*2;
                tgtPowerC = -varXpos*2;
                tgtPowerD = -varXpos*2;
            }

            if (manette1.left_trigger > 0 && manette1.right_trigger > 0) {
                tgtPowerA2 = 0;
                tgtPowerB2 = 0;
                tgtPowerC2 = 0;
                tgtPowerD2 = 0;
            }
            else if (manette1.left_trigger > 0) {
                tgtPowerA = -1;
                tgtPowerB = 1;
                tgtPowerC = 1;
                tgtPowerD = -1;
            } else if (manette1.right_trigger > 0) {
                tgtPowerA = 1;
                tgtPowerB = -1;
                tgtPowerC = -1;
                tgtPowerD = 1;
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
                LeftFront.setPower(-tgtPowerA * 5);
                LeftBack.setPower(tgtPowerB * 5);
                RightFront.setPower(-tgtPowerC * 5);
                RightBack.setPower(tgtPowerD * 5);

                LeftFront.setPower(-tgtPowerA2 * 2);
                LeftBack.setPower(tgtPowerB2 * 2);
                RightFront.setPower(-tgtPowerC2 * 2);
                RightBack.setPower(tgtPowerD2 * 2);
            } else if(PrecisionMode) {
                LeftFront.setPower(-(tgtPowerA / 3.5));
                LeftBack.setPower((tgtPowerB / 3.5));
                RightFront.setPower(-(tgtPowerC / 3.5));
                RightBack.setPower((tgtPowerD / 3.5));

                LeftFront.setPower(-(tgtPowerA2 / 3.5));
                LeftBack.setPower((tgtPowerB2 / 3.5));
                RightFront.setPower(-(tgtPowerC2 / 3.5));
                RightBack.setPower((tgtPowerD2 / 3.5));//en mode precision, reduit la vitesse par 4
            }
            else {
                LeftFront.setPower(-(tgtPowerA ));
                LeftBack.setPower((tgtPowerB ));
                RightFront.setPower(-(tgtPowerC ));
                RightBack.setPower((tgtPowerD ));

                LeftFront.setPower(-(tgtPowerA2 / 2));
                LeftBack.setPower((tgtPowerB2 / 2));
                RightFront.setPower(-(tgtPowerC2 / 2));
                RightBack.setPower((tgtPowerD2 / 2));
            }



            if (manette2.b){
                roueTest.setPower(10);
            }
            else if (manette2.a){
                roueTest.setPower(-10);
            }
            else{
                roueTest.setPower(0);
            }
        }
    }
}