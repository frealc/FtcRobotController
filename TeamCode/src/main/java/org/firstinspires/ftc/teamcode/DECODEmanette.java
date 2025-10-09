package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DECODEmanette extends LinearOpMode {
    private DcMotorEx LeftFront;
    private DcMotorEx LeftBack;
    private DcMotorEx RightFront;
    private DcMotorEx RightBack;
    private DcMotorEx roueLanceur;

    private Servo pousseballe;

    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");

        pousseballe = hardwareMap.get(Servo.class, "pousseballe");



        double tgtPowerA = 0;
        double tgtPowerB = 0;
        double tgtPowerC = 0;
        double tgtPowerD = 0;
        double tgtPowerA2 = 0;
        double tgtPowerB2 = 0;
        double tgtPowerC2 = 0;
        double tgtPowerD2 = 0;
        double tgtpowerRota = 0;

        double varY = 0;
        double varX = 0;


        double varYpos = 0;
        double varXpos = 0;

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
            varYpos = varY;
            varXpos = varX;


            /// Mouvements
            double Power = 0.5*varYpos;
            double strafe = 0.5*varXpos;
            double Rotate = 0.8*tgtpowerRota;




            if (manette1.left_trigger > 0 && manette1.right_trigger > 0) {

                tgtpowerRota=0;

            }
            else if (manette1.left_trigger > 0) {

                tgtpowerRota=0.8;

            } else if (manette1.right_trigger > 0) {

                tgtpowerRota=-0.8;

            }
            else {
                tgtpowerRota=0;
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
                RightFront.setPower(-(Power + strafe - Rotate) * 2);
                LeftFront.setPower(-(Power - strafe + Rotate)*2);
                RightBack.setPower(-(Power - strafe - Rotate)*2);
                LeftBack.setPower(-(Power + strafe + Rotate)*2);


            } else if(PrecisionMode) {
                RightFront.setPower(-(Power + strafe - Rotate)/3.5);
                LeftFront.setPower(-(Power - strafe + Rotate)/3.5);
                RightBack.setPower(-(Power - strafe - Rotate)/3.5);
                LeftBack.setPower(-(Power + strafe + Rotate)/3.5);
                //en mode precision, reduit la vitesse par 3.5
            }
            else {
                RightFront.setPower(-(Power + strafe - Rotate));
                LeftFront.setPower(-(Power - strafe + Rotate));
                RightBack.setPower(-(Power - strafe - Rotate));
                LeftBack.setPower(-(Power + strafe + Rotate));

            }



            if (manette2.b){
                pousseballe.setPosition(150);
                roueLanceur.setPower(10);

            }
            else if (manette2.a){
                roueLanceur.setPower(-10);


            }
            else{
                roueLanceur.setPower(0);

                pousseballe.setPosition(0);

            }
        }
    }
}
