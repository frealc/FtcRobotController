package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp
public class DECODEmanette extends LinearOpMode {
    private DcMotorEx LeftFront;
    private DcMotorEx LeftBack;
    private DcMotorEx RightFront;
    private DcMotorEx RightBack;
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private Servo pousseballe;

    private CRServo attrapeballe;

    private CRServo roue_a_balle;

    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        pousseballe = hardwareMap.get(Servo.class, "pousseballe");
        attrapeballe = hardwareMap.get(CRServo.class, "attrapeballe");
        roue_a_balle = hardwareMap.get(CRServo.class, "roue_a_balle");


        double tgtpowerRota = 0;

        double varY = 0;
        double varX = 0;


        double varYpos = 0;
        double varXpos = 0;



        boolean PrecisionMode = false; //precision mis en faut quand initialisé

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
            double Power = varYpos;
            double strafe = varXpos;
            double Rotate = tgtpowerRota;




            if (manette1.left_trigger > 0 && manette1.right_trigger > 0) {

                tgtpowerRota=0;

            }
            else if (manette1.left_trigger > 0) {

                tgtpowerRota=1;

            } else if (manette1.right_trigger > 0) {

                tgtpowerRota=-1;

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


            if(PrecisionMode) {
                RightFront.setPower(-(Power + strafe - Rotate)/3.5);
                LeftFront.setPower((Power - strafe + Rotate)/3.5);
                RightBack.setPower((Power - strafe - Rotate)/3.5);
                LeftBack.setPower(-(Power + strafe + Rotate)/3.5);
                //en mode precision, reduit la vitesse par 3.5
            }
            else {
                RightFront.setPower(-(Power + strafe - Rotate));
                LeftFront.setPower((Power - strafe + Rotate));
                RightBack.setPower((Power - strafe - Rotate));
                LeftBack.setPower(-(Power + strafe + Rotate));

            }



            if (manette2.b){
                roueLanceur.setPower(0.87);
                roueLanceur1.setPower(0.87);
            }
            else if (manette2.a){
                roueLanceur.setPower(0.65);
                roueLanceur1.setPower(0.65);

            }
            else{
                roueLanceur.setPower(0);
                roueLanceur1.setPower(0);
            }

            if (manette2.right_bumper){
                pousseballe.setPosition(0.28);

            }
            else{
                pousseballe.setPosition(0.40);
            }

            if (manette2.x){
                attrapeballe.setPower(1);
                roue_a_balle.setPower(1);
            }
            else if (manette2.y) {
                attrapeballe.setPower(-1);
                roue_a_balle.setPower(-1);
            }
            else{
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
            }


            telemetry.addData("vitesse moteur 1 du lanceur : ", roueLanceur.getVelocity());
            telemetry.addData("vitesse moteur 2 du lanceur : ", roueLanceur1.getVelocity());
            telemetry.addData("vitesse roue avant droite", RightFront.getVelocity());
            telemetry.addData("vitesse roue avant gauche", LeftFront.getVelocity());
            telemetry.addData("vitesse roue arriere droite", RightBack.getVelocity());
            telemetry.addData("vitesse roue arriere gauche", LeftBack.getVelocity());
            telemetry.update();
        }
    }
}
