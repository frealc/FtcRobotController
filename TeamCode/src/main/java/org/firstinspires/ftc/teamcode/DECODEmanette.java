package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


/*
*
* CODE SANS UTILISATION DE DEAD WHEELS OU PEDRO PATHING
*
*/
@TeleOp
public class DECODEmanette extends LinearOpMode {

    /**/
    private DcMotorEx LeftFront;
    private DcMotorEx LeftBack;
    private DcMotorEx RightFront;
    private DcMotorEx RightBack;
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private Servo pousseballe;

    private CRServo attrapeballe;

    private CRServo roue_a_balle;
    private CRServo chargement_manuel;

    @Override
    public void runOpMode() {

        /*
        *associe les nom données a ce qui est brancher sur le control hub
        *Bien pensé a mettre les moteur, servo etc... dans le driver hub (... --> configure Robot --> edit .....)
        */

        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        pousseballe = hardwareMap.get(Servo.class, "pousseballe");
        attrapeballe = hardwareMap.get(CRServo.class, "attrapeballe");
        roue_a_balle = hardwareMap.get(CRServo.class, "roue_a_balle");
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");


        //creation des variables utilisé dans le code

        double tgtpowerRota = 0;
        double varY = 0;
        double varX = 0;


        double varYpos = 0;
        double varXpos = 0;



        boolean PrecisionMode = false; //precision mis en faux quand initialisé


        Gamepad manette1 = this.gamepad1; //donne le nom "manette1 "
        Gamepad manette2 = this.gamepad2;


        waitForStart();




        while (opModeIsActive()) {

            /* *******************************************
            **********************************************
            * MANETTE 1 : PILOTE DEPLACEMENT
            **********************************************
            ********************************************** */

            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;



            // utilisé sur un ancien code pour les mettre en valeur absolue, inutile ici
            //varYpos = varY;
            //varXpos = varX;


            double Power = varY;
            double strafe = varX;
            double Rotate = tgtpowerRota;




            if (manette1.left_trigger > 0 && manette1.right_trigger > 0) { //rotation avec les trigger
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
            //active le mode precision quand B est appuyé et le desactive quand B est re appuyé
            if (PrecisionMode){
                while (manette1.b){
                    PrecisionMode = false;
                }
            } else {
                while (manette1.b){
                    PrecisionMode = true;
                }
            }

            //gestion des moteurs pour le déplacement
            if(PrecisionMode) {
                //en mode precision, reduit la vitesse par 4
                RightFront.setPower(-(Power + strafe - Rotate)/4);
                LeftFront.setPower((Power - strafe + Rotate)/4);
                RightBack.setPower((Power - strafe - Rotate)/4);
                LeftBack.setPower(-(Power + strafe + Rotate)/4);
            }
            else {
                RightFront.setPower(-(Power + strafe - (Rotate/1.5)));
                LeftFront.setPower((Power - strafe + (Rotate/1.5)));
                RightBack.setPower((Power - strafe - (Rotate/1.5)));
                LeftBack.setPower(-(Power + strafe + (Rotate/1.5)));
            }


            /* ************************************
            ***************************************
            * MANETTE 2 : PILOTE TIRE
            ***************************************
            * **************************************/

            if (manette2.right_trigger > 0) { //fait tourné les roues de tire a un certains tick/s
                roueLanceur.setVelocity(1675);
                roueLanceur1.setVelocity(1675);
            } else if (manette2.left_trigger > 0) {
                roueLanceur.setVelocity(1360);
                roueLanceur1.setVelocity(1360);
            } //PROBLEME AVEC CETTE METHODE :
            //le moteur dois allé a la vitesse max (1800 tick/s) avant de redescendre a la vitesse demandé

            else if (manette2.dpad_left) {
                roueLanceur.setVelocity(-1275);
                roueLanceur1.setVelocity(-1275); // au cas ou une balle se block, fait tourné dans l'autre sens pour la sortir
            } else {
                roueLanceur.setPower(0);
                roueLanceur1.setPower(0);
            }

            //faire tourné les elastique pour recup les balles
            if (manette2.x) {
                attrapeballe.setPower(1);
                roue_a_balle.setPower(1);
            } else if (manette2.dpad_down) {
                attrapeballe.setPower(-1);
                roue_a_balle.setPower(-1);
            } else if (manette2.a) {
                attrapeballe.setPower(-1);
                roue_a_balle.setPower(-1);
            } else if (manette2.b) {
                attrapeballe.setPower(1);
                roue_a_balle.setPower(1);
            } else {
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
            }

            if (manette2.a || manette2.b || manette2.y) { //laisse passé les balles en montant la barre
                pousseballe.setPosition(0.29);
            } else {
                pousseballe.setPosition(0.41);
            }

            chargement_manuel.setPower(-manette2.left_stick_x); //control la plaque ronde en bois pour faire tombé les balles


            /*
            * TELEMETRY (affichage de données sur la tablette pour débug)
             */
            telemetry.addData("vitesse moteur 1 du lanceur : ", roueLanceur.getVelocity());
            telemetry.addData("vitesse moteur 2 du lanceur : ", roueLanceur1.getVelocity());
            telemetry.addData("vitesse roue avant droite", RightFront.getVelocity());
            telemetry.addData("vitesse roue avant gauche", LeftFront.getVelocity());
            telemetry.addData("vitesse roue arriere droite", RightBack.getVelocity());
            telemetry.addData("vitesse roue arriere gauche", LeftBack.getVelocity());
            telemetry.addData("vitesse chargement manuelle", chargement_manuel.getPower());
            telemetry.update();
        }
    }
}
