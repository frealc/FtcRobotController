package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


/*
 *
 * CODE SANS UTILISATION DE DEAD WHEELS OU PEDRO PATHING
 *
 */
@TeleOp(name = "DECODEmanette Bleu - 25017", group = "Alone")
public class DECODEmanetteBleu2 extends LinearOpMode {

    /**/
    private DcMotorEx LeftFront;
    private DcMotorEx LeftBack;
    private DcMotorEx RightFront;
    private DcMotorEx RightBack;
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private Servo pousseballe;

    private DcMotorEx attrapeballe;

    //private CRServo roue_a_balle;
    private CRServo chargement_manuel;

    private DcMotorEx Motsoulever;

    private DigitalChannel ledR;
    private DigitalChannel ledG;


    private Follower follower;
    VisionTest vision = new VisionTest();
    alldatacode data = new alldatacode();


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
        attrapeballe = hardwareMap.get(DcMotorEx.class, "attrapeballe");
        //roue_a_balle = hardwareMap.get(CRServo.class, "roue_a_balle");
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");
        Motsoulever = hardwareMap.get(DcMotorEx.class, "Motsoulever");

        ledR = hardwareMap.get(DigitalChannel.class, "ledR");
        ledG = hardwareMap.get(DigitalChannel.class, "ledG");


        //création des variables utilisées dans le code

        double yaw = 0;
        double range = 0;
        double bearing = 0;
        double f = 0;


        double tgtpowerRota = 0;
        double varY = 0;
        double varX = 0;


        double varYpos = 0;
        double varXpos = 0;

        vision.init(hardwareMap, telemetry);





        boolean PrecisionMode = false; //précision mis en faux quand initialisé


        Gamepad manette1 = this.gamepad1; //donne le nom "manette1 "
        Gamepad manette2 = this.gamepad2;


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(SharedPose.finalPose);   // position de fin de l'auto
        follower.update();



        waitForStart();

        ledR.setMode(DigitalChannel.Mode.OUTPUT);
        ledG.setMode(DigitalChannel.Mode.OUTPUT);


        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = VisionTest.getTagBySpecificId(20);

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




            if (manette1.right_stick_x == 0) { //rotation avec le joystick droit
                tgtpowerRota=0;
            }
            else {
                tgtpowerRota=manette1.right_stick_x;
            }

            //active le mode précision quand b est appuyé et le désactive quand b est re appuyé
            if (gamepad1.a) {
                PrecisionMode = !PrecisionMode;
                sleep(250);
            }

            double divisor = PrecisionMode ? 3.5 : 1.0; //quand précision activé, change le chiffre à 3, puis le repasse a 1 quand desactivé


            if (tag != null) {
                vision.update();

                yaw = tag.ftcPose.yaw;
                range = tag.ftcPose.range;
                bearing = tag.ftcPose.bearing;

                if(manette1.x) {

                    vision.update();

                    vision.update();
                    if (bearing >= 2 && divisor == 1) {
                        tgtpowerRota = -0.5/3.5;
                    } else if (bearing <= -2 && divisor == 1) {
                        tgtpowerRota = 0.5/3.5;
                    } else if (bearing >= 2 && divisor == 3.5){
                        tgtpowerRota = -0.5;
                    } else if (bearing <= -2 && divisor == 3.5){
                        tgtpowerRota = 0.5;
                    } else {
                        tgtpowerRota = 0;
                    }
                    vision.updateTelemetry();

                    //(fonction de la vitesse (en tick/s) par rapport a la distance (en cm))



                    //bearing = lateral
                    //yaw = angle
                    //range = distance

                    //tire proche = 240cm --> 1340 tick/s
                    //tire loin == 400cm --> 1675 tick/s

                    // Fonction 4 roues lanceurs
                    //f = (-1.736*Math.pow(10,-5))*Math.pow(range,3)+(0.01430229) * Math.pow(range, 2) + -1.814514 * range +1030.6704;

                    //Fonction lanceur rouge
                    f = 2.0765 * range + 870.63;
                }



            }

            //arrive pas à monter
            //gestion des moteurs pour déplacement
            RightFront.setPower(-(Power + strafe - tgtpowerRota) / (divisor));
            LeftFront.setPower(-(Power - strafe + tgtpowerRota) / (divisor));
            RightBack.setPower(-(Power - strafe - tgtpowerRota) / (divisor));
            LeftBack.setPower(-(Power + strafe + tgtpowerRota) / (divisor+0.2));


            if (manette1.dpad_down){
                while(Motsoulever.getCurrentPosition() <= 100) {
                    Motsoulever.setPower(1);
                    telemetry.addData("pos moteur soulever : ", Motsoulever.getCurrentPosition());
                    telemetry.update();
                    if (Motsoulever.getCurrentPosition() >= 105) {
                        Motsoulever.setPower(-0.4);
                    } else if (Motsoulever.getCurrentPosition() <= 99){
                        Motsoulever.setPower(1);
                    } else {
                        Motsoulever.setPower(0);
                        break;
                    }
                    if (manette1.dpad_up){
                        break;
                    }
                }

                Motsoulever.setPower(0);
            }
            if(manette1.dpad_up){
                while(Motsoulever.getCurrentPosition() >= 0){
                    Motsoulever.setPower(-0.5);
                    telemetry.addData("pos moteur soulever : ", Motsoulever.getCurrentPosition());
                    telemetry.update();
                }
                Motsoulever.setPower(0);
            }


            /* ************************************
             ***************************************
             * MANETTE 2 : PILOTE TIRE
             ***************************************
             * **************************************/

            if (manette2.right_trigger > 0) { //fait tourné les roues de tire a un certains tick/s
                roueLanceur.setVelocity(data.vitesse_de_tir_derriere);
                roueLanceur1.setVelocity(-data.vitesse_de_tir_derriere);
            } else if (manette2.left_bumper) {
                roueLanceur.setVelocity(data.vitesse_de_tir_devant);
                roueLanceur1.setVelocity(-data.vitesse_de_tir_devant);
            } //PROBLEME AVEC CETTE METHODE :
            //le moteur dois allé a la vitesse max (1800 tick/s) avant de redescendre a la vitesse demandé

            else if (manette2.dpad_left) {
                roueLanceur.setVelocity(-data.vitesse_retire_balle);
                roueLanceur1.setVelocity(data.vitesse_retire_balle); // au cas ou une balle se block, fait tourné dans l'autre sens pour la sortir
            }else if (manette2.left_trigger > 0){
                roueLanceur1.setVelocity(-f);
                roueLanceur .setVelocity(f);
            }
            else {
                roueLanceur.setPower(0);
                roueLanceur1.setPower(0);
            }
            telemetry.addData("vitesse de F = ", -f);

            if (roueLanceur.getVelocity() > f - 40 && roueLanceur.getVelocity() < f + 40){
                ledR.setState(true);
                ledG.setState(false);
            } else {
                ledR.setState(false);
                ledG.setState(true);
            }

            //faire tourné les elastique pour recup les balles
            if (manette2.x) {
                attrapeballe.setPower(-1);
                //roue_a_balle.setPower(-1);
            } else if (manette2.dpad_down) {
                attrapeballe.setPower(1);
                //roue_a_balle.setPower(1);
            } else if (manette2.a) {
                attrapeballe.setPower(0.5);
                //roue_a_balle.setPower(1);
            } else if (manette2.b) {
                attrapeballe.setPower(-1);
                //roue_a_balle.setPower(-1);
            } else if (manette2.dpad_right){
                attrapeballe.setPower(-0.5);
            }
            else {
                attrapeballe.setPower(0);
                //roue_a_balle.setPower(0);
            }

            if (manette2.b || manette2.y) { //laisse passé les balles en montant la barre
                pousseballe.setPosition(data.servo_moteur_angle_haut);
            } else {
                pousseballe.setPosition(data.servo_moteur_angle_bas);
            }

            chargement_manuel.setPower(-manette2.left_stick_x); //control la plaque ronde en bois pour faire tombé les balles




            /*
             * TELEMETRY (affichage de données sur la tablette pour débug)
             */

            if (roueLanceur1.getVelocity() > f -5 && roueLanceur1.getVelocity() < f + 5){
                telemetry.addLine("LANCEUR PRET A TIRER!!!!!");
                telemetry.update();
            }
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
