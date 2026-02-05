package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


/*
 *
 * CODE SANS UTILISATION DE DEAD WHEELS OU PEDRO PATHING
 *
 */
@TeleOp(name = "DECODEmanette bleu")
public class AutoEdit extends LinearOpMode {

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


    private Follower follower;
    VisionTest vision = new VisionTest();


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





        boolean PrecisionMode = false; //precision mis en faux quand initialisé


        Gamepad manette1 = this.gamepad1; //donne le nom "manette1 "
        Gamepad manette2 = this.gamepad2;


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(SharedPose.finalPose);   // position de fin de l'auto
        follower.update();



        waitForStart();


        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = VisionTest.getTagBySpecificId(30);

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

            //active le mode precision quand b est appuyé et le desactive quand b est re appuyé
            if (gamepad1.b) {
                PrecisionMode = !PrecisionMode;
                sleep(250);
            }

            double divisor = PrecisionMode ? 3.5 : 1.0; //quand precision activé, change le chiffre a 3, puis le repasse a 1 quand desactivé


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

                    //f(x) = 2.09375x + 837.5 (fonction de la vitesse (en tick/s) par rapport a la distance (en cm))



                    //bearing = lateral
                    //yaw = angle
                    //range = distance

                    //tire proche = 240cm --> 1340 tick/s
                    //tire loin == 400cm --> 1675 tick/s


                }
                //f = 2.09375*tag.ftcPose.range+837.5;


            }

            /*if (manette2.right_bumper){
                roueLanceur1.setVelocity(-f);
            }
            telemetry.addData("vitesse de F = ", -f); *///arrive pas a monté

            //gestion des moteur pour deplacement
            RightFront.setPower(-(Power + strafe - tgtpowerRota) / (divisor));
            LeftFront.setPower(-(Power - strafe + tgtpowerRota) / (divisor));
            RightBack.setPower(-(Power - strafe - tgtpowerRota) / (divisor));
            LeftBack.setPower(-(Power + strafe + tgtpowerRota) / (divisor+0.2));


            /* ************************************
             ***************************************
             * MANETTE 2 : PILOTE TIRE
             ***************************************
             * **************************************/

            if (manette2.right_trigger > 0) { //fait tourné les roues de tire a un certains tick/s
                roueLanceur.setVelocity(1675);
                roueLanceur1.setVelocity(-1675);
            } else if (manette2.left_trigger > 0) {
                roueLanceur.setVelocity(1360);
                roueLanceur1.setVelocity(-1360);
            } //PROBLEME AVEC CETTE METHODE :
            //le moteur dois allé a la vitesse max (1800 tick/s) avant de redescendre a la vitesse demandé

            else if (manette2.dpad_left) {
                roueLanceur.setVelocity(-1275);
                roueLanceur1.setVelocity(1275); // au cas ou une balle se block, fait tourné dans l'autre sens pour la sortir
            } else {
                roueLanceur.setPower(0);
                roueLanceur1.setPower(0);
            }

            //faire tourné les elastique pour recup les balles
            if (manette2.x) {
                attrapeballe.setPower(1);
                roue_a_balle.setPower(-1);
            } else if (manette2.dpad_down) {
                attrapeballe.setPower(-1);
                roue_a_balle.setPower(1);
            } else if (manette2.a) {
                attrapeballe.setPower(-1);
                roue_a_balle.setPower(1);
            } else if (manette2.b) {
                attrapeballe.setPower(1);
                roue_a_balle.setPower(-1);
            } else {
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
            }

            if (manette2.b || manette2.y) { //laisse passé les balles en montant la barre
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
