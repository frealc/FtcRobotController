package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Supplier;


/*
 * CE CODE UTILISE PEDRO PATHING. la methode est donc différente des autre mode auto
 */
@TeleOp(name = "DECODEpedro bleu", group = "DecodePedro")
public class teleopPedroB extends LinearOpMode {


    private DcMotorEx LeftFront, LeftBack, RightFront, RightBack;
    private DcMotorEx roueLanceur, roueLanceur1;
    private Servo pousseballe;
    private CRServo attrapeballe, roue_a_balle, chargement_manuel;

    private ShooterManager shooter;

    private Follower follower;
    private boolean automatedDrive = false;
    private Supplier<PathChain> ChargementBleu;
    private Supplier<PathChain> poseFrontB;
    private Supplier<PathChain> Ouverture;
    private Supplier<PathChain> poseBackB;

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
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");

        shooter = new ShooterManager(
                roueLanceur,
                roueLanceur1,
                pousseballe,
                attrapeballe,
                roue_a_balle
        );

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(SharedPose.finalPose);   // position de fin de l'auto
        follower.update();

        /*
         * creation des position
         */

        final Pose tirePoseFB = new Pose(-4.54, -2.68, -2.28);

        final Pose tirePoseBB = new Pose(55.38, -12, -2.73);
        final Pose ChargementB = new Pose(49.61, 54.23, -3.14);

        final Pose FaceOuverture = new Pose(-4.25,-40.13, -3.13);

        /*
         *creation des chemins du robot
         */

        poseFrontB = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(SharedPose.finalPose, tirePoseFB)))
                .setLinearHeadingInterpolation(SharedPose.finalPose.getHeading(), tirePoseFB.getHeading())
                .build();

        poseBackB = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(SharedPose.finalPose, tirePoseBB)))
                .setLinearHeadingInterpolation(SharedPose.finalPose.getHeading(), tirePoseBB.getHeading())
                .build();

        ChargementBleu = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(SharedPose.finalPose, ChargementB)))
                .setLinearHeadingInterpolation(SharedPose.finalPose.getHeading(), ChargementB.getHeading())
                .build();

        Ouverture = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(SharedPose.finalPose, FaceOuverture)))
                .setLinearHeadingInterpolation(SharedPose.finalPose.getHeading(), FaceOuverture.getHeading())
                .build();

        boolean PrecisionMode = false;
        double tgtpowerRota = 0;

        waitForStart();
        follower.startTeleopDrive(); //a peu ettre changer

        while (opModeIsActive()) {
            shooter.update();
            follower.update();

            // /////////////////////////////////////////////////////
            //              CONTROLE AUTOMATIQUE : DPAD UP / DOWN
            // /////////////////////////////////////////////////////



            if (!automatedDrive && gamepad1.a){
                follower.followPath(poseBackB.get());
                automatedDrive = true;
            }//quand a appuyer, vas a pos de tire arriere
            else if (!automatedDrive && gamepad1.y){
                follower.followPath(poseFrontB.get());
                automatedDrive = true;
            }//quand y appuyer, vas a pos de tire avant
            else if (!automatedDrive && gamepad1.x){
                follower.followPath(ChargementBleu.get());
                automatedDrive = true;
            }//quand b appuyer, vas devant la gate
            else if (!automatedDrive && gamepad1.b){
                follower.followPath(Ouverture.get());
                automatedDrive = true;
            }//quand x appuyer, vas a pos de joueur humain

            //arret manuelle quand le pad est appuyer
            if (automatedDrive) {
                if (!follower.isBusy() || gamepad1.dpad_down) {
                    follower.startTeleopDrive();
                    automatedDrive = false;
                }
            }

            //telemetry pendant le chemin auto
            if (automatedDrive) {
                telemetry.addLine("FOLLOW PATH EN COURS");
                telemetry.addData("vas vers : ", follower.getCurrentPath());
                telemetry.addData("position actuelle : ", follower.getPose());
                telemetry.update();
                continue;
            }



            /* *******************************************
             **********************************************
             * MANETTE 1 : PILOTE DEPLACEMENT
             **********************************************
             ********************************************** */

            double varY = gamepad1.left_stick_y;
            double varX = gamepad1.left_stick_x;

            double Power = varY;
            double strafe = varX;

            // Rotation avec triggers
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                tgtpowerRota = 0;
            } else if (gamepad1.left_trigger > 0) {
                tgtpowerRota = 1;
            } else if (gamepad1.right_trigger > 0) {
                tgtpowerRota = -1;
            } else {
                tgtpowerRota = 0;
            }

            //active le mode precision quand bumper droite est appuyé et le desactive quand bumper droite est re appuyé
            if (gamepad1.right_bumper) {
                PrecisionMode = !PrecisionMode;
                sleep(250);
            }

            double divisor = PrecisionMode ? 3.5 : 1.0;//quand precision activé, change le chiffre a 3, puis le repasse a 1 quand desactivé

            //gestion des moteur pour deplacement
            RightFront.setPower(-(Power + strafe - tgtpowerRota) / divisor);
            LeftFront.setPower(-(Power - strafe + tgtpowerRota) / divisor);
            RightBack.setPower(-(Power - strafe - tgtpowerRota) / divisor);
            LeftBack.setPower(-(Power + strafe + tgtpowerRota) / divisor);

            /* ************************************
             ***************************************
             * MANETTE 2 : PILOTE TIRE
             ***************************************
             * **************************************/

            if (gamepad2.right_trigger > 0) {
                roueLanceur.setVelocity(1615);
                roueLanceur1.setVelocity(1615);//fait tourné les roues de tire a un certains tick/s
            } else if (gamepad2.left_trigger > 0) {
                roueLanceur.setVelocity(1360);
                roueLanceur1.setVelocity(1360);
            } else if (gamepad2.dpad_left) {
                roueLanceur.setVelocity(-1275);
                roueLanceur1.setVelocity(-1275);// au cas ou une balle se block, fait tourné dans l'autre sens pour la sortir
            } else {
                shooter.stopShooter();
                shooter.update();
            }

            //faire tourné les elastique pour recup les balles
            if (gamepad2.x) {
                attrapeballe.setPower(1);
                roue_a_balle.setPower(1);
            } else if (gamepad2.a) {
                attrapeballe.setPower(-1);
                roue_a_balle.setPower(-1);
            } else if (gamepad2.b) {
                attrapeballe.setPower(1);
                roue_a_balle.setPower(1);
            } else {
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
            }

            //laisse passé les balles en montant la barre
            if (gamepad2.b || gamepad2.y) {
                pousseballe.setPosition(0.29);
            } else {
                pousseballe.setPosition(0.41);
            }

            chargement_manuel.setPower(-gamepad2.left_stick_x);//control la plaque ronde en bois pour faire tombé les balles

            // /////////////////////////////////////////////////////
            // TELEMETRY
            // /////////////////////////////////////////////////////

            telemetry.addData("AutoDrive", automatedDrive);
            telemetry.addData("Follower Pose", follower.getPose());
            telemetry.addData("Lanceur1", roueLanceur.getVelocity());
            telemetry.addData("Lanceur2", roueLanceur1.getVelocity());
            telemetry.addData("vitesse chargement manuelle", chargement_manuel.getPower());
            telemetry.addData("Y pour pos de lancé avant", 0);
            telemetry.addData("A pour pos de lancé arriere", 0);
            telemetry.update();
        }
    }
}
