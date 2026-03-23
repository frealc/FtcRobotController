package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


/*
 * CE CODE UTILISE PEDRO PATHING. la methode est donc différente des autre mode auto
 */

@Autonomous(name = "auto back rouge JH", group = "rouge")
public class autobackRtest extends OpMode {

    //gestion du shooter (utilise le code "shooter manager")

    private long startTime = 0;

    int shotCount = 0;
    boolean wasShooterReady = false;
    static final double SHOOTER_HIGH = 1680;
    static final double SHOOTER_READY = 1580;
    static final double SHOOTER_LOW   = 1490;

    // Hardware
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private DcMotorEx attrapeballe;

    private Servo pousseballe;

    private CRServo chargement_manuel;

    boolean timerStarted = false;
    private Follower follower;


    /*
     *PEDRO PATHING
     */
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    /*
     *creation des positions utilisé sur le terrain
     *utilisé le code tuning (init puis right bumper 2 fois pour allé a Localization Test)
     * --> Localization --> Localization Test puis lancé le code
     */

    private final Pose startPose = new Pose(59, 15, 3.17);
    private final Pose tirePose = new Pose(52, 12, 2.78);

    private final Pose rotatest = new Pose(36, 19,-1.57);

    private final Pose priseballe1 = new Pose(36, 54,-1.57);

    private final Pose replace2 = new Pose(36, 61,3.14);

    private final Pose priseballe2 = new Pose(53, 61,3.14);
    private final Pose poseFinal = new Pose(55, 30, 2.78);

    boolean shotLocked = false;
    VisionTest vision = new VisionTest();

    /*
     *creation des nom pour les chemins du robot
     */
    private PathChain tire, gotopose1, takepose1, lance2, gotopose2, takepose2, lance3, fin;


    @Override
    public void init() {
        // ---- HARDWARE ----

        attrapeballe = hardwareMap.get(DcMotorEx.class, "attrapeballe");
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        pousseballe = hardwareMap.get(Servo.class, "pousseballe");




        vision.init(hardwareMap, telemetry);

        // ---- FOLLOWER / PATHS ----
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        pathTimer = new Timer();
    }

    /*
     *creation des chemins du robot
     * bien pensé a mettre un nom dans le pathChain avant de crée un chemin
     *
     * methode :  mettre un .addPath et dire le type de chemin, Bezier Line ou Curve.
     * Line (2 position a rensègner): fait un chemin en ligne d'une position a une autre
     * curve (3 position a rensègner): fait une courbe d'une position a une autre en passant par un point
     * 1ere pos : position de départ, 2eme pos : position de passage de la courbe, 3eme pos : position de fin
     *
     * ensuite mettre le .setLinearHeadingInterpolation
     * sert a mettre l'orientation de debut et de fin du robot
     */
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(tirePose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        tire = follower.pathBuilder()
                .addPath(new BezierLine(startPose, tirePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), tirePose.getHeading())
                .build();

        gotopose1 = follower.pathBuilder()
                .addPath(new BezierLine(tirePose, rotatest))
                .setLinearHeadingInterpolation(tirePose.getHeading(), rotatest.getHeading())
                .build();

        takepose1 = follower.pathBuilder()
                .addPath(new BezierLine(rotatest, priseballe1))
                .setLinearHeadingInterpolation(rotatest.getHeading(), priseballe1.getHeading())
                .build();


        lance2 = follower.pathBuilder()
                .addPath(new BezierLine(priseballe1, tirePose))
                .setLinearHeadingInterpolation(priseballe1.getHeading(), tirePose.getHeading())
                .build();

        gotopose2 = follower.pathBuilder()
                .addPath(new BezierLine(tirePose, replace2))
                .setLinearHeadingInterpolation(tirePose.getHeading(), replace2.getHeading())
                .build();

        takepose2 = follower.pathBuilder()
                .addPath(new BezierLine(replace2,priseballe2))
                .setLinearHeadingInterpolation(replace2.getHeading(), priseballe2.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        lance3 =    follower.pathBuilder()
                .addPath(new BezierLine(priseballe2, tirePose))
                .setLinearHeadingInterpolation(priseballe2.getHeading(), tirePose.getHeading())
                .build();
        fin =    follower.pathBuilder()
                .addPath(new BezierLine(tirePose, poseFinal))
                .setLinearHeadingInterpolation(tirePose.getHeading(), poseFinal.getHeading())
                .build();

    }

    //initialise les timer
    @Override
    public void start() {
        pathState = 0;
        startTime = System.currentTimeMillis();
        pathTimer.resetTimer();
    }


    //met des debug et update les etapes en boucle
    @Override
    public void loop() {
        // Update shooter et follower

        follower.update();

        vision.update();
        AprilTagDetection tag = VisionTest.getTagBySpecificId(24);
        if (tag != null) {
            vision.update();


            double range = tag.ftcPose.range;


            double f = 0.0023 * Math.pow(range, 2) + 0.35 * range + 1121;
            telemetry.addData("vitesse de F = ", f);
        }

        // Mise à jour des paths
        autonomousPathUpdate();

        // Debug
        telemetry.addData("State", pathState);
        telemetry.addData("Shooter RPM", roueLanceur.getVelocity());
        telemetry.update();

        Drawing.drawDebug(follower); //visualisation sur le panel pedro (192.168.43.1:8001 pour acceder au panel)
    }


    /*
     * Debut des chemin
     */
    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if(!follower.isBusy()) {
                    //shooter.startShooter(1615);
                    roueLanceur1.setVelocity(-1600);
                    roueLanceur.setVelocity(1600);// lance les roues de tire a un Tick/s ciblé
                    //follower.followPath(tire, true);//vas a la position de tire
                    startTime = System.currentTimeMillis();//lance un timer
                    setPathState(1);//passe a la prochaine étape
                }
                break;

            case 1 :

                if(!follower.isBusy()){

                    //if( System.currentTimeMillis() - startTime >= 1500) {
                    follower.followPath(tire, true);
                    setPathState(2);
                    //}
                }

            case 2:
                if (System.currentTimeMillis() - startTime >= 2000) {
                    if (!follower.isBusy()) {

                        attrapeballe.setPower(-0.15);
                        follower.setMaxPower(1);

                        double velocity = roueLanceur.getVelocity();
                        telemetry.addData("Shooter velocity", velocity);
                        telemetry.addData("Shots", shotCount);


                        //attrapeballe.setPower(-0.5);
                        //roue_a_balle.setPower(-0.5);
                        pousseballe.setPosition(0.28);

                        /*
                         * gestion de la plaque tournante avec vitesse moteur
                         */
                        if (velocity >= SHOOTER_READY && !shotLocked) {
                            chargement_manuel.setPower(0.25);//si les moteur sont pareil que shooter ready, fait tourné la plaque
                        } else {
                            chargement_manuel.setPower(0);
                        }


                        if (velocity <= SHOOTER_LOW && !shotLocked) {
                            shotCount++;
                            shotLocked = true;
                            /*
                             *quand la premiere balle est tiré, moteur baisse en vitesse
                             * ajout 1 au compte de balle tiré
                             */
                        }


                        if (velocity >= SHOOTER_READY) {
                            shotLocked = false;
                        }


                        if (shotCount >= 4 || System.currentTimeMillis() - startTime >= 10000) {
                            chargement_manuel.setPower(0);// quant toute les balles sont tiré ou 10s sont passé, arrete tout
                            shotLocked = false;
                            setPathState(3);//passe a la prochaine étape
                        }
                    }
                }
                break;



            case 3:
                roueLanceur1.setPower(0);
                roueLanceur.setPower(0);
                attrapeballe.setPower(0);
                chargement_manuel.setPower(0);
                pousseballe.setPosition(0.5);
                // Lancer le path suivant
                follower.followPath(gotopose1, true); //se pose devant les balles au sol (a probablement enlevé)
                setPathState(4);
                break;

            case 4:

                if (!follower.isBusy()) {
                    attrapeballe.setPower(-1);

                    follower.followPath(takepose1, true); //rammasse les balle
                    setPathState(5);
                }
                break;
            case 5:

                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    setPathState(6);
                }
                break;

            case 6:

                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roueLanceur1.setVelocity(-1560);
                    roueLanceur.setVelocity(1560);
                    follower.followPath(lance2, true);//vas a la pos de tire
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(7);


                }
                break;
            case 7:
                if (!follower.isBusy()) {

                    roueLanceur1.setVelocity(-1560);
                    roueLanceur.setVelocity(1560);
                    attrapeballe.setPower(-1);
                    pousseballe.setPosition(0.28); //commence a tiré
                    if (System.currentTimeMillis() - startTime >= 6000) {
                        roueLanceur1.setVelocity(-900);
                        roueLanceur.setVelocity(900);
                        attrapeballe.setPower(0);
                        pousseballe.setPosition(0.5);
                        // Lancer le path suivant
                        follower.followPath(gotopose2, true);
                        startTime = System.currentTimeMillis();
                        setPathState(8); //apres 5s passé au total, passe a la prochaine etape
                    }
                }
                break;
            case 8:

                if (!follower.isBusy()) {
                    attrapeballe.setPower(-1);
                    follower.followPath(takepose2, true); // rammasse les balles
                    if (System.currentTimeMillis() - startTime >= 3000) {
                        setPathState(9);
                    }

                }
                break;
            case 9:

                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roueLanceur1.setVelocity(-1580);
                    roueLanceur.setVelocity(1580);//prepare le tire (a changé pour utilisé le start shooter)

                    follower.followPath(lance3, true);//vas a la position de tire
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(10);

                }
                break;
            case 10:
                // Attendre 2 secondes sans bloquer
                if (!follower.isBusy()) {

                    attrapeballe.setPower(-1);
                    pousseballe.setPosition(0.28); // tire les balles
                    if (System.currentTimeMillis() - startTime >= 6000) {
                        attrapeballe.setPower(0);
                        pousseballe.setPosition(0.5);
                        setPathState(11); // apres 4s au total passe a la prochaine etape
                    }
                }
                break;

            case 11:
                // Attendre 2 secondes sans bloquer
                if (!follower.isBusy()) {
                    follower.followPath(fin, true);
                    setPathState(12); // sort de la zone de tire
                }
                break;
            case 12:
                roueLanceur1.setVelocity(-1540);
                roueLanceur.setVelocity(1540);
                setPathState(13);
                break; // fin
        }
    }

    /*
     * a l'arret du code, enregistre la derniere position pour le teleop
     */
    @Override
    public void stop() {

        SharedPose.finalPose = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading());
    }

    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }
}
