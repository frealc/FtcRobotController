package org.firstinspires.ftc.teamcode;


import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous(name = "TEST AUTO proche ")
public class testautomoteur extends OpMode {

    private long startTime = 0;
    private ShooterManager shooter;

    int shotCount = 0;
    boolean wasShooterReady = false;
    static final double SHOOTER_READY = 1210;
    static final double SHOOTER_LOW   = 1170;
    boolean shotLocked = false;
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private CRServo attrapeballe;

    private Servo pousseballe;
    private CRServo roue_a_balle;
    private CRServo chargement_manuel;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(-63.77, 44.81, 2.21);// Start Pose of our robot.
    private final Pose tirePose = new Pose(-22.12, 13.29, 2.30);

    private final Pose correct1 = new Pose(-13.15, 23.25, -1.54);
    private final Pose priseballe1 = new Pose(-15.42, 45.22, -1.57);
    private final Pose faceOuverture = new Pose(-4.25,30.13, 3.13);
    private final Pose Ouverture = new Pose(-4.25, 56.58, -3.13);
    private final Pose replace = new Pose(-3.25, 52.58, 2.61);

    //private final Pose pose2 = new Pose(22.21, 29.51,-2.17);
    private final Pose replace2 = new Pose(11.19, 4.19,-1.57);

    private final Pose priseballe2 = new Pose(10.21, 47.26,-2.19);
    private final Pose poseFinal = new Pose(-9.50, 24.92, -2.19);



    private Path scorePreload;
    private PathChain tire, gotopose1, takepose1, lance2, ouvreBalle, gotopose2, takepose2, lance3, replacepose, fin;

    private int vitesse_lanceur = 0;



    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(tirePose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        tire = follower.pathBuilder()
                .addPath(new BezierLine(startPose, tirePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), tirePose.getHeading())
                .build();

        /*gotopose1 = follower.pathBuilder()
                .addPath(new BezierLine(tirePose, rotatest))
                .setLinearHeadingInterpolation(tirePose.getHeading(), rotatest.getHeading())
                .build();*/

        takepose1 = follower.pathBuilder()
                .addPath(new BezierCurve(tirePose, correct1, priseballe1))
                .setLinearHeadingInterpolation(correct1.getHeading(), priseballe1.getHeading())
                .build();

        /*replacepose = follower.pathBuilder()
                .addPath(new BezierLine(priseballe1, replace))
                .setLinearHeadingInterpolation(priseballe1.getHeading(), replace.getHeading())
                .build();*/

        lance2 = follower.pathBuilder()
                .addPath(new BezierLine(priseballe1, tirePose))
                .setLinearHeadingInterpolation(priseballe1.getHeading(), tirePose.getHeading())
                .build();

        takepose2 = follower.pathBuilder()
                .addPath(new BezierCurve(tirePose, replace2, priseballe2))
                .setLinearHeadingInterpolation(replace2.getHeading(), priseballe2.getHeading())
                .build();

        ouvreBalle = follower.pathBuilder()
                .addPath(new BezierCurve(priseballe2, faceOuverture, Ouverture))
                .setLinearHeadingInterpolation(faceOuverture.getHeading(), Ouverture.getHeading())
                .build();

        /*gotopose2 = follower.pathBuilder()
                .addPath(new BezierLine(tirePose, pose2))
                .setLinearHeadingInterpolation(tirePose.getHeading(), pose2.getHeading())
                .build();*/


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        lance3 = follower.pathBuilder()
                .addPath(new BezierLine(priseballe2, tirePose))
                .setLinearHeadingInterpolation(priseballe2.getHeading(), tirePose.getHeading())
                .build();
        fin =    follower.pathBuilder()
                .addPath(new BezierLine(tirePose, faceOuverture))
                .setLinearHeadingInterpolation(tirePose.getHeading(), faceOuverture.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if(!follower.isBusy()) {
                    shooter.startShooter(1210);
                    follower.followPath(tire, true);
                    startTime = System.currentTimeMillis();
                    setPathState(1);
                }
                break;

            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

            case 1:
                shooter.update();
                // On attend que le robot ait fini le path précédent
                shooter.update();
                if (follower.isBusy()) break;

                double velocity = roueLanceur.getVelocity();
                telemetry.addData("Shooter velocity", velocity);
                telemetry.addData("Shots", shotCount);


                attrapeballe.setPower(-0.1);
                roue_a_balle.setPower(-0.1);
                pousseballe.setPosition(0.28);


                if (velocity >= SHOOTER_READY && !shotLocked) {
                    chargement_manuel.setPower(0.35);
                } else {
                    chargement_manuel.setPower(0);
                }



                if (velocity <= SHOOTER_LOW && !shotLocked) {
                    shotCount++;
                    shotLocked = true;
                }


                if (velocity >= SHOOTER_READY) {
                    shotLocked = false;
                }


                if (shotCount >= 4) {
                    chargement_manuel.setPower(0);
                    shotCount = 0;
                    shotLocked = false;
                    setPathState(2);
                }

                break;


            case 2:
                // Attendre 2 secondes sans bloquer
                shooter.stopShooter();
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
                pousseballe.setPosition(0.40);
                // Lancer le path suivant
                //follower.followPath(gotopose1, true);
                setPathState(3);

                break;

            case 3:
                shooter.update();
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    follower.setMaxPower(0.6);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(takepose1, true);
                    setPathState(4);
                }
                break;
            case 4:
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roue_a_balle.setPower(0);
                    follower.setMaxPower(1);
                    shooter.startShooter(1260);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //follower.followPath(replacepose, true);
                    setPathState(5);
                }
                break;

            case 5:
                shooter.update();
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(lance2, true);
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    while (roueLanceur.getVelocity() < 1200) {
                    }
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0.28);
                    if (System.currentTimeMillis() - startTime >= 5000) {
                        shooter.stopShooter();
                        roueLanceur.setPower(0);
                        roueLanceur1.setPower(0);
                        attrapeballe.setPower(0);
                        pousseballe.setPosition(0.40);
                        // Lancer le path suivant
                        //follower.followPath(gotopose2, true);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                shooter.update();
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(0.8);
                    follower.followPath(takepose2, true);
                    setPathState(8);
                }
                break;


            case 8:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    //follower.followPath(ouvreBalle, true);
                    setPathState(9);
                }
                break;


            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roue_a_balle.setPower(0);
                    shooter.startShooter(1260);
                    follower.setMaxPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(lance3, true);
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(10);

                }
                break;
            case 10:
                // Attendre 2 secondes sans bloquer
                if (!follower.isBusy()) {
                    while (roueLanceur.getVelocity() < 1200) {
                    }
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0.28);
                    setPathState(11);

                }
                break;
            case 11:
                if(!follower.isBusy()){
                    if (System.currentTimeMillis() - startTime >= 5000) {
                        follower.followPath(fin, true);
                        shooter.stopShooter();
                        attrapeballe.setPower(0);
                        roue_a_balle.setPower(0);
                        pousseballe.setPosition(0.40);
                        setPathState(12);
                    }
                }
                break;


        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        shooter.update();
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        vitesse_lanceur = (int) roueLanceur.getVelocity();
        telemetry.addData("vitesse moteur 2 du lanceur : ", vitesse_lanceur);
        telemetry.update();

    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        roue_a_balle = hardwareMap.get(CRServo.class, "roue_a_balle");
        attrapeballe = hardwareMap.get(CRServo.class, "attrapeballe");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        pousseballe = hardwareMap.get(Servo.class, "pousseballe");
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");

        shooter = new ShooterManager(
                roueLanceur,
                roueLanceur1,
                pousseballe,
                attrapeballe,
                roue_a_balle
        );

        pousseballe.setPosition(0.40);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        pathState = 0;
        startTime = System.currentTimeMillis();
        pathTimer.resetTimer();
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        final Pose finalPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        SharedPose.finalPose = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading());
    }
}

