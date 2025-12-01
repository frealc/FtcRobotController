package org.firstinspires.ftc.teamcode;


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

@Autonomous(name = "auto back bleu ", group = "bleu")
public class autoBackB extends OpMode {

    private long startTime = 0;
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private CRServo attrapeballe;

    private Servo pousseballe;
    private CRServo roue_a_balle;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(55.38, -8.35, -2.73); // Start Pose of our robot.

    private final Pose rotatest = new Pose(-1.7, -13.73, 1.84);
    private final Pose priseballe1 = new Pose(5.37, -47.89, 2.017); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose replace = new Pose(4, -30, 2);
    private final Pose pose2 = new Pose(22.21, -29.51,2.17);

    private final Pose priseballe2 = new Pose(30.91, -46.7,2.11);


    public final Pose finalPose = new Pose();
    private Path scorePreload;
    private PathChain gotopose1, takepose1, lance2, gotopose2, takepose2, lance3, replacepose;

    private int vitesse_lanceur = 0;



    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        gotopose1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, rotatest))
                .setLinearHeadingInterpolation(startPose.getHeading(), rotatest.getHeading())
                .build();

        takepose1 = follower.pathBuilder()
                .addPath(new BezierLine(rotatest, priseballe1))
                .setLinearHeadingInterpolation(rotatest.getHeading(), priseballe1.getHeading())
                .build();
        replacepose = follower.pathBuilder()
                .addPath(new BezierLine(priseballe1, replace))
                .setLinearHeadingInterpolation(priseballe1.getHeading(), replace.getHeading())
                .build();

        lance2 = follower.pathBuilder()
                .addPath(new BezierLine(replace, startPose))
                .setLinearHeadingInterpolation(replace.getHeading(), startPose.getHeading())
                .build();

        gotopose2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose2))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose2.getHeading())
                .build();

        takepose2 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, priseballe2))
                .setLinearHeadingInterpolation(pose2.getHeading(), priseballe2.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        lance3 =    follower.pathBuilder()
                .addPath(new BezierLine(priseballe2, startPose))
                .setLinearHeadingInterpolation(priseballe2.getHeading(), startPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

            case 0:
                // On attend que le robot ait fini le path précédent
                if (!follower.isBusy()) {

                    // Activer les roues du lanceur
                    roueLanceur.setPower(0.87);
                    roueLanceur1.setPower(0.87);
                    // Démarrer le timer
                    while (roueLanceur.getVelocity() < 1500) {
                    }

                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0);

                    startTime = System.currentTimeMillis();

                    // On passe dans l'état d'attente
                    setPathState(1);
                }
                break;


            case 1:
                // Attendre 2 secondes sans bloquer
                if (System.currentTimeMillis() - startTime >= 5000) {
                    roueLanceur.setPower(0);
                    roueLanceur1.setPower(0);
                    attrapeballe.setPower(0);
                    roue_a_balle.setPower(0);
                    pousseballe.setPosition(0.43);
                    // Lancer le path suivant
                    follower.followPath(gotopose1, true);
                    setPathState(2);
                }
                break;

            case 2:
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    follower.setMaxPower(0.4);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(takepose1, true);
                    setPathState(3);
                }
                break;
            case 3:
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roue_a_balle.setPower(0);
                    follower.setMaxPower(1);
                    roueLanceur.setPower(0.87);
                    roueLanceur1.setPower(0.87);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(replacepose, true);
                    setPathState(4);
                }
                break;

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(lance2, true);
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(5);


                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    while (roueLanceur.getVelocity() < 1500) {
                    }
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0);
                    if (System.currentTimeMillis() - startTime >= 7000) {
                        roueLanceur.setPower(0);
                        roueLanceur1.setPower(0);
                        attrapeballe.setPower(0);
                        pousseballe.setPosition(0.43);
                        // Lancer le path suivant
                        follower.followPath(gotopose2, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(0.4);
                    follower.followPath(takepose2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roue_a_balle.setPower(0);
                    roueLanceur.setPower(0.87);
                    roueLanceur1.setPower(0.87);
                    follower.setMaxPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(lance3, true);
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(8);

                }
                break;
            case 8:
                // Attendre 2 secondes sans bloquer
                if (!follower.isBusy()) {
                    while (roueLanceur.getVelocity() < 1500) {
                    }
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0);

                    setPathState(9);

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
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        final Pose finalPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
    }
}
