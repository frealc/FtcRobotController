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

@Autonomous(name = "auto back rouge exemple", group = "Examples")
public class autoTestBackR extends OpMode {

    private long startTime = 0;
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private CRServo attrapeballe;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(54.19, -19.10, 2.5); // Start Pose of our robot.

    private final Pose rotatest = new Pose(-3.87, -30.7, 2.216); // Start Pose of our robot.
    private final Pose scorePose = new Pose(0, 0, Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(0, 0,0); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose testcaré = new Pose(0, 0, Math.toRadians(270));
    private Path scorePreload;
    private PathChain Test1, test2, test3, test4;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Test1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, rotatest))
                .setLinearHeadingInterpolation(startPose.getHeading(), rotatest.getHeading())
                .build();

        test2 = follower.pathBuilder()
                .addPath(new BezierLine(rotatest, pickup1Pose))
                .setLinearHeadingInterpolation(rotatest.getHeading(), pickup1Pose.getHeading())
                .build();

        test3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, startPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), startPose.getHeading())
                .build();

        test4 = follower.pathBuilder()
                .addPath(new BezierLine(testcaré, startPose))
                .setLinearHeadingInterpolation(testcaré.getHeading(), startPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

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
                    attrapeballe.setPower(-1);
                    // Démarrer le timer
                    startTime = System.currentTimeMillis();

                    // On passe dans l'état d'attente
                    setPathState(100);
                }
                break;


            case 100:
                // Attendre 2 secondes sans bloquer
                if (System.currentTimeMillis() - startTime >= 5000) {
                    roueLanceur.setPower(0);
                    roueLanceur1.setPower(0);
                    attrapeballe.setPower(0);
                    // Lancer le path suivant
                    follower.followPath(Test1, true);
                    setPathState(1);
                }
                break;

            case 1:
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(test2, true);
                    setPathState(2);
                }
                break;

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(test3, true);
                    setPathState(3);

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
        attrapeballe = hardwareMap.get(CRServo.class, "attrapeballe");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
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
    }
}