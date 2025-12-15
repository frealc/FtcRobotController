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

@Autonomous(name = "auto back bleu ", group = "bleu")
public class autoBackB extends OpMode {

    // Hardware
    private ShooterManager shooter;
    private long startTime = 0;

    int shotCount = 0;
    boolean wasShooterReady = false;
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private CRServo attrapeballe;

    private Servo pousseballe;

    private CRServo chargement_manuel;
    private CRServo roue_a_balle;
    boolean timerStarted = false;
    private Follower follower;

    static final double SHOOTER_READY = 1620;
    static final double SHOOTER_LOW   = 1550;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose tirePose = new Pose(55.38, -12, -2.78);

    private final Pose startPose = new Pose(58.91, -15.28, -3.17);
    // Start Pose of our robot.

    private final Pose rotatest = new Pose(-1.7, -13.73, 1.84);
    private final Pose correct = new Pose(17.37, -25.09, 1.19);

    private final Pose priseballe1 = new Pose(1.37, -42, 2.11); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose replace = new Pose(4, -30, 2.17);

    //private final Pose pose2 = new Pose(22.21, 29.51,-2.17);
    private final Pose replace2 = new Pose(19, -19,2.17);

    private final Pose priseballe2 = new Pose(30.91, -42,2.11);
    private final Pose poseFinal = new Pose(55.38, -30, -2.78);


    public final Pose finalPose = new Pose(0, 0, 0);
    private Path scorePreload;
    private PathChain tire, gotopose1, takepose1, lance2, gotopose2, takepose2, lance3, replacepose, fin;

    private int vitesse_lanceur = 0;
    boolean shotLocked = false;

    @Override
    public void init() {
        // ---- HARDWARE ----
        roue_a_balle = hardwareMap.get(CRServo.class, "roue_a_balle");
        attrapeballe = hardwareMap.get(CRServo.class, "attrapeballe");
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        pousseballe = hardwareMap.get(Servo.class, "pousseballe");

        // ---- SHOOTER MANAGER ----
        shooter = new ShooterManager(
                roueLanceur,
                roueLanceur1,
                pousseballe,
                attrapeballe,
                roue_a_balle
        );

        // ---- FOLLOWER / PATHS ----
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        pathTimer = new Timer();
    }

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
                .addPath(new BezierCurve(rotatest, correct, priseballe1))
                .setLinearHeadingInterpolation(correct.getHeading(), priseballe1.getHeading())
                .build();
        /*replacepose = follower.pathBuilder()
                .addPath(new BezierLine(priseballe1, replace))
                .setLinearHeadingInterpolation(priseballe1.getHeading(), replace.getHeading())
                .build();*/

        lance2 = follower.pathBuilder()
                .addPath(new BezierCurve(priseballe1, replace, tirePose))
                .setLinearHeadingInterpolation(priseballe1.getHeading(), tirePose.getHeading())
                .build();

        /*gotopose2 = follower.pathBuilder()
                .addPath(new BezierLine(tirePose, pose2))
                .setLinearHeadingInterpolation(tirePose.getHeading(), pose2.getHeading())
                .build();*/

        takepose2 = follower.pathBuilder()
                .addPath(new BezierCurve(tirePose, replace2, priseballe2))
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


    @Override
    public void start() {
        pathState = 0;
        startTime = System.currentTimeMillis();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // Update shooter et follower
        shooter.update();
        follower.update();

        // Mise à jour des paths
        autonomousPathUpdate();

        // Debug
        telemetry.addData("State", pathState);
        telemetry.addData("Shooter RPM", roueLanceur.getVelocity());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if(!follower.isBusy()) {
                    shooter.startShooter(1615); // RPM cible
                    follower.followPath(tire, true);
                    startTime = System.currentTimeMillis();
                    setPathState(1);
                }
                break;

            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

            case 1:
                shooter.update();
                if (follower.isBusy()) break;

                double velocity = roueLanceur.getVelocity();
                telemetry.addData("Shooter velocity", velocity);
                telemetry.addData("Shots", shotCount);


                attrapeballe.setPower(-0.1);
                roue_a_balle.setPower(-0.1);
                pousseballe.setPosition(0.28);


                if (velocity >= SHOOTER_READY && !shotLocked) {
                    chargement_manuel.setPower(0.4);
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


                shooter.stopShooter();
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
                chargement_manuel.setPower(0);
                pousseballe.setPosition(0.41);
                // Lancer le path suivant
                follower.followPath(gotopose1, true);
                setPathState(3);
                break;

            case 3:
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
                    roueLanceur.setVelocity(1610);
                    roueLanceur1.setVelocity(1610);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //follower.followPath(replacepose, true);
                    setPathState(5);
                }
                break;

            case 5:
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
                    while (roueLanceur.getVelocity() < 1500) {
                    }
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0.28);
                    if (System.currentTimeMillis() - startTime >= 5000) {
                        roueLanceur.setVelocity(900);
                        roueLanceur1.setVelocity(900);
                        attrapeballe.setPower(0);
                        pousseballe.setPosition(0.41);
                        // Lancer le path suivant
                        //follower.followPath(gotopose2, true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(0.7);
                    follower.followPath(takepose2, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roue_a_balle.setPower(0);
                    roueLanceur.setVelocity(1610);
                    roueLanceur1.setVelocity(1610);
                    follower.setMaxPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(lance3, true);
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(9);

                }
                break;
            case 9:
                // Attendre 2 secondes sans bloquer
                if (!follower.isBusy()) {
                    while (roueLanceur.getVelocity() < 1500) {
                    }
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0.28);
                    if (System.currentTimeMillis() - startTime >= 4000) {
                        attrapeballe.setPower(0);
                        roue_a_balle.setPower(0);
                        pousseballe.setPosition(0.41);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                // Attendre 2 secondes sans bloquer
                if (!follower.isBusy()) {
                    follower.followPath(fin, true);
                    setPathState(11);

                }
                break;
        }
    }
    @Override
    public void stop() {
        final Pose finalPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
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
