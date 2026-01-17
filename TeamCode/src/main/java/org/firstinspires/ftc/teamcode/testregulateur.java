package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * CE CODE UTILISE PEDRO PATHING. la methode est donc différente des autre mode auto
 */

@Disabled
@Configurable
@Autonomous(name = "regulateur ", group = "test")
public class testregulateur extends OpMode {

    //gestion du shooter (utilise le code "shooter manager")
    private ShooterManager shooter;
    private long startTime = 0;

    int shotCount = 0;

    double lastError = 0;
    double integral = 0;
    double maxI = 1.0; // limite de l'intégrale pour anti-windup
    ElapsedTime pidTimer = new ElapsedTime();
    boolean wasShooterReady = false;

    // PID constants (à ajuster)
    public static double nominalVoltage = 12.0;
    public static double kP = 0.0004;
    public static double kI = 0;
    public static double kD = 0.00005;
    public static double kF = 0.00048;

    // PID state

    static final double SHOOTER_HIGH = 1700;
    static final double SHOOTER_READY = 1640;
    static final double SHOOTER_LOW   = 1550;

    // Hardware
    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private CRServo attrapeballe;

    private Servo pousseballe;

    private CRServo chargement_manuel;
    private CRServo roue_a_balle;
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
    private final Pose tirePose = new Pose(55.38, 12, 2.78);

    private final Pose startPose = new Pose(58.91, 15.28, 3.17);


    private final Pose rotatest = new Pose(-1.7, 13.73, -1.84);
    private final Pose correct = new Pose(17.37, 25.09, -1.19);

    private final Pose priseballe1 = new Pose(1.37, 42, -2.11);
    private final Pose replace = new Pose(4, 30, -2.17);

    private final Pose replace2 = new Pose(19, 19,-2.17);

    private final Pose priseballe2 = new Pose(30.91, 42,-2.11);
    private final Pose poseFinal = new Pose(55.38, 30, 2.78);

    boolean shotLocked = false;

    /*
     *creation des nom pour les chemins du robot
     */
    private PathChain tire, gotopose1, takepose1, lance2, gotopose2, takepose2, lance3, replacepose, fin;


    @Override
    public void init() {
        // ---- HARDWARE ----
        roue_a_balle = hardwareMap.get(CRServo.class, "roue_a_balle");
        attrapeballe = hardwareMap.get(CRServo.class, "attrapeballe");
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        pousseballe = hardwareMap.get(Servo.class, "pousseballe");

        //dis a shooter manager les element utilisé
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

        roueLanceur.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roueLanceur1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        roueLanceur.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        roueLanceur1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


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
        shooter.update();
        follower.update();

        // Mise à jour des paths
        autonomousPathUpdate();

        // Debug
        telemetry.addData("State", pathState);
        telemetry.addData("Shooter RPM", roueLanceur.getVelocity());
        telemetry.update();

        Drawing.drawDebug(follower); //visualisation sur le panel pedro (192.168.43.1:8001 pour acceder au panel)

        telemetry.addData("V1", roueLanceur.getVelocity());
        telemetry.addData("V2", roueLanceur1.getVelocity());
        telemetry.update();

    }


    /*
     * Debut des chemin
     */
    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if(!follower.isBusy()) {
                    setLauncherVelocityPID(roueLanceur, roueLanceur1, 1600);
                    follower.followPath(tire, true);//vas a la position de tire
                    startTime = System.currentTimeMillis();//lance un timer
                    setPathState(1);//passe a la prochaine étape
                }
                break;

            case 1:

                if (follower.isBusy()) break;
                setLauncherVelocityPID(roueLanceur, roueLanceur1, 1615);

                double velocity = roueLanceur.getVelocity();
                telemetry.addData("Shooter velocity", velocity);
                telemetry.addData("Shots", shotCount);


                attrapeballe.setPower(-0.1);
                roue_a_balle.setPower(-0.1);
                pousseballe.setPosition(0.28);

                /*
                 * gestion de la plaque tournante avec vitesse moteur
                 */

                if (velocity >= SHOOTER_READY && !shotLocked) {
                    chargement_manuel.setPower(0.4);//si les moteur sont pareil que shooter ready, fait tourné la plaque
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
                    // quant toute les balles sont tiré ou 10s sont passé, arrete tout
                    chargement_manuel.setPower(0);
                    shotCount = 0;
                    shotLocked = false;
                    setPathState(2);//passe a la prochaine étape
                }

                break;


            case 2:
                setLauncherVelocityPID(roueLanceur, roueLanceur1, 1000);
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
                chargement_manuel.setPower(0);
                pousseballe.setPosition(0.41);
                follower.followPath(gotopose1, true);
                setPathState(3);
                break;


            case 3:

                if (!follower.isBusy()) {
                    setLauncherVelocityPID(roueLanceur, roueLanceur1, 1000);
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);//fait touné les elastique
                    follower.setMaxPower(0.6);

                    follower.followPath(takepose1, true); //rammasse les balle
                    setPathState(4);
                }
                break;
            case 4:

                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roue_a_balle.setPower(0);
                    follower.setMaxPower(1);
                    setLauncherVelocityPID(roueLanceur, roueLanceur1, 1550);

                    //follower.followPath(replacepose, true);
                    setPathState(5);
                }
                break;

            case 5:

                if (!follower.isBusy()) {
                    setLauncherVelocityPID(roueLanceur, roueLanceur1, 1550);
                    follower.followPath(lance2, true);//vas a la pos de tire
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(6);


                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    setLauncherVelocityPID(roueLanceur, roueLanceur1, 1530);
                    while (roueLanceur.getVelocity() < 1500) {
                    } // attente

                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0.28); //commence a tiré
                    setLauncherVelocityPID(roueLanceur, roueLanceur1, 1580);

                    if (System.currentTimeMillis() - startTime >= 5000) {
                        setLauncherVelocityPID(roueLanceur, roueLanceur1, 1000);
                        attrapeballe.setPower(0);
                        pousseballe.setPosition(0.41);
                        // Lancer le path suivant
                        //follower.followPath(gotopose2, true);
                        setPathState(7); //apres 5s passé au total, passe a la prochaine etape
                    }
                }
                break;
            case 7:

                if (!follower.isBusy()) {
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);

                    follower.setMaxPower(0.7);
                    follower.followPath(takepose2, true); // rammasse les balles
                    setPathState(8);
                }
                break;
            case 8:

                if (!follower.isBusy()) {
                    attrapeballe.setPower(0);
                    roue_a_balle.setPower(0);
                    setLauncherVelocityPID(roueLanceur, roueLanceur1, 1550);
                    follower.setMaxPower(1);

                    follower.followPath(lance3, true);//vas a la position de tire
                    startTime = 0;
                    startTime = System.currentTimeMillis();
                    setPathState(9);

                }
                break;
            case 9:
                // Attendre 2 secondes sans bloquer
                if (!follower.isBusy()) {
                    setLauncherVelocityPID(roueLanceur, roueLanceur1, 1550);
                    while (roueLanceur.getVelocity() < 1500) {
                    } // attente
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                    pousseballe.setPosition(0.28); // tire les balles
                    if (System.currentTimeMillis() - startTime >= 4000) {
                        attrapeballe.setPower(0);
                        roue_a_balle.setPower(0);
                        pousseballe.setPosition(0.41);
                        setPathState(10); // apres 4s au total passe a la prochaine etape
                    }
                }
                break;

            case 10:
                // Attendre 2 secondes sans bloquer
                if (!follower.isBusy()) {
                    follower.followPath(fin, true);
                    setPathState(11); // sort de la zone de tire
                }
                break;
            case 11:
                stopMotorPID(roueLanceur, roueLanceur1);
                setPathState(12);
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

    public void setLauncherVelocityPID(DcMotorEx m1, DcMotorEx m2, double targetVelocity) {

        // Mesure du temps écoulé depuis le dernier loop
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Mesure de la vitesse moyenne
        double velocity = (m1.getVelocity() + m2.getVelocity()) / 2.0;

        // Calcul de l'erreur
        double error = targetVelocity - velocity;

        // Intégrale avec anti-windup
        integral += error * dt;
        integral = Math.max(-maxI, Math.min(maxI, integral));

        // Dérivée
        double derivative = (error - lastError) / dt;

        // PID
        double pid = (kP * error) + (kI * integral) + (kD * derivative);

        // Feedforward avec compensation de tension batterie
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double feedforward = kF * targetVelocity * (nominalVoltage / batteryVoltage);

        // Calcul puissance finale
        double power = feedforward + pid;

        // Limite puissance entre 0 et 1
        power = Math.max(0, Math.min(1, power));

        // Appliquer aux moteurs
        m1.setPower(power);
        m2.setPower(power);

        lastError = error;
    }
    public void stopMotorPID(DcMotorEx m1, DcMotorEx m2) {
        integral = 0;
        lastError = 0;
        m1.setPower(0);
        m2.setPower(0);
    }
    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }
}
