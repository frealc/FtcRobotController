package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous(name = "TEST AUTO LANCER STABLE", group = "none")
public class testautomoteur extends OpMode {

    // Hardware
    private DcMotorEx roueLanceur, roueLanceur1;
    private CRServo attrapeballe, roue_a_balle;
    private Servo pousseballe;

    private CRServo chargement_manuel;
    private ShooterManager shooter;
    private Follower follower;

    // Paths
    private PathChain tire, gotopose1, takepose1;

    private Timer pathTimer;
    private int pathState = 0;
    private long startTime;

    // Poses
    private final Pose startPose = new Pose(58.91, 15.28, 3.17);
    private final Pose tirePose = new Pose(55.38, 12, 2.78);
    private final Pose rotatest = new Pose(-1.7, 13.73, -1.84);
    private final Pose correct = new Pose(17.37, 25.09, -1.19);
    private final Pose priseballe1 = new Pose(1.37, 42, -2.11);

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

    private void buildPaths() {
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

            // -------------------
            // CASE 0 : Démarrage shooter et déplacement vers la pose tire
            // -------------------
            case 0:
                if (!follower.isBusy()) {
                    shooter.startShooter(1615); // RPM cible

                    follower.followPath(tire, true);
                    startTime = System.currentTimeMillis();
                    setPathState(1);


                }
                break;

            // -------------------
            // CASE 1 : Attendre fin du path tire et continuer vers gotopose1
            // -------------------
            case 1:
                if (!follower.isBusy()) {
                    while (roueLanceur1.getVelocity()< 1600){

                    }
                    if (System.currentTimeMillis() - startTime >= 2000) {
                        chargement_manuel.setPower(0.4);
                        attrapeballe.setPower(-0.1);
                        roue_a_balle.setPower(-0.1);
                        pousseballe.setPosition(0.28);

                        startTime = System.currentTimeMillis();

                        // On passe dans l'état d'attente
                        setPathState(2);
                    }
                }
                break;

            // -------------------
            // CASE 2 : Aller chercher les balles
            // -------------------
            case 2:
                if (!follower.isBusy()) {
                    shooter.stopShooter();
                    follower.followPath(gotopose1, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Shooter prêt, tirer automatiquement ici si nécessaire
                shooter.update();

                break;
        }
    }

    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }
}
