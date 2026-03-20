package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 *
 * CODE SANS UTILISATION DE DEAD WHEELS OU PEDRO PATHING
 *
 */
@Disabled
@Configurable
@TeleOp(name = "test pid teleop")
public class testporg extends LinearOpMode {

    private DcMotorEx LeftFront;
    private DcMotorEx LeftBack;
    private DcMotorEx RightFront;
    private DcMotorEx RightBack;
    private DcMotorEx roueLanceur;
    private DcMotorEx roueLanceur1;
    private Servo pousseballe;
    private DcMotorEx attrapeballe;
    private CRServo chargement_manuel;
    private DcMotorEx Motsoulever;

    // ── Coefficients PIDF du lanceur ──────────────────────────────────────────
    // kF : feedforward — base proportionnelle à la vitesse cible
    // Calibré pour des ticks/s élevés (f > 1000), ajuste kP/kI/kD si oscillations
    public static double kF = 0.00055;  // ≈ 1 / vitesse_max_en_tps  (ex: 1/1800 ≈ 0.00055)
    public static double kP = 0.00005;
    public static double kI = 0;
    public static double kD = 0.000005;

    // Anti-windup : limite l'accumulation de l'intégrale
    static final double MAX_INTEGRAL = 0.3;

    // ── Variables PIDF ────────────────────────────────────────────────────────
    double integralSum = 0;
    double lastError   = 0;
    ElapsedTime pidfTimer = new ElapsedTime();

    private Follower follower;
    VisionTest vision = new VisionTest();


    @Override
    public void runOpMode() {

        LeftFront  = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack   = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack  = hardwareMap.get(DcMotorEx.class, "RightBack");
        roueLanceur  = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        pousseballe  = hardwareMap.get(Servo.class, "pousseballe");
        attrapeballe = hardwareMap.get(DcMotorEx.class, "attrapeballe");
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");
        Motsoulever = hardwareMap.get(DcMotorEx.class, "Motsoulever");

        // ── Moteurs lanceur : RUN_USING_ENCODER par défaut (pour setVelocity) ─
        // On switche en RUN_WITHOUT_ENCODER uniquement pour le mode PIDF (right_bumper)
        roueLanceur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roueLanceur1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roueLanceur.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        roueLanceur1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double yaw     = 0;
        double range   = 0;
        double bearing = 0;
        double f       = 0;

        double tgtpowerRota = 0;
        double varY = 0;
        double varX = 0;

        boolean PrecisionMode = false;

        Gamepad manette1 = this.gamepad1;
        Gamepad manette2 = this.gamepad2;

        vision.init(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(SharedPose.finalPose);
        follower.update();

        waitForStart();
        pidfTimer.reset();

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = VisionTest.getTagBySpecificId(24);

            /* ************************************************
             * MANETTE 1 : PILOTE DEPLACEMENT
             ************************************************ */

            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;

            double Power  = varY;
            double strafe = varX;

            if (manette1.left_trigger > 0 && manette1.right_trigger > 0) {
                tgtpowerRota = 0;
            } else if (manette1.left_trigger > 0) {
                tgtpowerRota = 1;
            } else if (manette1.right_trigger > 0) {
                tgtpowerRota = -1;
            } else {
                tgtpowerRota = 0;
            }

            if (gamepad1.a) {
                PrecisionMode = !PrecisionMode;
                sleep(250);
            }

            double divisor = PrecisionMode ? 3.5 : 1.0;

            if (tag != null) {
                vision.update();
                yaw     = tag.ftcPose.yaw;
                range   = tag.ftcPose.range;
                bearing = tag.ftcPose.bearing;

                if (manette1.x) {
                    vision.update();
                    if      (bearing >= 2  && divisor == 1)   tgtpowerRota = -0.5 / 3.5;
                    else if (bearing <= -2 && divisor == 1)   tgtpowerRota =  0.5 / 3.5;
                    else if (bearing >= 2  && divisor == 3.5) tgtpowerRota = -0.5;
                    else if (bearing <= -2 && divisor == 3.5) tgtpowerRota =  0.5;
                    else                                       tgtpowerRota =  0;
                    vision.updateTelemetry();

                    // f(x) = 0.0023x² + 0.35x + 1121  (ticks/s selon distance en cm)
                    f = 0.0023 * Math.pow(range, 2) + 0.35 * range + 1121;
                }
            }

            RightFront.setPower(-(Power + strafe - tgtpowerRota) / (divisor));
            LeftFront.setPower( -(Power - strafe + tgtpowerRota) / (divisor));
            RightBack.setPower( -(Power - strafe - tgtpowerRota) / (divisor));
            LeftBack.setPower(  -(Power + strafe + tgtpowerRota) / (divisor + 0.2));

            if (manette1.dpad_down) {
                while (Motsoulever.getCurrentPosition() <= 100) {
                    Motsoulever.setPower(1);
                    telemetry.addData("pos moteur soulever : ", Motsoulever.getCurrentPosition());
                    telemetry.update();
                    if      (Motsoulever.getCurrentPosition() >= 105) Motsoulever.setPower(-0.1);
                    else if (Motsoulever.getCurrentPosition() <= 99)  Motsoulever.setPower(1);
                    else { Motsoulever.setPower(0); break; }
                    if (manette1.dpad_up) break;
                }
                Motsoulever.setPower(0);
            }
            if (manette1.dpad_up) {
                while (Motsoulever.getCurrentPosition() >= 0) {
                    Motsoulever.setPower(-0.5);
                    telemetry.addData("pos moteur soulever : ", Motsoulever.getCurrentPosition());
                    telemetry.update();
                }
                Motsoulever.setPower(0);
            }

            /* ************************************************
             * MANETTE 2 : PILOTE TIRE
             ************************************************ */

            if (manette2.right_trigger > 0) {
                // Vitesse fixe haute → setVelocity (RUN_USING_ENCODER)
                setLanceurMode(DcMotor.RunMode.RUN_USING_ENCODER);
                roueLanceur.setVelocity(1675);
                roueLanceur1.setVelocity(-1675);
                resetPIDF();

            } else if (manette2.left_bumper) {
                // Vitesse fixe basse → setVelocity (RUN_USING_ENCODER)
                setLanceurMode(DcMotor.RunMode.RUN_USING_ENCODER);
                roueLanceur.setVelocity(1360);
                roueLanceur1.setVelocity(-1360);
                resetPIDF();

            } else if (manette2.dpad_left) {
                // Déblocage balle : sens inverse → setVelocity (RUN_USING_ENCODER)
                setLanceurMode(DcMotor.RunMode.RUN_USING_ENCODER);
                roueLanceur.setVelocity(-1275);
                roueLanceur1.setVelocity(1275);
                resetPIDF();

            } else if (manette2.left_trigger > 0) {
                // Vitesse f depuis AprilTag → setVelocity (RUN_USING_ENCODER)
                // f est en ticks/s (valeur > 1000), compatible setVelocity directement
                setLanceurMode(DcMotor.RunMode.RUN_USING_ENCODER);
                roueLanceur.setVelocity(f);
                roueLanceur1.setVelocity(-f);
                resetPIDF();

            } else if (manette2.right_bumper) {
                // ── PIDF avec f comme vitesse cible ──────────────────────────
                // RUN_WITHOUT_ENCODER : on applique une puissance (0..1) calculée
                // par notre PIDF, qui utilise getVelocity() pour lire la vitesse réelle
                setLanceurMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double puissance = calculerPIDF(f);
                roueLanceur.setPower(puissance);
                roueLanceur1.setPower(-puissance);

            } else {
                setLanceurMode(DcMotor.RunMode.RUN_USING_ENCODER);
                roueLanceur.setPower(0);
                roueLanceur1.setPower(0);
                resetPIDF();
            }

            // ── Attrape-balle ─────────────────────────────────────────────────
            if      (manette2.x)         attrapeballe.setPower(-1);
            else if (manette2.dpad_down) attrapeballe.setPower(1);
            else if (manette2.a)         attrapeballe.setPower(1);
            else if (manette2.b)         attrapeballe.setPower(-1);
            else                         attrapeballe.setPower(0);

            // ── Pousse-balle ──────────────────────────────────────────────────
            if (manette2.b || manette2.y) pousseballe.setPosition(0.29);
            else                          pousseballe.setPosition(0.42);

            // ── Chargement manuel ─────────────────────────────────────────────
            chargement_manuel.setPower(-manette2.left_stick_x);

            /* ************************************************
             * TELEMETRY
             ************************************************ */
            double vitesseActuelle = getVitesseMoyenne();
            telemetry.addData("f cible (tps)",             "%.1f", f);
            telemetry.addData("vitesse lanceur actuelle",  "%.1f", vitesseActuelle);
            telemetry.addData("erreur PIDF (tps)",         "%.1f", f - vitesseActuelle);
            if (roueLanceur1.getVelocity() > f - 5 && roueLanceur1.getVelocity() < f + 5) {
                telemetry.addLine("LANCEUR PRET A TIRER!!!!!");
            }
            telemetry.addData("vitesse moteur 1 du lanceur : ", roueLanceur.getVelocity());
            telemetry.addData("vitesse moteur 2 du lanceur : ", roueLanceur1.getVelocity());
            telemetry.addData("vitesse roue avant droite",      RightFront.getVelocity());
            telemetry.addData("vitesse roue avant gauche",      LeftFront.getVelocity());
            telemetry.addData("vitesse roue arriere droite",    RightBack.getVelocity());
            telemetry.addData("vitesse roue arriere gauche",    LeftBack.getVelocity());
            telemetry.addData("vitesse chargement manuelle",    chargement_manuel.getPower());
            telemetry.update();
        }

        roueLanceur.setPower(0);
        roueLanceur1.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // MÉTHODES PIDF
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Calcule la puissance à appliquer via PIDF.
     *
     * @param targetTps  vitesse cible en ticks/seconde (= valeur de f)
     * @return           puissance entre 0.0 et 1.0
     *
     * GUIDE DE TUNING :
     *  1. kP=kI=kD=0 → monte kF jusqu'à atteindre ~80% de la vitesse cible.
     *  2. Monte kP jusqu'à ce que la vitesse atteigne la cible (légères oscillations OK).
     *  3. Ajoute kD pour amortir les oscillations.
     *  4. Ajoute kI (très petit) pour éliminer l'erreur résiduelle à l'état stable.
     */
    private double calculerPIDF(double targetTps) {
        double dt = pidfTimer.seconds();
        pidfTimer.reset();
        if (dt <= 0) dt = 0.001; // évite la division par zéro

        double vitesseActuelle = getVitesseMoyenne();
        double erreur = targetTps - vitesseActuelle;

        // Terme I avec anti-windup (évite que l'intégrale explose si le moteur est bloqué)
        integralSum += erreur * dt;
        integralSum = Math.max(-MAX_INTEGRAL / kI,
                Math.min( MAX_INTEGRAL / kI, integralSum));

        // Terme D
        double derivative = (erreur - lastError) / dt;
        lastError = erreur;

        double F = kF * targetTps;   // feedforward : la base de la puissance
        double P = kP * erreur;      // proportionnel
        double I = kI * integralSum; // intégral
        double D = kD * derivative;  // dérivé

        double puissance = F + P + I + D;

        // Clamp 0..1 (le lanceur ne tourne que dans un sens)
        return Math.max(0.0, Math.min(1.0, puissance));
    }

    /** Vitesse moyenne absolue des deux moteurs lanceur en ticks/seconde. */
    private double getVitesseMoyenne() {
        return (Math.abs(roueLanceur.getVelocity()) + Math.abs(roueLanceur1.getVelocity())) / 2.0;
    }

    /** Remet les variables PIDF à zéro (appelé quand on quitte le mode right_bumper). */
    private void resetPIDF() {
        integralSum = 0;
        lastError   = 0;
        pidfTimer.reset();
    }

    /**
     * Change le RunMode des deux moteurs lanceur seulement si nécessaire
     * (évite les appels inutiles à chaque itération de boucle = plus stable).
     */
    private void setLanceurMode(DcMotor.RunMode mode) {
        if (roueLanceur.getMode() != mode) {
            roueLanceur.setMode(mode);
            roueLanceur1.setMode(mode);
        }
    }
}