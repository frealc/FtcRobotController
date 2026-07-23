package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "FonctionLanceur")
public class AjusteVitesseDistance extends LinearOpMode {

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

    private DigitalChannel ledR;
    private DigitalChannel ledG;

    private Follower follower;
    VisionTest vision = new VisionTest();

    // Limites de sécurité pour la vitesse consigne du lanceur
    private static final double SHOOTER_VELOCITY_MIN = 500;
    private static final double SHOOTER_VELOCITY_MAX = 2500;

    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");

        pousseballe = hardwareMap.get(Servo.class, "pousseballe");
        attrapeballe = hardwareMap.get(DcMotorEx.class, "attrapeballe");
        chargement_manuel = hardwareMap.get(CRServo.class, "chargement_manuel");
        Motsoulever = hardwareMap.get(DcMotorEx.class, "Motsoulever");

        ledR = hardwareMap.get(DigitalChannel.class, "ledR");
        ledG = hardwareMap.get(DigitalChannel.class, "ledG");

        double yaw = 0;
        double range = 0;
        double bearing = 0;
        double f = 0;

        // true tant qu'aucun tag n'a jamais été vu depuis le démarrage,
        // sert uniquement à afficher "N/A" en télémétrie tant que rien n'est détecté
        boolean tagDejaVu = false;

        double tgtpowerRota = 0;
        double varY = 0;
        double varX = 0;

        double varYpos = 0;
        double varXpos = 0;

        vision.init(hardwareMap, telemetry);

        boolean PrecisionMode = false;

        Gamepad manette1 = this.gamepad1;
        Gamepad manette2 = this.gamepad2;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(SharedPose.finalPose);
        follower.update();

        // ===== Vitesse lanceur réglable (manette 2) =====
        double shooterVelocity = 1675;
        double step = 0.01; // ±1% par appui

        boolean upPressed = false;
        boolean downPressed = false;

        waitForStart();

        ledR.setMode(DigitalChannel.Mode.OUTPUT);
        ledG.setMode(DigitalChannel.Mode.OUTPUT);

        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = VisionTest.getTagBySpecificId(24);

            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;

            double Power = varY;
            double strafe = varX;

            if (manette1.left_trigger > 0 && manette1.right_trigger > 0) {
                tgtpowerRota = 0;
            }
            else if (manette1.left_trigger > 0) {
                tgtpowerRota = 1;
            }
            else if (manette1.right_trigger > 0) {
                tgtpowerRota = -1;
            }
            else {
                tgtpowerRota = 0;
            }

            if (gamepad1.a) {
                PrecisionMode = !PrecisionMode;
                sleep(250);
            }

            double divisor = PrecisionMode ? 3.5 : 1.0;

            if (tag != null) {

                yaw = tag.ftcPose.yaw;
                range = tag.ftcPose.range;
                bearing = tag.ftcPose.bearing;
                tagDejaVu = true;

                if(manette1.x) {

                    if (bearing >= 2 && divisor == 1) {
                        tgtpowerRota = -0.5/3.5;
                    }
                    else if (bearing <= -2 && divisor == 1) {
                        tgtpowerRota = 0.5/3.5;
                    }
                    else if (bearing >= 2 && divisor == 3.5){
                        tgtpowerRota = -0.5;
                    }
                    else if (bearing <= -2 && divisor == 3.5){
                        tgtpowerRota = 0.5;
                    }
                    else {
                        tgtpowerRota = 0;
                    }

                    vision.updateTelemetry();

                    f = 0.0023 * Math.pow(range, 2) + 0.35 * range + 1121;
                }
            }

            // ===== Réglage vitesse lanceur ±5% avec la manette 2 =====
            if (manette2.dpad_up && !upPressed) {
                shooterVelocity *= (1 + step);
                upPressed = true;
            }
            if (!manette2.dpad_up) upPressed = false;

            if (manette2.dpad_right && !downPressed) {
                shooterVelocity *= (1 - step);
                downPressed = true;
            }
            if (!manette2.dpad_right) downPressed = false;

            // on empêche la consigne de sortir d'une plage raisonnable
            if (shooterVelocity > SHOOTER_VELOCITY_MAX) shooterVelocity = SHOOTER_VELOCITY_MAX;
            if (shooterVelocity < SHOOTER_VELOCITY_MIN) shooterVelocity = SHOOTER_VELOCITY_MIN;

            // déplacement
            RightFront.setPower(-(Power + strafe - tgtpowerRota) / (divisor));
            LeftFront.setPower(-(Power - strafe + tgtpowerRota) / (divisor));
            RightBack.setPower(-(Power - strafe - tgtpowerRota) / (divisor));
            LeftBack.setPower(-(Power + strafe + tgtpowerRota) / (divisor+0.2));

            // ===== LANCEUR =====

            if (manette2.right_trigger > 0) {
                roueLanceur.setVelocity(shooterVelocity);
                roueLanceur1.setVelocity(-shooterVelocity);
            }

            else if (manette2.left_bumper) {
                roueLanceur.setVelocity(1360);
                roueLanceur1.setVelocity(-1360);
            }

            else if (manette2.dpad_left) {
                roueLanceur.setVelocity(-1275);
                roueLanceur1.setVelocity(1275);
            }

            else if (manette2.left_trigger > 0){
                roueLanceur1.setVelocity(-f);
                roueLanceur.setVelocity(f);
            }

            else {
                roueLanceur.setPower(0);
                roueLanceur1.setPower(0);
            }

            // LED vitesse correcte
            if (roueLanceur.getVelocity() > shooterVelocity - 40 &&
                    roueLanceur.getVelocity() < shooterVelocity + 40){
                ledR.setState(true);
                ledG.setState(false);
            } else {
                ledR.setState(false);
                ledG.setState(true);
            }

            // attrape balle
            if (manette2.x) {
                attrapeballe.setPower(-1);
            } else if (manette2.dpad_down) {
                attrapeballe.setPower(1);
            } else if (manette2.a) {
                attrapeballe.setPower(1);
            } else if (manette2.b) {
                attrapeballe.setPower(-0.6);
            } else {
                attrapeballe.setPower(0);
            }

            if (manette2.b || manette2.y) {
                pousseballe.setPosition(0.29);
            } else {
                pousseballe.setPosition(0.42);
            }

            chargement_manuel.setPower(-manette2.left_stick_x);

            // ===== TELEMETRY =====

            telemetry.addLine("--- Lanceur ---");
            telemetry.addData("Consigne vitesse lanceur", "%.0f", shooterVelocity);
            telemetry.addData("Vitesse reelle lanceur 1", "%.0f", roueLanceur.getVelocity());
            telemetry.addData("Vitesse reelle lanceur 2", "%.0f", roueLanceur1.getVelocity());

            telemetry.addLine("--- Camera / QR code ---");
            if (tagDejaVu) {
                telemetry.addData("Distance QR code (range)", "%.1f cm", range);
                telemetry.addData("Bearing", "%.1f deg", bearing);
                telemetry.addData("Yaw", "%.1f deg", yaw);
            } else {
                telemetry.addData("Distance QR code (range)", "N/A - aucun tag détecté");
            }

            telemetry.addLine("--- Drivetrain ---");
            telemetry.addData("vitesse roue avant droite", RightFront.getVelocity());
            telemetry.addData("vitesse roue avant gauche", LeftFront.getVelocity());
            telemetry.addData("vitesse roue arriere droite", RightBack.getVelocity());
            telemetry.addData("vitesse roue arriere gauche", LeftBack.getVelocity());

            telemetry.addData("vitesse chargement manuel", chargement_manuel.getPower());

            telemetry.update();
        }
    }
}