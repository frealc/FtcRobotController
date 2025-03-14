package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutonomeGauche (Expérimental)")
public class AutonomeGaucheV2 extends LinearOpMode {
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;
    private DcMotorEx bras1;

    private DcMotorEx coudeA;
    private Servo boite;

    private Servo pince;

    private Servo pinceP;
    private Servo rotapinceP;
    private DistanceSensor distance0;
    private DistanceSensor distance1;
    private DistanceSensor distance2;
    double adherence = 0.2375;
    double brascheck = 0;


    public void resetMotors(){
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);

        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void avancer(double distance, double power) {
        double ROTATIONS = distance / adherence;
        double COUNTS = ROTATIONS * 515.46;
        int TargetA = (int) COUNTS + motorA.getCurrentPosition();
        int TargetB = (int) COUNTS - motorB.getCurrentPosition();
        motorA.setTargetPosition(TargetA);
        motorB.setTargetPosition(-TargetB);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        motorC.setPower(power);
        motorD.setPower(-power);
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("Target", motorA.getTargetPosition());
            telemetry.addData("Current", motorA.getCurrentPosition());
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }

    public void reculer(double distance, double power) {
        double ROTATIONS = distance / 0.2375;
        double COUNTS = ROTATIONS * 515.46;
        int TargetA = (int) COUNTS - motorA.getCurrentPosition();
        int TargetB = (int) COUNTS + motorB.getCurrentPosition();
        int TargetC = (int) COUNTS - motorC.getCurrentPosition();
        int TargetD = (int) COUNTS + motorD.getCurrentPosition();
        motorA.setTargetPosition(-TargetA);
        motorB.setTargetPosition(TargetB);
        motorC.setTargetPosition(-TargetC);
        motorD.setTargetPosition(TargetD);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        motorC.setPower(power);
        motorD.setPower(power);
        while (motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy()) {
            telemetry.addData("Target", motorA.getTargetPosition());
            telemetry.addData("Current", motorA.getCurrentPosition());
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }

    public void droite(double distance, double power) {
        double ROTATIONS = distance / 0.2375;
        double COUNTS = ROTATIONS * 515.46;
        int TargetA = (int) COUNTS - motorA.getCurrentPosition();
        int TargetB = (int) COUNTS - motorB.getCurrentPosition();
        int TargetC = (int) COUNTS + motorC.getCurrentPosition();
        int TargetD = (int) COUNTS + motorD.getCurrentPosition();
        motorA.setTargetPosition(-TargetA);
        motorB.setTargetPosition(-TargetB);
        motorC.setTargetPosition(TargetC);
        motorD.setTargetPosition(TargetD);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        motorC.setPower(power);
        motorD.setPower(power);
        while (motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy()) {
            telemetry.addData("Target", motorA.getTargetPosition());
            telemetry.addData("Current", motorA.getCurrentPosition());
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }

    public void gauche(double distance, double power) {
        double ROTATIONS = distance / 0.2375;
        double COUNTS = ROTATIONS * 515.46;
        int TargetA = (int) COUNTS + motorA.getCurrentPosition();
        int TargetB = (int) COUNTS + motorB.getCurrentPosition();
        int TargetC = (int) COUNTS - motorC.getCurrentPosition();
        int TargetD = (int) COUNTS - motorD.getCurrentPosition();
        motorA.setTargetPosition(TargetA);
        motorB.setTargetPosition(TargetB);
        motorC.setTargetPosition(-TargetC);
        motorD.setTargetPosition(-TargetD);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        motorC.setPower(power);
        motorD.setPower(power);
        while (motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy()) {
            telemetry.addData("Target", motorA.getTargetPosition());
            telemetry.addData("Current", motorA.getCurrentPosition());
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }

    public void rotaD(double vitesse, long temps) {
        motorA.setPower(vitesse);
        motorB.setPower(vitesse);
        motorC.setPower(vitesse);
        motorD.setPower(vitesse);
        sleep(temps);
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }

    public void rotaG(double vitesse, long temps) {
        motorA.setPower(-vitesse);
        motorB.setPower(-vitesse);
        motorC.setPower(-vitesse);
        motorD.setPower(-vitesse);
        sleep(temps);
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }

    public void bras(double vitesse, long temps) {
        bras1.getCurrentPosition();

        bras1.setPower(vitesse);

        sleep(temps);
        bras1.setPower(0);

    }

    public void high_basket() {

        // il reste à intégrer les déplacements pour se raprocher du high basket
        bras1.setPower(-1);
        sleep(1100);
        bras1.setPower(-0.4);
        avancer(0.1,0.1);
        boite.setPosition(0);
        sleep(800);
        boite.setPosition(1);
        sleep(400);
        reculer(0.3, 0.5);
        bras1.setPower(0.7);
        sleep(500);
    }

    public void coude(double vitesse, long temps) {
        coudeA.setPower(vitesse);
        sleep(temps);
        coudeA.setPower(0);
    }
    public void alignement(long seuil, double vitesse) {
        // Probleme avec capteur distance
        while (Math.abs(distance0.getDistance(DistanceUnit.CM)-distance1.getDistance(DistanceUnit.CM))>=seuil) {
            if (distance0.getDistance(DistanceUnit.CM)>=distance1.getDistance(DistanceUnit.CM)) {
                motorA.setPower(vitesse);
                motorB.setPower(vitesse);
                motorC.setPower(vitesse);
                motorD.setPower(vitesse);
            }
            else {
                motorA.setPower(-vitesse);
                motorB.setPower(-vitesse);
                motorC.setPower(-vitesse);
                motorD.setPower(-vitesse);
            }
        }
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }
    public void take_sample() {
        coude(-0.3, 650);
        pinceP.setPosition(1);
        sleep(500);
        coude(1, 1400);
        rotapinceP.setPosition(0);
        sleep(500);
        pinceP.setPosition(0.4);
        sleep(1000);
        coude(-1, 1205);
    }

    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorB = hardwareMap.get(DcMotor.class, "motorB");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        motorD = hardwareMap.get(DcMotor.class, "motorD");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");

        coudeA = hardwareMap.get(DcMotorEx.class, "coudeA");
        pince = hardwareMap.get(Servo.class, "pince");
        pinceP = hardwareMap.get(Servo.class, "pinceP");
        boite = hardwareMap.get(Servo.class, "boite");
        rotapinceP = hardwareMap.get(Servo.class, "rotapinceP");
        distance0 = hardwareMap.get(DistanceSensor.class, "distance0");
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance1");


        // Put initialization blocks here
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);

        pince.setPosition(0);

        telemetry.addData("z :", "Mode autonome initialisé");
        telemetry.update();

        waitForStart();

        droite(0.40, 0.3);
        coude(-1,1100);
        pinceP.setPosition(0.4);
        rotapinceP.setPosition(0.9);
        high_basket();



        sleep(100);
        //alignement(1, 0.5);
        rotaG(0.4,700);
        reculer(0.15,0.3);
        take_sample();

        while (opModeIsActive()) {

            //pas d'action car c'est une boucle
        }
    }
}