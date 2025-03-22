package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutonomeDroite (Principal)")
public class AutonomeDroite extends LinearOpMode {
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;
    private DcMotorEx bras1;
    private DcMotorEx coudeA;
    private DcMotorEx ascent;
    private Servo boite;

    private Servo pince;

    private Servo pinceP;
    private Servo rotapinceP;
    private DistanceSensor distance0;
    private DistanceSensor distance1;
    private DistanceSensor distance2;

    double adherence = 0.2375;
    long bas;
    long cliphaut;

    long clipbas;


    public void resetMotors(){
        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void bras(double vitesse, long temps) {
        bras1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bras1.setPower(vitesse);
        //ascent.setPower(vitesse);
        sleep(temps);
        bras1.setPower(0);
        //ascent.setPower(0);
    }

    public void coude(double vitesse, long temps) {
        coudeA.setPower(vitesse);
        sleep(temps);
        coudeA.setPower(0);
    }
    public void avancerBras(double distance, double power) {
        double ROTATIONS = distance / adherence;
        double COUNTS = ROTATIONS * 515.46;
        int TargetA = (int) COUNTS + motorA.getCurrentPosition();
        int TargetB = (int) COUNTS - motorB.getCurrentPosition();
        int TargetC = (int) COUNTS + motorC.getCurrentPosition();
        int TargetD = (int) COUNTS - motorD.getCurrentPosition();
        motorA.setTargetPosition(TargetA);
        motorB.setTargetPosition(-TargetB);
        motorC.setTargetPosition(TargetC);
        motorD.setTargetPosition(-TargetD);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        motorC.setPower(power);
        motorD.setPower(power);
        telemetry.addData("étape actuelle :", "Strafe vers l'avant et développement de la glissière");
        telemetry.update();
        coude(-1, 500);
        bras_to_pos(cliphaut,-0.5,50);
        while (motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy()) {
            telemetry.addData("Target", motorA.getTargetPosition());
            telemetry.addData("Current", motorA.getCurrentPosition());
            telemetry.addData("distancea droite du robot", distance0.getDistance(DistanceUnit.CM));
            telemetry.addData("distance a droite du robot", distance1.getDistance(DistanceUnit.CM));
            telemetry.addData("distance derrier le robot", distance2.getDistance(DistanceUnit.CM));
            telemetry.update();

            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }
    public void avancer(double distance, double power) {
        double ROTATIONS = distance / 0.2375;
        double COUNTS = ROTATIONS * 515.46;
        int TargetA = (int) COUNTS + motorA.getCurrentPosition();
        int TargetB = (int) COUNTS - motorB.getCurrentPosition();
        int TargetC = (int) COUNTS + motorC.getCurrentPosition();
        int TargetD = (int) COUNTS - motorD.getCurrentPosition();
        motorA.setTargetPosition(TargetA);
        motorB.setTargetPosition(-TargetB);
        motorC.setTargetPosition(TargetC);
        motorD.setTargetPosition(-TargetD);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        motorC.setPower(power);
        motorD.setPower(power);
        telemetry.addData("étape actuelle :", "Strafe vers l'avant");
        telemetry.update();
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("Target", motorA.getTargetPosition());
            telemetry.addData("Current", motorA.getCurrentPosition());
            telemetry.addData("distancea droite du robot", distance0.getDistance(DistanceUnit.CM));
            telemetry.addData("distance a droite du robot", distance1.getDistance(DistanceUnit.CM));
            telemetry.addData("distance derrier le robot", distance2.getDistance(DistanceUnit.CM));
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
    void rotaD(double power) {

        double ROTATIONS = 1 / adherence; //rotation a 180 degree
        double COUNTS = ROTATIONS * 515.46;
        int TargetA = (int)  COUNTS - motorA.getCurrentPosition();
        int TargetB = (int) COUNTS - motorB.getCurrentPosition();
        int TargetC = (int) COUNTS - motorC.getCurrentPosition();
        int TargetD = (int) COUNTS - motorD.getCurrentPosition();
        motorA.setTargetPosition(-TargetA);
        motorB.setTargetPosition(-TargetB);
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
        telemetry.addData("étape actuelle :", "Rotation vers la droite");
        telemetry.update();
        while (motorA.isBusy() || motorB.isBusy()|| motorC.isBusy()|| motorD.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.addData("distancea droite du robot", distance0.getDistance(DistanceUnit.CM));
            telemetry.addData("distance a droite du robot", distance1.getDistance(DistanceUnit.CM));
            telemetry.addData("distance derrier le robot", distance2.getDistance(DistanceUnit.CM));
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }

    void rotaG(double power) {

        double ROTATIONS = 1 / adherence; //rotation a 180 degree
        double COUNTS = ROTATIONS * 515.46;
        int TargetA = (int)  COUNTS + motorA.getCurrentPosition();
        int TargetB = (int) COUNTS + motorB.getCurrentPosition();
        int TargetC = (int) COUNTS + motorC.getCurrentPosition();
        int TargetD = (int) COUNTS + motorD.getCurrentPosition();
        motorA.setTargetPosition(TargetA);
        motorB.setTargetPosition(TargetB);
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
        telemetry.addData("étape actuelle :", "Rotation vers la gauche");
        telemetry.update();
        while (motorA.isBusy() || motorB.isBusy()|| motorC.isBusy()|| motorD.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.addData("distancea droite du robot", distance0.getDistance(DistanceUnit.CM));
            telemetry.addData("distance a droite du robot", distance1.getDistance(DistanceUnit.CM));
            telemetry.addData("distance derrier le robot", distance2.getDistance(DistanceUnit.CM));
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }
    public void rotaGP(double vitesse, long temps) {
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
    public void bras_to_pos(long position, double vitesse, long marge){
        telemetry.addData("test",bras1.getCurrentPosition());
        while (Math.abs(position-bras1.getCurrentPosition())>marge){
            if (bras1.getCurrentPosition()>position) {
                bras1.setPower(vitesse);
            }
            else {
                bras1.setPower(-vitesse);
            }
            bras1.getCurrentPosition();
            telemetry.addData("bras :", bras1.getCurrentPosition());
            telemetry.addData("target :", position);
            telemetry.update();
        }
        bras1.setPower(0);
        telemetry.update();
    }
    public void pose1() {
        coude(-1, 500);
        avancer(0.6, 0.6);
        bras_to_pos(cliphaut,0.5,50);
        boite.setPosition(1);
        avancer(0.14, 0.3);
        bras_to_pos(clipbas,0.6,50);
        pince.setPosition(1);
    }

    public void prise1() {
        reculer(0.25, 1);
        droite(1.3,1);
        rotaG(1);
    // A MODIFIER EN FONCTION DU DECALAGE
        rotaGP(0.5  ,200);
        //rotaG(0.1616);
        avancer(0.60,0.45);
        bras(0.3, 300);
        pince.setPosition(0);
        sleep(375);
    }

    public void pose2() {
        reculer(0.40,0.50);
        rotaD(1);
        gauche(1.4,1);
        bras_to_pos(cliphaut,-0.6,50);
        avancer(0.35, 0.4);
        bras_to_pos(clipbas,0.6,50);
        sleep(200);
        pince.setPosition(1);

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
        rotapinceP = hardwareMap.get(Servo.class, "rotapinceP");
        boite = hardwareMap.get(Servo.class, "boite");
        ascent = hardwareMap.get(DcMotorEx.class, "ascent");
        distance0 = hardwareMap.get(DistanceSensor.class, "distance0");
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance1");
        bras1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bras1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Put initialization blocks here
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
        pince.setPosition(0);
        bas = 40;
        cliphaut = 1848;
        clipbas = 1150;
        waitForStart();

        telemetry.addData("z :", "Mode autonome initialisé");
        telemetry.addData("distancea droite du robot", distance0.getDistance(DistanceUnit.CM));
        telemetry.addData("distance a droite du robot", distance1.getDistance(DistanceUnit.CM));
        telemetry.addData("distance derrier le robot", distance2.getDistance(DistanceUnit.CM));
        telemetry.update();


        //bras_to_pos(bas,0.2,50);
        //bras_to_pos(cliphaut,0.2,50);
        pose1();
       /* prise1();

        reculer(0.45,0.8);
        droite(1.45,1);
        rotaD(1);
        bras_to_pos(bas,0.4,50);
        avancer(0.49,0.40);


        pince.setPosition(0);
        sleep(500);
        pose2();
        reculer(0.5,0.8); */

        //se met a l'endroit pour pousser les sample
        reculer(0.25, 1);
        droite(0.9,1);
        avancer(0.7,1);

        //pousse le premier sample
        droite(0.30,1);
        //rotaD(1);
        reculer(1,1);
        avancer(1,1);

        // pousse le 2eme sample
        droite(0.37,1);
        reculer(1.2,1);

        //prent 2eme clip
       /* avancer(0.30,1);
        rotaG(1);
        //rotaG(0.1616);
        avancer(0.55,0.45);
        bras(0.3, 1000);

        pince.setPosition(0);
        sleep(500);*/

        //pose 2eme clip
       /* reculer(0.40,0.50);
        rotaD(1);
        gauche(1.4,1);
        bras(0.7,1500);
        avancer(0.3, 0.7);
        bras(-0.7,300);
        pince.setPosition(1);*/

        //pousse le 3eme sample
       /* avancer(1,1);
        droite(0.20,0.6);
        reculer(1,1); */



        while (opModeIsActive()) {

            //pas d'action car c'est une boucle mais telemetry autorisé

        }
    }
}