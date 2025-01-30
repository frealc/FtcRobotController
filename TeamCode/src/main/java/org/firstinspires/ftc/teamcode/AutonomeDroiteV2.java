package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomeDroite (Expérimental)")
public class AutonomeDroiteV2 extends LinearOpMode {
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;
    private DcMotorEx bras1;
    private DcMotorEx bras2;
    private DcMotorEx coudeA;
    private Servo boite;

    private Servo pince;

    private Servo pinceP;
    private DistanceSensor distanceSensor;

    double adherence = 0.2375;

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
        while (motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy()) {
            telemetry.addData("Target", motorA.getTargetPosition());
            telemetry.addData("Current", motorA.getCurrentPosition());
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }

    public void reculer(double distance, double power) {
        double ROTATIONS = distance / adherence;
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
        double ROTATIONS = distance / adherence;
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
        double ROTATIONS = distance / adherence;
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

        double ROTATIONS = 0.99 / adherence; //rotation a 180 degree
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
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }

    void rotaG(double power) {

        double ROTATIONS = 0.99 / adherence; //rotation a 180 degree
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
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.update();
            if (!opModeIsActive()) {break;}
        }
        resetMotors();
    }

    public void bras(double vitesse, long temps) {
        bras1.setPower(vitesse);
        bras2.setPower(vitesse);
        sleep(temps);
        bras1.setPower(0);
        bras2.setPower(0);
    }

    public void coude(double vitesse, long temps) {
        coudeA.setPower(vitesse);
        sleep(temps);
        coudeA.setPower(0);
    }

    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorB = hardwareMap.get(DcMotor.class, "motorB");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        motorD = hardwareMap.get(DcMotor.class, "motorD");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coudeA = hardwareMap.get(DcMotorEx.class, "coudeA");
        pince = hardwareMap.get(Servo.class, "pince");
        pinceP = hardwareMap.get(Servo.class, "pinceP");
        boite = hardwareMap.get(Servo.class, "boite");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        // Put initialization blocks here
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
        pince.setPosition(0);

        waitForStart();
        telemetry.addData("z :", "Mode autonome initialisé");
        telemetry.update();

        avancer(0.6, 0.7);
        coude(-0.42, 1400);
        bras(-0.6, 1500);
        avancer(0.2, 0.6);
        boite.setPosition(1);
        bras(0.55, 300);
        pince.setPosition(1);

        /* Pose du specimen */
        bras(0.45, 225);

        //se met a l'endroit pour pousser les sample
        reculer(0.25, 1);
        droite(0.9,1);
        avancer(0.8,1);

        //pousse le premier sample
        droite(0.25,1);
        rotaD(1);
        avancer(1,1);
        reculer(1,1);

        //pousse le 2eme sample
        gauche(0.25,1);
        avancer(1,1);


        //pousse le 3eme sample
        /*reculer(1,1);
        gauche(0.25,0.6);
        avancer(1,0.6);*/

        while (opModeIsActive()) {
            //telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));
            //telemetry.update();
            //pas d'action car c'est une boucle
        }
    }
}