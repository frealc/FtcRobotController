package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomeGauche (Expérimental)")
public class AutonomeGaucheV2 extends LinearOpMode {
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
        while (motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy()) {
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
        // Put initialization blocks here
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
        pince.setPosition(0);

        waitForStart();
        telemetry.addData("z :", "Mode autonome initialisé");
        telemetry.update();

        gauche(0.3,0.1);
        droite(0.3,0.1);
        avancer(0.3,0.1);
        reculer(0.3,0.1);


        while (opModeIsActive()) {
            //pas d'action car c'est une boucle
        }
    }
}