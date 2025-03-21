package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;






@Autonomous(name="AutonomeGauche(panier)")
public class AutonomeGauche extends LinearOpMode {
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotor motorC;
    private DcMotor motorD;
    private DcMotorEx bras1;
    private DcMotorEx coudeA;
    private Servo boite;
    private Servo pince;

    private Servo pinceP;
    public void avancer(double vitesse, long temps) {
        motorA.setPower(vitesse);
        motorB.setPower(-vitesse);
        motorC.setPower(vitesse*1.06);
        motorD.setPower(-vitesse);
        sleep(temps);
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }
    public void reculer(double vitesse, long temps) {
        motorA.setPower(-vitesse);
        motorB.setPower(vitesse);
        motorC.setPower(-vitesse);
        motorD.setPower(vitesse);
        sleep(temps);
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }
    public void droite(double vitesse, long temps) {
        motorA.setPower(-vitesse);
        motorB.setPower(-vitesse);
        motorC.setPower(vitesse);
        motorD.setPower(vitesse);
        sleep(temps);
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }
    public void gauche(double vitesse, long temps) {
        motorA.setPower(vitesse);
        motorB.setPower(vitesse);
        motorC.setPower(-vitesse);
        motorD.setPower(-vitesse);
        sleep(temps);
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
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

    public void bras (double vitesse, long temps){
        bras1.setPower(vitesse);
        sleep(temps);
        bras1.setPower(0);
    }
    public void coude (double vitesse, long temps){
        coudeA.setPower(vitesse);
        sleep(temps);
        coudeA.setPower(0);
    }

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
    public void avancerT(double distance, double power) {
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

        // Put initialization blocks here
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);

        pince.setPosition(0);

        bras1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bras1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        long bas = bras1.getCurrentPosition();
        long haut = bas + 2000;
        waitForStart();
        telemetry.addData("z :", "Mode autonome initialisé");
        telemetry.update();

        bras_to_pos(haut,0.2,50);
        sleep(5000);
        bras_to_pos(bas,2,100);
        //avancer(0.5, 1050);

        /*avancer(0.5, 800);
        coude(-0.42, 1200);

        bras(-0.57, 1325);
        avancer(0.16, 1000);

        //boite.setPosition(1);
        bras(0.55, 300);
        pince.setPosition(1);
        bras(0.45, 225);
        reculer(0.5, 1000);

        gauche(0.6, 1500);*/




        while (opModeIsActive()) {
            //pas d'action car c'est une boucle
        }
    }
}