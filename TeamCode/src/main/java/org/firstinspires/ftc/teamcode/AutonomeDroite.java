package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;






@Autonomous(name="AutonomeDroite(clip)")
public class AutonomeDroite extends LinearOpMode {
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
    double adherence = 0.2375;


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
        bras2.setPower(vitesse);
        sleep(temps);
        bras1.setPower(0);
        bras2.setPower(0);
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

        /*avancer(0.5, 1050);

        avancer(0.5, 800);

        coude(-0.42, 1200);

        bras(-0.57, 1325);
        avancer(0.16, 1000);
        motorC.setPower(0.4);
        sleep(450);
        motorC.setPower(0);


        boite.setPosition(1);
        bras(0.55, 300);
        pince.setPosition(1);
        bras(0.45, 225);
        reculer(0.3, 450);
        droite(0.6, 1800);
        rotaG(0.5, 1400);

        //avancer(0.35,1550);

        avancer(0.35, 925);
        */

        rotaD(0.6);

        //rotaG(1);



        while (opModeIsActive()) {
            //pas d'action car c'est une boucle
        }

    }

}
