package org.firstinspires.ftc.teamcode;



import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.PanelsConfigurables;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
@TeleOp (name = "test teleop pedro ")
public class encodertest extends OpMode {

    private DcMotorEx roueLanceur;

    private DcMotorEx roueLanceur1;
    private CRServo attrapeballe;

    private Servo pousseballe;
    private CRServo roue_a_balle;
    private DcMotorEx LeftFront;
    private DcMotorEx LeftBack;
    private DcMotorEx RightFront;
    private DcMotorEx RightBack;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private final Pose startPose = new Pose(55.38, 8.35, 2.73);

    private final Pose test1 = new Pose(0, 0, 0);


    private double slowModeMultiplier = 0.5;
    double tgtpowerRota = 0;

    double varY = 0;
    double varX = 0;


    double varYpos = 0;
    double varXpos = 0;


    boolean PrecisionMode = false; //precision mis en faut quand initialisé

    double dpad_up = 0;

    Gamepad manette1;
    Gamepad manette2;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(SharedPose.finalPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(startPose, test1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), test1.getHeading())
                .build();

        roue_a_balle = hardwareMap.get(CRServo.class, "roue_a_balle");
        attrapeballe = hardwareMap.get(CRServo.class, "attrapeballe");
        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");
        pousseballe = hardwareMap.get(Servo.class, "pousseballe");
        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");

        pousseballe.setPosition(0.43);

        manette1 = this.gamepad1;
        manette2 = this.gamepad2;

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();


        if (manette2.a) {
            while (roueLanceur.getVelocity() < 1500) {
                roueLanceur.setPower(0.80);
                roueLanceur1.setPower(0.80);
            }
            attrapeballe.setPower(1);
            roue_a_balle.setPower(1);
            pousseballe.setPosition(0);
        }

        //Automated PathFollowing
        if (manette1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        } else {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (manette1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }


        if (!automatedDrive) {
            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;


            // Convertion pour Moteurs
            varYpos = varY;
            varXpos = varX;


            /// Mouvements
            double Power = varYpos;
            double strafe = varXpos;
            double Rotate = tgtpowerRota;


            if (manette1.left_trigger > 0 && manette1.right_trigger > 0) {

                tgtpowerRota = 0;

            } else if (manette1.left_trigger > 0) {

                tgtpowerRota = 1;

            } else if (manette1.right_trigger > 0) {

                tgtpowerRota = -1;

            } else {
                tgtpowerRota = 0;
            }


            if (PrecisionMode) {
                while (manette1.b) {
                    PrecisionMode = false;
                }
            } else {
                while (manette1.b) {
                    PrecisionMode = true;
                }
            }//active le mode precision quand B est appuyé et le desactive quand B est re appuyé


            if (PrecisionMode) {
                RightFront.setPower(-(Power + strafe - Rotate) / 3.5);
                LeftFront.setPower((Power - strafe + Rotate) / 3.5);
                RightBack.setPower((Power - strafe - Rotate) / 3.5);
                LeftBack.setPower(-(Power + strafe + Rotate) / 3.5);
                //en mode precision, reduit la vitesse par 3.5
            } else {
                RightFront.setPower(-(Power + strafe - Rotate));
                LeftFront.setPower((Power - strafe + Rotate));
                RightBack.setPower((Power - strafe - Rotate));
                LeftBack.setPower(-(Power + strafe + Rotate));

            }


            if (manette2.b) {
                roueLanceur.setPower(0.87);
                roueLanceur1.setPower(0.87);
            }
            /*else if (manette2.a){
                roueLanceur.setPower(0.65);
                roueLanceur1.setPower(0.65);

            }*/
            else {
                roueLanceur.setPower(0);
                roueLanceur1.setPower(0);
            }

            if (manette2.right_bumper) {
                pousseballe.setPosition(0.28);

            } else {
                pousseballe.setPosition(0.40);
            }

            if (manette2.x) {
                attrapeballe.setPower(1);
                roue_a_balle.setPower(1);
            } else if (manette2.y) {
                attrapeballe.setPower(-1);
                roue_a_balle.setPower(-1);
            } else {
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
            }


            telemetry.addData("vitesse moteur 1 du lanceur : ", roueLanceur.getVelocity());
            telemetry.addData("vitesse moteur 2 du lanceur : ", roueLanceur1.getVelocity());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();

        }

    }
}




