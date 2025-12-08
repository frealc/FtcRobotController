package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Supplier;

@TeleOp
public class encodertest extends LinearOpMode {


    private DcMotorEx LeftFront, LeftBack, RightFront, RightBack;
    private DcMotorEx roueLanceur, roueLanceur1;
    private Servo pousseballe;
    private CRServo attrapeballe, roue_a_balle;


    private Follower follower;
    private boolean automatedDrive = false;
    private Supplier<PathChain> poseFrontR;
    private Supplier<PathChain> poseFrontB;
    private Supplier<PathChain> poseBackR;
    private Supplier<PathChain> poseBackB;

    @Override
    public void runOpMode() {


        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");

        roueLanceur = hardwareMap.get(DcMotorEx.class, "rouelanceur");
        roueLanceur1 = hardwareMap.get(DcMotorEx.class, "rouelanceur1");

        pousseballe = hardwareMap.get(Servo.class, "pousseballe");
        attrapeballe = hardwareMap.get(CRServo.class, "attrapeballe");
        roue_a_balle = hardwareMap.get(CRServo.class, "roue_a_balle");


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(SharedPose.finalPose);   // position de fin de votre auto
        follower.update();


        final Pose tirePoseFR = new Pose(-11.84, 6.16, 2.40);
        final Pose tirePoseFB = new Pose(-11.84, -6.16, 2.40);
        final Pose tirePoseBR = new Pose(55.38, 8.35, 2.73);
        final Pose tirePoseBB = new Pose(55.38, -8.35, -2.73);

        poseFrontR = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(SharedPose.finalPose, tirePoseFR)))
                .setLinearHeadingInterpolation(SharedPose.finalPose.getHeading(), tirePoseFR.getHeading())
                .build();
        poseFrontB = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(SharedPose.finalPose, tirePoseFB)))
                .setLinearHeadingInterpolation(SharedPose.finalPose.getHeading(), tirePoseFB.getHeading())
                .build();
        poseBackR = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(SharedPose.finalPose, tirePoseBR)))
                .setLinearHeadingInterpolation(SharedPose.finalPose.getHeading(), tirePoseBR.getHeading())
                .build();
        poseBackB = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(SharedPose.finalPose, tirePoseBB)))
                .setLinearHeadingInterpolation(SharedPose.finalPose.getHeading(), tirePoseBB.getHeading())
                .build();

        boolean PrecisionMode = false;
        double tgtpowerRota = 0;

        waitForStart();
        follower.startTeleopDrive(); //a peu ettre changer

        while (opModeIsActive()) {

            follower.update();

            // /////////////////////////////////////////////////////
            //              CONTROLE AUTOMATIQUE : DPAD UP / DOWN
            // /////////////////////////////////////////////////////



            if (!automatedDrive && gamepad1.y) {
                telemetry.addLine("choix de path tire avant : ");
                telemetry.addLine("X = pose de tire Bleu ");
                telemetry.addLine("B = pose de tire Rouge ");
                telemetry.update();
                if (!automatedDrive && gamepad1.b){
                    follower.followPath(poseFrontR.get());
                    automatedDrive = true;
                }
                else if (!automatedDrive && gamepad1.x){
                    follower.followPath(poseFrontB.get());
                    automatedDrive = true;
                }
            }

            else if (!automatedDrive && gamepad1.a) {
                telemetry.addLine("choix de path tire arriere : ");
                telemetry.addLine("X = pose de tire Bleu ");
                telemetry.addLine("B = pose de tire Rouge ");
                telemetry.update();
                if (!automatedDrive && gamepad1.b){
                    follower.followPath(poseBackR.get());
                    automatedDrive = true;
                }
                else if (!automatedDrive && gamepad1.x){
                    follower.followPath(poseBackB.get());
                    automatedDrive = true;
                }
            }


            if (automatedDrive) {
                if (!follower.isBusy() || gamepad1.dpad_down) {
                    follower.startTeleopDrive();
                    automatedDrive = false;
                }
            }


            if (automatedDrive) {
                telemetry.addLine("FOLLOW PATH EN COURS");
                telemetry.addData("vas vers : ", follower.getCurrentPath());
                telemetry.addData("position actuelle : ", follower.getPose());
                telemetry.update();
                continue;
            }


            double varY = gamepad1.left_stick_y;
            double varX = gamepad1.left_stick_x;

            double Power = varY;
            double strafe = varX;

            // Rotation avec triggers
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                tgtpowerRota = 0;
            } else if (gamepad1.left_trigger > 0) {
                tgtpowerRota = 1;
            } else if (gamepad1.right_trigger > 0) {
                tgtpowerRota = -1;
            } else {
                tgtpowerRota = 0;
            }


            if (gamepad1.right_bumper) {
                PrecisionMode = !PrecisionMode;
                sleep(250);
            }

            double divisor = PrecisionMode ? 3.5 : 1.0;

            RightFront.setPower(-(Power + strafe - tgtpowerRota) / divisor);
            LeftFront.setPower(-(Power - strafe + tgtpowerRota) / divisor);
            RightBack.setPower(-(Power - strafe - tgtpowerRota) / divisor);
            LeftBack.setPower(-(Power + strafe + tgtpowerRota) / divisor);

            // /////////////////////////////////////////////////////
            //                  CONTROLES MANETTE 2
            // /////////////////////////////////////////////////////

            if (gamepad2.b) {
                roueLanceur.setVelocity(1675);
                roueLanceur1.setVelocity(1675);
                if (roueLanceur.getVelocity() > 1500) {
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                }

            } else if (gamepad2.a) {
                roueLanceur.setVelocity(1440);
                roueLanceur1.setVelocity(1440);
                if (roueLanceur.getVelocity() > 1400) {
                    attrapeballe.setPower(1);
                    roue_a_balle.setPower(1);
                }

            } else if (gamepad2.x) {
                attrapeballe.setPower(1);
                roue_a_balle.setPower(1);

            } else if (gamepad2.y) {
                attrapeballe.setPower(-1);
                roue_a_balle.setPower(-1);

            } else {
                roueLanceur.setPower(0);
                roueLanceur1.setPower(0);
                attrapeballe.setPower(0);
                roue_a_balle.setPower(0);
            }

            // Servo pousse balle
            pousseballe.setPosition(gamepad2.right_bumper ? 0.28 : 0.40);

            // /////////////////////////////////////////////////////
            // TELEMETRY
            // /////////////////////////////////////////////////////

            telemetry.addData("AutoDrive", automatedDrive);
            telemetry.addData("Follower Pose", follower.getPose());
            telemetry.addData("Lanceur1", roueLanceur.getVelocity());
            telemetry.addData("Lanceur2", roueLanceur1.getVelocity());
            telemetry.update();
        }
    }
}
