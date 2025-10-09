package org.firstinspires.ftc.teamcode;

/*import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;*/
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "RR test")
public class testporg extends LinearOpMode {


    @Override
    public void runOpMode() {
        /*
        Pose2d beginPose = new Pose2d(10, -65, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);*/

        waitForStart();

        /*if (isStopRequested()) return;
        Actions.runBlocking(
            drive.actionBuilder(beginPose)
            //mettre action dedans (test sur meepmeep avant de faire ici)

                    .splineTo(new Vector2d(38, -30), Math.toRadians(90))
                    .waitSeconds(1.5)
                    .strafeTo(new Vector2d(38, -40))

                    .build()
        );*/
    }
}