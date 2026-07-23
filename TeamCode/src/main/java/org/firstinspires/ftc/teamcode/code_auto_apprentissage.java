package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "auto app", group = "rouge")
public class code_auto_apprentissage extends OpMode {
    private Servo pousseballe;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private long startTime = 0;

    private int pathState;

    private final Pose startPose = new Pose(-63.77, 44.81, 3.14);// Start Pose of our robot.
    private final Pose tirePose = new Pose(-70, 40, 0); // Shoot pose of our robot
    private Path scorePreload;
    private PathChain test,b;

    public void buildPaths() {
        test =    follower.pathBuilder()
                .addPath(new BezierLine(startPose, tirePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), tirePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                if(!follower.isBusy()) {
                    pousseballe.setPosition(0.5);
                    follower.followPath(test, true);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {

                    pousseballe.setPosition(0.28);
                }
                break;

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


        pousseballe = hardwareMap.get(Servo.class, "pousseballe");
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }

    @Override
    public void start() {
        pathState = 0;
        startTime = System.currentTimeMillis();
        pathTimer.resetTimer();
    }
}
