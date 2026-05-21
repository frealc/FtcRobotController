package org.firstinspires.ftc.teamcode;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "servo")
public class DistanceSensorDebbugger extends LinearOpMode {

    private DcMotorEx lanceur;
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double targetPosition = 1500;

    PIDCoefficients coefficients = new PIDCoefficients(Kp,Ki,Kd);
    BasicPID controller = new BasicPID(coefficients);

    private Servo pousseballe;

    @Override
    public void runOpMode() {

        lanceur = hardwareMap.get(DcMotorEx.class, "LeftBack");

        waitForStart();

        while (opModeIsActive()) {
            double measuredPosition = lanceur.getCurrentPosition();

            double output = controller.calculate(targetPosition, measuredPosition);
        }
    }
}