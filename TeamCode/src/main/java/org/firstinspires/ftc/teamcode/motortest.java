package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class motortest extends LinearOpMode{
    private DcMotorEx LeftFront;
    private DcMotorEx LeftBack;
    private DcMotorEx RightFront;
    private DcMotorEx RightBack;


    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");

        Gamepad manette1 = this.gamepad1;
        Gamepad manette2 = this.gamepad2;

        waitForStart();




        while(opModeIsActive()) {

            if(manette1.x){
                LeftFront.setPower(1);
            } else if(manette1.y){
                RightFront.setPower(-1);
            } else if (manette1.a){
                LeftBack.setPower(-1);
            } else if (manette1.b){
                RightBack.setPower(1);
            } else {
                LeftFront.setPower(0);
                RightFront.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);
            }

        }


    }
}
