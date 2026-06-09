package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class test_transfer extends LinearOpMode {

    DcMotorEx motor_intake, motor_transfer;

    @Override


    public void runOpMode() throws InterruptedException{
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");

        waitForStart();

        while(opModeIsActive()) {


            if (gamepad1.right_trigger > 0.1) {
                motor_intake.setPower(1);
                motor_transfer.setPower(-1);
            } else if (gamepad1.left_trigger > 0.1) {
                motor_intake.setPower(-1);
                motor_transfer.setPower(1);
            } else {
                motor_transfer.setPower(0);
                motor_intake.setPower(0);
            }
        }
    }


}
