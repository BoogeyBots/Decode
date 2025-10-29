package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class EncoderTurret extends LinearOpMode {
    DcMotorEx encoder;
    @Override
    public void runOpMode() throws InterruptedException {
        encoder = hardwareMap.get(DcMotorEx.class, "rightRear");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Ticks", encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
