package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.frozenmilk.sinister.loading.Pinned;

@TeleOp
public class SlothTest extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(0.1);
        }
    }
}
