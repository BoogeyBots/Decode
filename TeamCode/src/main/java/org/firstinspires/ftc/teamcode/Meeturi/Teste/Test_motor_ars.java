package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class Test_motor_ars extends LinearOpMode {
    DcMotorEx m;
    @Override
    public void runOpMode() throws InterruptedException {
        m = hardwareMap.get(DcMotorEx.class, "rightFront");

        waitForStart();

        while (opModeIsActive()) {
            m.setPower(1);
        }
    }
}
