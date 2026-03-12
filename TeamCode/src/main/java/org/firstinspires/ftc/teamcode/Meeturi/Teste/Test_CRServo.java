package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp
public class Test_CRServo extends LinearOpMode {
    CRServo sl;
    @Override
    public void runOpMode() throws InterruptedException {
        sl = hardwareMap.get(CRServo.class, "ridicare_left");

        waitForStart();

        while (opModeIsActive()) {
            sl.setPower(1);
        }
    }
}
