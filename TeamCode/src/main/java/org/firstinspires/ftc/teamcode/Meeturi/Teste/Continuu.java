package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp
public class Continuu extends LinearOpMode {
    CRServo s;
    @Override
    public void runOpMode() throws InterruptedException {
        s = hardwareMap.get(CRServo.class, "servo_left");

        waitForStart();

        while (opModeIsActive()) {
            s.setPower(1);
        }

    }
}
