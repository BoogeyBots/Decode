package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@Configurable
@TeleOp
public class TestMGN extends LinearOpMode {
    CRServo left, right;
    public static double pl, pr;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(CRServo.class, "ridicare_left");
        right = hardwareMap.get(CRServo.class, "ridicare_right");

        waitForStart();

        while (opModeIsActive()) {
            left.setPower(pl);
            right.setPower(pr);
        }
    }
}
