package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Configurable
public class Test_ridicare extends LinearOpMode {
    CRServo sr, sl;
    public static boolean r = false;
    @Override
    public void runOpMode() throws InterruptedException {
        sr = hardwareMap.get(CRServo.class, "ridicare_right");
        sl = hardwareMap.get(CRServo.class, "ridicare_left");



        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_trigger > 0.01) {
                sr.setPower(1);
                sl.setPower(1);
            }
        }

    }
}
