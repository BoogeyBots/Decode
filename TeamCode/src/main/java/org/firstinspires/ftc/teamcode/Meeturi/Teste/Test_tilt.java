package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Configurable
public class Test_tilt extends LinearOpMode {
    Servo sr, sl;
    public static double poz;
    @Override
    public void runOpMode() throws InterruptedException {
        sr = hardwareMap.get(Servo.class, "ridicare_right");
        sl = hardwareMap.get(Servo.class, "ridicare_left");


        waitForStart();

        while (opModeIsActive()) {
            sr.setPosition(1 - poz);
            sl.setPosition(poz);
        }

    }
}
