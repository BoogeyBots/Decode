package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
@TeleOp
public class Test_tureta_servo_mode extends LinearOpMode {
    Servo s1, s2;
    public static double angle = 180;
    @Override
    public void runOpMode() throws InterruptedException {
        s1 = hardwareMap.get(Servo.class, "servo_right");
        s2 = hardwareMap.get(Servo.class, "servo_left");

        waitForStart();

        while (opModeIsActive()) {
            s1.setPosition(angle * 0.00277);
            s2.setPosition(angle * 0.00277);
        }
    }
}
