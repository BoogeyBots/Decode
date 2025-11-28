package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@Configurable
@Disabled
@TeleOp
public class Test_tureta_manual extends LinearOpMode {
    CRServo servo1;
    CRServo servo2;
    public static double power;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(CRServo.class, "servo_left");
        servo2 = hardwareMap.get(CRServo.class, "servo_right");
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.right_trigger > 0.01) {
                servo1.setPower(gamepad1.right_trigger);
                servo2.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger > 0.01) {
                servo1.setPower(-gamepad1.left_trigger);
                servo2.setPower(-gamepad1.left_trigger);
            }
            else {
                servo1.setPower(0);
                servo2.setPower(0);
            }

//            servo1.setPower(power);
//            servo2.setPower(power);
        }

    }
}

