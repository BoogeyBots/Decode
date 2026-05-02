package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class Automatizare extends OpMode {
    DcMotorEx motor;
    ElapsedTime timer;
    boolean conditie = false;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    @Override
    public void loop() {
        if(!conditie) {
            motor.setPower(1);
            timer.reset();
            conditie = true;
        }

        if(timer.seconds() > 3 && conditie) {
            motor.setPower(0);
            //  conditie = false;
        }
    }
}
