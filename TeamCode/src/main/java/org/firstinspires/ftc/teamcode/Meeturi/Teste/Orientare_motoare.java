package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp
public class Orientare_motoare extends LinearOpMode {
    DcMotor motor_sus, motor_jos, motor_opus;
    public static boolean ms = false, mj = false, mo = false;
    @Override
    public void runOpMode() throws InterruptedException {
        motor_sus = hardwareMap.get(DcMotorEx.class, "motor_sus");
        motor_jos = hardwareMap.get(DcMotorEx.class, "motor_jos");
        motor_opus = hardwareMap.get(DcMotorEx.class, "motor_opus");

        motor_sus.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_jos.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            if(ms) {
                motor_sus.setPower(1);
            }
            else {
                motor_sus.setPower(0);
            }

            if(mj) {
                motor_jos.setPower(1);
            }
            else {
                motor_jos.setPower(0);
            }

            if(mo) {
                motor_opus.setPower(1);
            }
            else {
                motor_opus.setPower(0);
            }
        }
    }
}
