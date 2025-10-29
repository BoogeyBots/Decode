package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeModule extends Constants.intake {
    HardwareMap hardwareMap;
    public IntakeModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    DcMotorEx motor;
    Servo servo;
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");
        servo = hardwareMap.get(Servo.class, "servo_intake");
        sus();
    }

    public void trage(double power) {
        motor.setPower(power);
    }

    public void scuipa(double power) {
        motor.setPower(-power);
    }

    public void stop() {
        motor.setPower(0);
    }

    public void poz_servo(double poz) {
        servo.setPosition(poz);
    }

    public void jos() {
        servo.setPosition(jos);
    }

    public void sus() {
        servo.setPosition(sus);
    }

}