package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeModule {
    HardwareMap hardwareMap;
    public OuttakeModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotorEx motor;
    Servo servo;

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor_outtake");
        servo = hardwareMap.get(Servo.class, "angle");
    }

    public void trage(double power) {
        motor.setPower(power);
        //probabil o sa avem o functie pentru far si una pentru close, vedem
    }

    public void stop() {
        motor.setPower(0);
    }

    public void angle_close() {
        servo.setPosition(0);
    }

    public void angle_far() {
        servo.setPosition(1);
    }

    public void custom_angle(double pos) {
        servo.setPosition(pos);
    }
}
