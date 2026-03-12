package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MGNModule extends Constants.intake {
    HardwareMap hardwareMap;
    public MGNModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    Servo servo;
    CRServo left, right;
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo_sclav");
        left = hardwareMap.get(CRServo.class, "ridicare_left");
        right = hardwareMap.get(CRServo.class, "ridicare_right");

        servo.setPosition(0);
    }

    public void ridicare() {
        servo.setPosition(0.5);
        if(servo.getPosition() == 0.5) {
            left.setPower(1);
            right.setPower(1);
        }
    }

}