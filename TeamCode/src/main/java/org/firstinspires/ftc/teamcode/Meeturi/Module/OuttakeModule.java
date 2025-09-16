package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeModule extends Constants.Outtake {
    DcMotorEx motor;
    Servo servo;

    public enum OuttakeCases {
        SHOOT,
        OFF
    }

    public OuttakeCases state = null;

    public OuttakeModule (HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_outtake");
        servo = hardwareMap.get(Servo.class, "angle");

        state = OuttakeCases.OFF;
    }

    public Actions actions;
    public class Actions {
        public void shoot() {
            motor.setPower(powerMotorShoot);
            //probabil o sa avem o functie pentru far si una pentru close, vedem
        }
    
        public void off() {
            motor.setPower(powerMotorOff);
        }
    
        public void angle_close() {
            servo.setPosition(positionServoAngleClose);
        }
    
        public void angle_far() {
            servo.setPosition(positionServoAngleFar);
        }
    
        public void custom_angle() {
            servo.setPosition(positionServoAngleCustom);
        }
    }
}
