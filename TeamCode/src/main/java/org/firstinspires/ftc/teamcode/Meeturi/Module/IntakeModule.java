package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeModule extends Constants.Intake {
    DcMotorEx motor;
    public enum IntakeCases {
        TAKE,
        EJECT,
        OFF
    }
    public IntakeCases state = null;

    public IntakeModule (HardwareMap hardwareMap) {
        init(hardwareMap);
    }
    void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");

        state = IntakeCases.OFF;
    }
    public Actions actions;
    public class Actions {
        public void take() {
            motor.setPower(powerMotorTake);
        }
    
        public void eject() {
            motor.setPower(powerMotorEject);
        }
    
        public void off() {
            motor.setPower(powerMotorOff);
        }
    }
}