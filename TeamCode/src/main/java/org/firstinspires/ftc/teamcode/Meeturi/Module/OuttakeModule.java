package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class OuttakeModule {
    HardwareMap hardwareMap;
    public OuttakeModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotorEx motor, motor2;
    Servo servo;
    double kp = 0, ki = 0, kd = 0;
    int target_velocity;

    PIDController controller = new PIDController(kp, ki, kd);

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor_outtake1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor_outtake2");
        servo = hardwareMap.get(Servo.class, "angle");

        controller.reset();
    }

    public void update() {
        controller.setPID(kp, ki, kd);
        double output = controller.calculate(motor.getVelocity(), target_velocity);

        motor.setVelocity(output);
        motor2.setVelocity(output);

    }

    public void stop() {
        motor.setPower(0);
        motor2.setPower(0);
    }

    public void custom_angle(double pos) {
        servo.setPosition(pos);
    }

    public double voltage1() {return motor.getCurrent(CurrentUnit.MILLIAMPS);}
    public double voltage2() {return motor2.getCurrent(CurrentUnit.MILLIAMPS);}
}
