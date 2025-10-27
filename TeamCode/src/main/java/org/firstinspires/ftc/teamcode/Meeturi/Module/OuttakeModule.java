package org.firstinspires.ftc.teamcode.Meeturi.Module;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Configurable
public class OuttakeModule extends Constants.outtake {
    HardwareMap hardwareMap;

    public OuttakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotorEx motor_sus, motor_jos, motor_opus;
    Servo servo_rampa, servo_blocaj;

    PIDController controller = new PIDController(kp, ki, kd);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

    public void init() {
        motor_sus = hardwareMap.get(DcMotorEx.class, "motor_sus");
        motor_jos = hardwareMap.get(DcMotorEx.class, "motor_jos");
        motor_opus = hardwareMap.get(DcMotorEx.class, "motor_opus");
        servo_rampa = hardwareMap.get(Servo.class, "servo_rampa");
        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");

        motor_sus.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_jos.setDirection(DcMotorSimple.Direction.REVERSE);

        controller.reset();
    }

    public void update() {
        controller.setPID(kp, ki, kd); //kp=4
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        double PID_output = controller.calculate(motor_sus.getVelocity(), target_velocity); //-2100
        double FF_output = feedforward.calculate(target_velocity);
        double output = PID_output + FF_output;

        motor_sus.setVelocity(output);
        motor_jos.setVelocity(output);
        motor_opus.setVelocity(output);
    }

    public double get_velocity_sus() {
        return motor_sus.getVelocity();
    }

    public double get_target() {
        return target_velocity;
    }

    public void departe() {
        servo_rampa.setPosition(1);
        target_velocity = 1600;
    }

    public void aproape() {
        servo_rampa.setPosition(0.5);
        target_velocity = 1200;
    }

    public void stop() {
        target_velocity = 0;
    }

    public void trage() {
        motor_sus.setPower(1);
        motor_jos.setPower(1);
        motor_opus.setPower(1);
    }

    public void servo_custom() {
        servo_rampa.setPosition(pos_servo);
    }

    public void blocat() {
        servo_blocaj.setPosition(blocat);
    }

    public void deblocat() {
        servo_blocaj.setPosition(deblocat);
    }


}
