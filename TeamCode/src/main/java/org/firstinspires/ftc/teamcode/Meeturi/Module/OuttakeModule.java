package org.firstinspires.ftc.teamcode.Meeturi.Module;


import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class OuttakeModule extends Constants.outtake {
    HardwareMap hardwareMap;

    public OuttakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotorEx motor_sus, motor_jos, motor_opus;
    Servo servo_rampa, servo_blocaj;

    PIDFController controller = new PIDFController(kp, ki, kd, kf);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

    public void init_teleOP() {
        motor_sus = hardwareMap.get(DcMotorEx.class, "motor_sus");
        motor_jos = hardwareMap.get(DcMotorEx.class, "motor_jos");
        motor_opus = hardwareMap.get(DcMotorEx.class, "motor_opus");
        servo_rampa = hardwareMap.get(Servo.class, "servo_rampa");
        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");

        motor_sus.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_jos.setDirection(DcMotorSimple.Direction.REVERSE);

        blocat();
        aproape();

        target_velocity = 0;

        //controller.reset();
    }

    public void init_auto_aproape() {
        motor_sus = hardwareMap.get(DcMotorEx.class, "motor_sus");
        motor_jos = hardwareMap.get(DcMotorEx.class, "motor_jos");
        motor_opus = hardwareMap.get(DcMotorEx.class, "motor_opus");
        servo_rampa = hardwareMap.get(Servo.class, "servo_rampa");
        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");

        motor_sus.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_jos.setDirection(DcMotorSimple.Direction.REVERSE);



        blocat();
        aproape();

        target_velocity = 0;

        //controller.reset();
    }

    public void update() {
        controller.setPIDF(kp, ki, kd, kf); //kp=4
        //feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        double output = controller.calculate(motor_opus.getVelocity(), target_velocity); //-2100

//        if(target_velocity == 0) {
//            controller.setP(0);
//        }
//
//        else {
//            controller.setP(kp);
//        }

        if(distanta <= 50 && activated) {
            target_velocity = 0;
            servo_rampa.setPosition(0);
        }

        if(distanta <= 60 && distanta > 50 && activated) {
            target_velocity = 1400;
            servo_rampa.setPosition(0.3);
        }

        if(distanta <= 70 && distanta > 60 && activated) {
            target_velocity = 1275;
            servo_rampa.setPosition(0.1);
        }

        if(distanta <= 75 && distanta > 70 && activated) {
            target_velocity = 1350;
            servo_rampa.setPosition(0.1);
        }

        if(distanta <= 80 && distanta > 76 && activated) {
            target_velocity = 1380;
            servo_rampa.setPosition(0.1);
        }


        if(distanta <= 110 && distanta > 80 && activated) {
            target_velocity = 1400;
            servo_rampa.setPosition(0.14);
        }

        if(distanta >= 120 && activated) {
            target_velocity = 1800;
            servo_rampa.setPosition(0.5);
        }

        velocity = motor_opus.getVelocity();

        if(target_velocity != 0) {
            motor_sus.setPower(output);
            motor_jos.setPower(output);
            motor_opus.setPower(output);
        }

        else {
            motor_sus.setPower(0);
            motor_jos.setPower(0);
            motor_opus.setPower(0);
        }
    }

    public void calibrare() {
        controller.setPIDF(kp, ki, kd, kf); //kp=4
        //feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        double output = controller.calculate(motor_opus.getVelocity(), target_velocity); //-2100
        velocity = motor_opus.getVelocity();

        servo_rampa.setPosition(pos_servo);


        if(target_velocity != 0) {
            motor_sus.setPower(output);
            motor_jos.setPower(output);
            motor_opus.setPower(output);
        }

        else {
            motor_sus.setPower(0);
            motor_jos.setPower(0);
            motor_opus.setPower(0);
        }


    }


    public void aproape() {
        servo_rampa.setPosition(aproape);
        target_velocity = 1200;
    }

    public void blocat() {
        servo_blocaj.setPosition(blocat);
    }

    public void deblocat() {
        servo_blocaj.setPosition(deblocat);
    }


}
