package org.firstinspires.ftc.teamcode.Meeturi.Module;


import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class OuttakeModule extends Constants.outtake {
    HardwareMap hardwareMap;

    public OuttakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotorEx motorDR, motorST;
    Servo servo_rampa, servo_blocaj;
    ElapsedTime timer;
    public static double vel50_60 = 1080, vel60_70 = 1100, vel70_75 = 1080, vel75_80 = 1120, vel80_90 = 1150, vel90_110 = 1200, vel120 = 1550;
    public static double p50_60 = 0, p60_70 = 0, p70_75 = 0, p75_80 = 0, p80_90 = 0, p90_110 = 0.12, p120 = 0.74;
    public static double reg50_60 = 0, reg60_70 = 0, reg70_75 = 0, reg75_80 = 0, reg80_90 = 0, reg90_110 = 0, reg120 = 0.45, pow = 50;

    PIDFController controller = new PIDFController(kp, 0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

    public void init_teleOP() {
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");

        servo_rampa = hardwareMap.get(Servo.class, "servo_rampa");
        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        motorST.setDirection(DcMotorSimple.Direction.REVERSE);

        blocat();
        aproape();

        target_velocity = 0;
    }

    public void init_auto_aproape() {
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");

        servo_rampa = hardwareMap.get(Servo.class, "servo_rampa");
        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");

        motorST.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        blocat();
        aproape();

        target_velocity = 0;
    }

    public void update() {
        velocity = motorDR.getVelocity();

        controller.setPIDF(kp, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        double PID_output = controller.calculate(velocity, target_velocity);
        double ff_output = feedforward.calculate(target_velocity);
        double output = PID_output + ff_output;

        if(distanta <= 50 && act_outtake && auto) {
            target_velocity = 1100;
            servo_rampa.setPosition(0);
        }

        if(distanta <= 54 && act_outtake && !auto) {
            target_velocity = 0;
            servo_rampa.setPosition(0);
        }

        if(distanta <= 60 && distanta > 55 && act_outtake) {
            target_velocity = vel50_60;
            if(!ramp) {
                servo_rampa.setPosition(p50_60);
            }
            zone = 1;
            timer.reset();
        }

        if(distanta <= 70 && distanta > 60 && act_outtake) {
            target_velocity = vel60_70;
            if(!ramp) {
                servo_rampa.setPosition(p60_70);
            }
            zone = 2;
            timer.reset();
        }

        if(distanta <= 75 && distanta > 70 && act_outtake) {
            target_velocity = vel70_75;
            if(!ramp) {
                servo_rampa.setPosition(p70_75);
            }
            zone = 3;
            timer.reset();
        }

        if(distanta <= 80 && distanta > 75 && act_outtake) {
            target_velocity = vel75_80;
            if(!ramp) {
                servo_rampa.setPosition(p75_80);
            }
            zone = 4;
            timer.reset();
        }


        if(distanta <= 89 && distanta > 80 && act_outtake) {
            target_velocity = vel80_90;
            if(!ramp) {
                servo_rampa.setPosition(p80_90);
            }
            zone = 5;
            timer.reset();
        }

        if(distanta <= 110 && distanta > 89 & act_outtake) {
            target_velocity = vel90_110;
            if(!ramp) {
                servo_rampa.setPosition(p90_110);
            }
            zone = 6;
            timer.reset();
        }

        if(distanta >= 110 && act_outtake) {
            if(!ramp) {
                servo_rampa.setPosition(p120);
                target_velocity = vel120 + pow;
            }
            zone = 7;
            timer.reset();
        }

        if(target_velocity != 0) {
            motorDR.setPower(output * (nominalvoltage / voltage));
            motorST.setPower(output * (nominalvoltage / voltage));
        }

        else {
            motorDR.setPower(0);
            motorST.setPower(0);
        }
    }


    public void aproape() {
        servo_rampa.setPosition(aproape);
        target_velocity = 1200;
    }

    public void reglare() {
        switch (zone) {
            case 1:
                servo_rampa.setPosition(reg50_60);

                break;

            case 2:
                servo_rampa.setPosition(reg60_70);

                break;

            case 3:
                servo_rampa.setPosition(reg70_75);

                break;

            case 4:
                servo_rampa.setPosition(reg75_80);

                break;

            case 5:
                servo_rampa.setPosition(reg80_90);

                break;

            case 6:
                servo_rampa.setPosition(reg90_110);

                break;

            case 7:
                servo_rampa.setPosition(reg120);
                target_velocity = vel120 + 40;

                break;

        }
        ramp = true;
    }

    public void blocat() {
        servo_blocaj.setPosition(blocat);
    }

    public void deblocat() {
        servo_blocaj.setPosition(deblocat);
    }


}
