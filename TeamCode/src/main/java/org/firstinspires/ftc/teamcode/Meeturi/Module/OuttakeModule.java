package org.firstinspires.ftc.teamcode.Meeturi.Module;


import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.velocityX;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.velocityY;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.gr;

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
    public static double vel50_60 = 960, vel60_70 = 970, vel70_75 = 1045, vel75_80 = 1140, vel80_86 = 1200, vel86_95 = 1180, vel95_105 = 1250, vel105_116 = 1320, vel120 = 1450, vel144 = 1240, progresie = 4.8; //1060 1090
    public static double p50_60 = 0, p60_70 = 0.1, p70_75 = 0.2, p75_80 = 0.34, p80_86 = 0.49, p86_95 = 0.41, p95_105 = 0.43, p105_116 = 0.46, p120 = 0.57, p_120auto = 0.65;
    public static double compaprapr=0.2, compaprdep=0.25, compdep=0.28;

    PIDFController controller = new PIDFController(kp, 0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

    public void init() {
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");

        servo_rampa = hardwareMap.get(Servo.class, "servo_rampa");
        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

//        motorST.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDR.setDirection(DcMotorSimple.Direction.REVERSE);

        blocat();
        aproape();


        target_velocity = 0;
    }


    public void init_auto_departe() {
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");

        servo_rampa = hardwareMap.get(Servo.class, "servo_rampa");
        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");

        motorDR.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


        servo_rampa.setPosition(0.6);
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

        if(distanta <= 76 && distanta > 70 && act_outtake) {
            target_velocity = vel70_75;
            if(!ramp) {
                servo_rampa.setPosition(p70_75);
            }
            zone = 3;
            timer.reset();
        }

        if(distanta <= 80 && distanta > 76 && act_outtake) {
            target_velocity = vel75_80;
            if(!ramp) {
                servo_rampa.setPosition(p75_80);
            }
            zone = 4;
            timer.reset();
        }


        if(distanta <= 86.5 && distanta > 80 && act_outtake) {
            target_velocity = vel80_86;
            if(!ramp) {
                servo_rampa.setPosition(p80_86);
            }
            zone = 5;
            timer.reset();
        }

        if(distanta <= 95 && distanta > 86.5 && act_outtake) {
            target_velocity = vel86_95;
            if(!ramp) {
                servo_rampa.setPosition(p86_95);
            }
            zone = 6;
            timer.reset();
        }

        if(distanta <= 105 && distanta > 95 && act_outtake) {
            target_velocity = vel95_105;
            if(!ramp) {
                servo_rampa.setPosition(p95_105);
            }
            zone = 7;
            timer.reset();
        }

        if(distanta <= 116 && distanta > 105 && act_outtake) {
            target_velocity = vel105_116;
            if(!ramp) {
                servo_rampa.setPosition(p105_116);
            }
            zone = 8;
            timer.reset();
        }

        if(distanta > 116 && distanta <= 140 && act_outtake) {
            if(!ramp) {
                servo_rampa.setPosition(p120);
                target_velocity = vel120;
            }
            zone = 9;
            timer.reset();
        }

        if(distanta > 140 && act_outtake) {
            if(!ramp) {
                servo_rampa.setPosition(0);
                target_velocity = vel144;
            }
            zone = 10;
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
        //servo_rampa.setPosition(aproape);
        target_velocity = 1200;
    }

    public void reglare_departe() {
        servo_rampa.setPosition(compdep);
        ramp = true;
    }

    public void reglare_departe_auto() {
        servo_rampa.setPosition(0.43);
        ramp = true;
    }

    public void reglare_aproape_far() {
        servo_rampa.setPosition(compaprdep);
        ramp = true;
    }

    public void reglare_aproape_aproape() {
        servo_rampa.setPosition(compaprapr);
        ramp = true;
    }

    public void blocat() {
        servo_blocaj.setPosition(blocat);
    }

    public void deblocat() {
        servo_blocaj.setPosition(deblocat);
    }


    //Compesare cinematica

    public void update_kinematics() {
        velocity = motorDR.getVelocity();

        controller.setPIDF(kp, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        if (distanta <= 50 && act_outtake && auto) {
            target_velocity = 1100;
            if(!ramp)
                servo_rampa.setPosition(0);
        }

        if (distanta <= 54 && act_outtake && !auto) {
            target_velocity = 0;
            if(!ramp)
                servo_rampa.setPosition(0);
        }

        if (distanta <= 60 && distanta > 55 && act_outtake) {
            target_velocity = vel50_60;
            if(!ramp)
                servo_rampa.setPosition(p50_60);
            zone = 1;
            timer.reset();
        }

        if (distanta <= 70 && distanta > 60 && act_outtake) {
            target_velocity = vel60_70;
            if(!ramp)
                servo_rampa.setPosition(p60_70);
            zone = 2;
            timer.reset();
        }

        if (distanta <= 76 && distanta > 70 && act_outtake) {
            target_velocity = vel70_75;
            if(!ramp)
                servo_rampa.setPosition(p70_75);
            zone = 3;
            timer.reset();
        }

        if (distanta <= 80 && distanta > 76 && act_outtake) {
            target_velocity = vel75_80;
            if(!ramp)
                servo_rampa.setPosition(p75_80);
            zone = 4;
            timer.reset();
        }


        if (distanta <= 86.5 && distanta > 80 && act_outtake) {
            target_velocity = vel80_86;
            if(!ramp)
                servo_rampa.setPosition(p80_86);
            zone = 5;
            timer.reset();
        }

        if (distanta <= 95 && distanta > 86.5 && act_outtake) {
            target_velocity = vel86_95;
            if(!ramp)
                servo_rampa.setPosition(p86_95);
            zone = 6;
            timer.reset();
        }

        if (distanta <= 105 && distanta > 95 && act_outtake) {
            target_velocity = vel95_105;
            if(!ramp)
                servo_rampa.setPosition(p95_105);
            zone = 7;
            timer.reset();
        }

        if (distanta <= 116 && distanta > 105 && act_outtake) {
            target_velocity = vel105_116;
            if(!ramp)
                servo_rampa.setPosition(p105_116);
            zone = 8;
            timer.reset();
        }

//        if(distanta > 120 && distanta <= 140 && act_outtake) {
//            if(!ramp) {
//                servo_rampa.setPosition(p120);
//                target_velocity = vel120;
//            }
//            zone = 9;
//            timer.reset();
//        }
//
//        if(distanta > 140 && act_outtake) {
//            if(!ramp) {re
//                servo_rampa.setPosition(0);
//                target_velocity = vel144;
//            }
//            zone = 10;
//            timer.reset();
//        }

        if (distanta > 116 && act_outtake) {
            double extra_power = distanta - 120;
            if(!ramp) {
                servo_rampa.setPosition(p120);
            }
            target_velocity = vel120 + extra_power * progresie;

            zone = 10;
        }

        if (target_velocity == 1100) {
            double PID_output = controller.calculate(velocity, 1100);
            double ff_output = feedforward.calculate(1100);
            double output = PID_output + ff_output;

            motorDR.setPower(output * (nominalvoltage / voltage));
            motorST.setPower(output * (nominalvoltage / voltage));
        }

        else if (target_velocity == 1400) {
            double PID_output = controller.calculate(velocity, 1400);
            double ff_output = feedforward.calculate(1400);
            double output = PID_output + ff_output;

            motorDR.setPower(output * (nominalvoltage / voltage));
            motorST.setPower(output * (nominalvoltage / voltage));
        }

        else {
            double gr_rad = Math.toRadians(gr);

            double viteza_compusa = velocityX * Math.cos(gr_rad) + velocityY * Math.sin(gr_rad);

            final_target = target_velocity - (viteza_compusa * factor_corectie);

            double PID_output = controller.calculate(velocity, final_target);
            double ff_output = feedforward.calculate(final_target);
            double output = PID_output + ff_output;

            motorDR.setPower(output * (nominalvoltage / voltage));
            motorST.setPower(output * (nominalvoltage / voltage));
        }
    }

    public void update_auto() {
        velocity = motorDR.getVelocity();

        controller.setPIDF(kp, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        double PID_output = controller.calculate(velocity, 1245);
        double ff_output = feedforward.calculate(1245);
        double output = PID_output + ff_output;

        motorDR.setPower(output * (nominalvoltage / voltage));
        motorST.setPower(output * (nominalvoltage / voltage));
    }

    public void update_auto_far() {
        velocity = motorDR.getVelocity();

        controller.setPIDF(kp, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        double extra_power = distanta - 120;
        target_velocity = vel120 + extra_power * progresie;

        double PID_output = controller.calculate(velocity, target_velocity);
        double ff_output = feedforward.calculate(target_velocity);
        double output = PID_output + ff_output;

        motorDR.setPower(output * (nominalvoltage / voltage));
        motorST.setPower(output * (nominalvoltage / voltage));
    }



    public void rampa(double x) {
        servo_rampa.setPosition(x);
    }


}
