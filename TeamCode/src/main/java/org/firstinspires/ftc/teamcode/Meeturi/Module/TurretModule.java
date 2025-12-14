package org.firstinspires.ftc.teamcode.Meeturi.Module;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.currentHeading;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.currentX;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.currentY;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretModule extends Constants.turret {
    HardwareMap hardwareMap;
    public TurretModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    CRServo servo_right, servo_left;
    DcMotorEx encoder;
    PIDController controller = new PIDController(kp, ki, kd);

    double gr = 270;


    public void init_teleOP() {
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");

        controller.reset();

        decalation = 0;
    }

    public void init_auto() {
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller.reset();
    }

    public void update_red() {
        controller.setPID(kp, ki, kd);

        if(currentHeading < 0) {
            currentHeading = 360 + currentHeading;
        }

        double turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        relative_angle = Math.toDegrees(Math.atan2(144 - currentY, 144 - currentX));

        gr = currentHeading + relative_angle;

        error = (currentHeading - 180) - relative_angle + turretCurrentPos + decalation;

        power = controller.calculate(error);


        if(gr > 180 && gr < 390) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }

        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }

    }

    public void update_blue() {
        controller.setPID(kp, ki, kd);

        currentHeading += 360;

        double turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        relative_angle = Math.toDegrees(Math.atan2(144 - currentY, 0 - currentX));

        gr = currentHeading + relative_angle;

        error = (currentHeading - 180) - relative_angle + turretCurrentPos;

        power = controller.calculate(error);


        if(gr > 338 && gr < 560) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }

        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }

    }



    public void update_auto_red (double x, double y, double h) {
        controller.setPID(kp, ki, kd);

        if(h < 0) {
            h = 360 + h;
        }

        double turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        relative_angle = Math.toDegrees(Math.atan2(144 - y, 144 - x));

        gr = h + relative_angle;

        error = (h - 180) - relative_angle + turretCurrentPos;

        power = controller.calculate(error);


        if(gr > 165 && gr < 390 && act_turret) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }

        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }
    }

    public void update_auto_blue (double x, double y, double h) {
        controller.setPID(kp, ki, kd);

        h += 360;

        double turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        relative_angle = Math.toDegrees(Math.atan2(144 - y, 0 - x));

        gr = h + relative_angle;

        error = (h - 180) - relative_angle + turretCurrentPos;

        power = controller.calculate(error);

        if(gr > 338 && gr < 560) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }

        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }
    }


    public double getError() {
        return error;
    }
    public double getPower() {
        return power;
    }

    public double getkP() {
        return kp;
    }
    public double gra() {
        return gr;
    }

}
