package org.firstinspires.ftc.teamcode.Meeturi.Module;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.currentHeading;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.currentX;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.currentY;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.velocityX;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.velocityY;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class TurretModule extends Constants.turret {
    public static double grade_st = 100, grade_dr = 600;
    HardwareMap hardwareMap;

    public TurretModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    CRServo servo_right, servo_left;
    DcMotorEx encoder;
    PIDFController controller = new PIDFController(kp, ki, kd, kf);


    public void init_teleOP() {
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");

        encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        servo_right.setDirection(DcMotorSimple.Direction.REVERSE);
        servo_left.setDirection(DcMotorSimple.Direction.REVERSE);

        controller.reset();

        decalation = 0;
    }

    public void init_auto() {
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");

        servo_right.setDirection(DcMotorSimple.Direction.REVERSE);
        servo_left.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        controller.reset();
    }

    private double calculatePowerWithKS(double error) {
        double p = controller.calculate(error);
        if (Math.abs(error) > 0.1) { // 0.1 degree deadband
            p += Math.signum(error) * ks;
        }
        return p;
    }

    public void update_red() {
        controller.setPIDF(kp, ki, kd, kf);

        if(currentHeading < 0) {
            currentHeading = 360 + currentHeading;
        }

        double turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        relative_angle = Math.toDegrees(Math.atan2(144 - currentY, 144 - currentX));

        gr = currentHeading + relative_angle;

        error = (currentHeading - 180) - relative_angle + turretCurrentPos + decalation;

        power = calculatePowerWithKS(error);


        if(gr > 195 && gr < 390) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }

        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }

    }

    public void update_blue() {
        controller.setPIDF(kp, ki, kd, kf);
        
        turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;
        if(turretCurrentPos > 0) {
            if(currentHeading < 0) currentHeading += 360;
        }

        else currentHeading += 360;

        relative_angle = Math.toDegrees(Math.atan2(141 - currentY, 0 - currentX));

        gr = currentHeading + relative_angle;

        error = (currentHeading - 180) - relative_angle + turretCurrentPos + decalation;

        power = calculatePowerWithKS(error);


        if(gr > 239 && gr < 544 && act_turret) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }

        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }

    }

    public void update_auto_red (double x, double y, double h) {
        turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        if(turretCurrentPos > 0) {
            if(h < 0) h += 360;
        }

        else h += 360;

        if(turretCurrentPos < -8) {
            controller.setPIDF(kp, ki, kd, kf);
        }

        else {
            controller.setPIDF(kp2, 0, kd2, kf2);
            ks = 0.005;
        }

        if (distanta < 120) {
            relative_angle = Math.toDegrees(Math.atan2(144 - y, 149 - x));
        }

        else if(distanta >= 120) {
            relative_angle = Math.toDegrees(Math.atan2(144 - y, 145.5 - x));
        }

        gr = h + relative_angle;

        error = (h - 180) - relative_angle + turretCurrentPos;

        power = calculatePowerWithKS(error);

        if(gr > 239 && gr < 544 && act_turret) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }

        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }
    }

    public void update_auto_blue (double x, double y, double h) {
        turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        if(turretCurrentPos > 0) {
            if(h < 0) h += 360;
        }

        else h += 360;

        if(turretCurrentPos < -8) {
            controller.setPIDF(kp, ki, kd, kf);
        }

        else {
            controller.setPIDF(kp2, 0, kd2, kf2);
            ks = 0.005;
        }

        if (distanta < 120) {
            relative_angle = Math.toDegrees(Math.atan2(144 - y, -5 - x));
        }

        else if(distanta >= 120) {
            relative_angle = Math.toDegrees(Math.atan2(144 - y, -1.5 - x));
        }

        gr = h + relative_angle;

        error = (h - 180) - relative_angle + turretCurrentPos;

        power = calculatePowerWithKS(error);

        if(gr > 239 && gr < 544 && act_turret) {
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
    public double gra() {
        return gr;
    }


    //Compensare cinematică

    public void update_kinematics_blue() {
        turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;
        if(turretCurrentPos > 0) {
            if(currentHeading < 0) currentHeading += 360;
        }

        else currentHeading += 360;

        if(turretCurrentPos < -8) {
            controller.setPIDF(kp, ki, kd, kf);
        }

        else {
            controller.setPIDF(kp2, 0, kd2, kf2);
            ks = 0.005;
        }


        double dx = 0 - currentX;
        double dy = 144 - currentY;
        double realDist = Math.hypot(dx, dy);

        double virtualX, virtualY;

        double transformare_inch = 12.368 / 28;

        double viteza_lansare = velocity * (transformare_inch * frecari);

        if(viteza_lansare < 5) {
            viteza_lansare = 100;
        }

        timp_aer = realDist / viteza_lansare;

        if(distanta < 120) {
            virtualX = 0 - (velocityX * timp_aer * constanta_inertie);
            virtualY = 144 - (velocityY * timp_aer * constanta_inertie);
        }

        else {
            virtualX = 3.5 - (velocityX * timp_aer * constanta_inertie);
            virtualY = 144 - (velocityY * timp_aer * constanta_inertie);
        }

        virtual_distance = Math.hypot(virtualX - currentX, virtualY - currentY);

        relative_angle = Math.toDegrees(Math.atan2(virtualY - currentY, virtualX - currentX));

        gr = currentHeading + relative_angle;

        error = (currentHeading - 180) - relative_angle + turretCurrentPos + decalation;

        power = calculatePowerWithKS(error);

        if(gr > 239 && gr < 544 && act_turret) {
            trage_gresit = false;
            servo_right.setPower(power);
            servo_left.setPower(power);
        }
        else {
            trage_gresit = true;
            servo_right.setPower(0);
            servo_left.setPower(0);
        }
    }

    public void update_kinematics_red() {
        turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;
        if(turretCurrentPos > 0) {
            if(currentHeading < 0) currentHeading += 360;
        }

        else currentHeading += 360;

        if(turretCurrentPos < -8) {
            controller.setPIDF(kp, ki, kd, kf);
        }

        else {
            controller.setPIDF(kp2, 0, kd2, kf2);
            ks = 0.005;
        }


        double dx = 144 - currentX;
        double dy = 144 - currentY;
        double realDist = Math.hypot(dx, dy);

        double virtualX, virtualY;

        double transformare_inch = 12.368 / 28;

        double viteza_lansare = velocity * (transformare_inch * frecari);

        if(viteza_lansare < 5) {
            viteza_lansare = 100;
        }

        timp_aer = realDist / viteza_lansare;

        if(distanta < 120) {
            virtualX = 144 - (velocityX * timp_aer * constanta_inertie);
            virtualY = 144 - (velocityY * timp_aer * constanta_inertie);
        }

        else {
            virtualX = 140.5 - (velocityX * timp_aer * constanta_inertie);
            virtualY = 144 - (velocityY * timp_aer * constanta_inertie);
        }

        virtual_distance = Math.hypot(virtualX - currentX, virtualY - currentY);

        relative_angle = Math.toDegrees(Math.atan2(virtualY - currentY, virtualX - currentX));

        gr = currentHeading + relative_angle;

        error = (currentHeading - 180) - relative_angle + turretCurrentPos + decalation;

        power = calculatePowerWithKS(error);

        if(gr > 239 && gr < 544 && act_turret) {
            trage_gresit = false;
            servo_right.setPower(power);
            servo_left.setPower(power);
        }
        else {
            trage_gresit = true;
            servo_right.setPower(0);
            servo_left.setPower(0);
        }
    }

    public void update_auto_blue_kinematics (double x, double y, double h, double vx, double vy) {
        controller.setPIDF(kp, ki, kd, kf);
        
        if(h < 0) {
            h = 360 + h;
        }

        double dx = 0 - x;
        double dy = 144 - y;
        double realDist = Math.hypot(dx, dy);

        double transformare_inch = 12.368 / 28;

        double viteza_lansare = velocity * (transformare_inch * frecari);

        if(viteza_lansare < 5) {
            viteza_lansare = 100;
        }

        timp_aer = realDist / viteza_lansare;

        double virtualX = 0 - (vx * timp_aer * constanta_inertie);
        double virtualY = 144 - (vy * timp_aer * constanta_inertie);

        virtual_distance = Math.hypot(virtualX - x, virtualY - y);

        double relative_angle = Math.toDegrees(Math.atan2(virtualY - y, virtualX - x));

        double turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;
        gr = h + relative_angle;
        error = (h - 180) - relative_angle + turretCurrentPos + decalation;

        power = calculatePowerWithKS(error);

        if(gr > 350 && gr < 560 && act_turret) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }
        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }
    }

    public double getPower() {
        return servo_right.getPower();
    }

}
