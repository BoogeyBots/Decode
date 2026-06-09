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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class TurretModule extends Constants.turret {
    public static double grade_st = 100, grade_dr = 600;
    HardwareMap hardwareMap;

    public TurretModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    Servo servo_right, servo_left;
    DcMotorEx encoder;
    PIDFController controller = new PIDFController(kp, ki, kd, kf);


    public void init() {
        servo_right = hardwareMap.get(Servo.class, "servo_right");
        servo_left = hardwareMap.get(Servo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");

        decalation = 0;
    }

    public double getError() {
        return error;
    }
    public double gra() {
        return gr;
    }

    public void up() {
        if(distanta < 120) {
            relative_angle = Math.toDegrees(Math.atan2(144 - currentY, 0 - currentX));
        }

        else relative_angle = Math.toDegrees(Math.atan2(144 - currentY, 5 - currentX));

//        double turretAngle = relative_angle - currentHeading;

        double turretAngle = AngleUnit.normalizeDegrees(relative_angle - currentHeading);



        double servoPos;

        if (Math.abs(turretAngle) > 150) {
            servoPos = 0.5;
        } else {
            servoPos = 0.5 + (turretAngle / 320.0) + decalation;
        }

        double limitMin = 30.0 / 360.0;
        double limitMax = 330.0 / 360.0;

        double safeServoPosition = Range.clip(servoPos, limitMin, limitMax);

        servo_right.setPosition(safeServoPosition);
        servo_left.setPosition(safeServoPosition);
    }

    public void up_auto(double x, double y, double h) {
        if(distanta < 120) {
            relative_angle = Math.toDegrees(Math.atan2(144 - y, 0 - x));
        }

        else relative_angle = Math.toDegrees(Math.atan2(144 - y, 5 - x));

//        double turretAngle = relative_angle - currentHeading;

        double turretAngle = AngleUnit.normalizeDegrees(relative_angle - h);



        double servoPos;

        if (Math.abs(turretAngle) > 150) {
            servoPos = 0.5;
        } else {
            servoPos = 0.5 + (turretAngle / 320.0) + decalation;
        }

        double limitMin = 30.0 / 360.0;
        double limitMax = 330.0 / 360.0;

        double safeServoPosition = Range.clip(servoPos, limitMin, limitMax);

        servo_right.setPosition(safeServoPosition);
        servo_left.setPosition(safeServoPosition);
    }

}
