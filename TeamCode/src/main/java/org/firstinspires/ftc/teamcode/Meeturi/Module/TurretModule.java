package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TurretModule extends Constants.turret {
    HardwareMap hardwareMap;
    public TurretModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    GoBildaPinpointDriver pinpoint;
    CRServo servo_right, servo_left;

    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");

        pinpoint.setOffsets(-130.0, -11.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        error = targetHeading - currentHeading;

        /*
        Daca oscileaza, implementam "banda moarta"
        if(Math.abs(error) < deadband) {
            s1.setPower(0);
            s2.setPower(0);
            return;
        }

         */

        power = kp * error;
        servo_right.setPower(power);
        servo_left.setPower(power);
    }


    //pentru calibrare

    public void calibrare(double kp, double d) {
        this.kp = kp;
        deadband = d;
    }

    public double getHeading() {
        return currentHeading;
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

}
