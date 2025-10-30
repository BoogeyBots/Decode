package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TurretModule extends Constants.turret {
    HardwareMap hardwareMap;
    public TurretModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    GoBildaPinpointDriver pinpoint;
    CRServo servo_right, servo_left;
    DcMotorEx encoder;
    PIDController controller = new PIDController(kp, ki, kd);


    public void init_teleOP() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");

        pinpoint.setOffsets(-130.0, -11.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller.reset();
    }

    public void init_auto() {
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller.reset();
    }

//    public void update() {
//        pinpoint.update();
//        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES) * 121.3629;
//        double error = targetHeading - currentHeading - encoder.getCurrentPosition(); //error = targetHeading - currentHeading - encoder;
//
//        power = kp * error;
//        servo_right.setPower(power);
//        servo_left.setPower(power);
//    }

    public void update() {
        controller.setPID(kp, ki, kd);
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        error = (currentHeading + 180) - targetHeading + encoder.getCurrentPosition() / 121.3629;
        power = controller.calculate(error);

        if(360 + currentHeading > 160 && 360 + currentHeading < 345 && currentHeading < 0)
            power = 0;

        servo_right.setPower(power);
        servo_left.setPower(power);
    }

    public void update_auto(double ch) {
        controller.setPID(kp, ki, kd);
        //pinpoint.update();
        error = (ch + 180) - targetHeading + encoder.getCurrentPosition() / 121.3629;
        power = controller.calculate(error);

        if(360 + ch > 160 && 360 + ch < 345 && ch < 0)
            power = 0;

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
    public double getErrore() {return error;}
    public double getPower() {
        return power;
    }

    public double getkP() {
        return kp;
    }

}
