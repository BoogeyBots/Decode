package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    Limelight3A camera;

    double gr = 270;


    public void init_teleOP() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");
        camera = hardwareMap.get(Limelight3A.class, "camera");

        pinpoint.setOffsets(-130.0, -11.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //pinpoint.setHeading(-90, AngleUnit.DEGREES);

        //pinpoint.resetPosAndIMU();

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

//    public void update() {
//        controller.setPID(kp, ki, kd);
//        pinpoint.update();
//        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
//        error = (currentHeading + 180) - targetHeading + encoder.getCurrentPosition() / 121.3629;
//        power = controller.calculate(error);
//
//        if(360 + currentHeading > 160 && 360 + currentHeading < 345 && currentHeading < 0)
//            power = 0;
//
//        servo_right.setPower(power);
//        servo_left.setPower(power);
//    }

    public void pinpoint_update() {
        pinpoint.update();
    }

    public void update() {
        controller.setPID(kp, ki, kd);
        double robotX = pinpoint.getPosX(DistanceUnit.INCH);
        double robotY = pinpoint.getPosY(DistanceUnit.INCH);
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        if(currentHeading < 0) {
            currentHeading = 360 + currentHeading;
        }

        double turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;

        double relative_angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        distanta = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        gr = currentHeading + relative_angle;

        error = (currentHeading - 180) - relative_angle + turretCurrentPos;

        power = controller.calculate(error);


        if(gr > 210 && gr < 390) {
            servo_right.setPower(power);
            servo_left.setPower(power);
        }

        else {
            servo_right.setPower(0);
            servo_left.setPower(0);
        }

    }

    // Metoda utilitara pentru normalizare unghi (Angle Wrap)
    private double angle_normalization(double grade) {
        while (grade > 180) {
            grade -= 360;
        }
        while (grade <= -180) {
            grade += 360;
        }
        return grade;
    }

    public void update_auto(double ch) {
        controller.setPID(kp, ki, kd);
        if(ch < 0)
            ch = 360 - Math.abs(ch);
        //pinpoint.update();
        error = (ch) - targetHeading + encoder.getCurrentPosition() / 121.3629;

        //error = (ch + 180) - targetHeading + encoder.getCurrentPosition() / 121.3629;
        power = controller.calculate(error);

//        if(360 + ch > 160 && 360 + ch < 345 && ch < 0)
//            power = 0;

        servo_right.setPower(power);
        servo_left.setPower(power);
    }

    public void update_crazy1() {
        controller.setPID(kp, ki, kd);
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);

        if(currentHeading < 0) {
            currentHeading = 360 - Math.abs(currentHeading);
        }

        double relative_angle = Math.atan2(Math.sqrt(2) * Math.abs(1.4197 * currentX + currentY - 137.3728), 1.73 * Math.abs(currentX - currentY));

        error = currentHeading - targetHeading + encoder.getCurrentPosition() / 121.3629 - relative_angle;

        power = controller.calculate(error);

        servo_right.setPower(power);
        servo_left.setPower(power);

    }

    public void update_crazy2() {
        controller.setPID(kp, ki, kd);
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);

        if(currentHeading < 0) {
            currentHeading = 360 - Math.abs(currentHeading);
        }

        double relative_angle = Math.acos(Math.abs(currentX - currentY) / Math.sqrt(2) * (Math.sqrt(72 - currentX) * (72 - currentX) + (72 - currentY) * (72 - currentY)));

        error = currentHeading - targetHeading + encoder.getCurrentPosition() / 121.3629 + Math.toDegrees(relative_angle);
    }


    //pentru calibrare

    public void calibrare(double kp, double d) {
        this.kp = kp;
        deadband = d;
    }

    public void manual(double p) {
        servo_right.setPower(p);
        servo_left.setPower(p);
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
    public double gra() {return gr;}

}
