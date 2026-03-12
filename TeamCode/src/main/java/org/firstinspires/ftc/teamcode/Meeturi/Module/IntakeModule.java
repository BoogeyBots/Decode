package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeModule extends Constants.intake {
    HardwareMap hardwareMap;
    public IntakeModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    DcMotorEx motor_intake, motor_transfer;
    DigitalChannel senzor_intake;
    DistanceSensor senzor_mij;
    Servo servo;

    private double lastIntakePower = 0;
    private double lastTransferPower = 0;

    public void init() {
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        senzor_intake = hardwareMap.get(DigitalChannel.class, "senzor_intake");

        senzor_intake.setMode(DigitalChannel.Mode.INPUT);


        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void trage_intake(double power) {
        double target = -power;
        if (Math.abs(target - lastIntakePower) > 0.005) {
            motor_intake.setPower(target);
            lastIntakePower = target;
        }
    }

    public void scuipa_intake(double power) {
        double target = power;
        if (Math.abs(target - lastIntakePower) > 0.005) {
            motor_intake.setPower(target);
            lastIntakePower = target;
        }
    }

    public void stop_intake() {
        if (lastIntakePower != 0) {
            motor_intake.setPower(0);
            lastIntakePower = 0;
        }
    }

    public void trage_transfer(double power) {
        double target = power;
        if (Math.abs(target - lastTransferPower) > 0.005) {
            motor_transfer.setPower(target);
            lastTransferPower = target;
        }
    }

    public void scuipa_transfer(double power) {
        double target = -power;
        if (Math.abs(target - lastTransferPower) > 0.005) {
            motor_transfer.setPower(target);
            lastTransferPower = target;
        }
    }

    public void stop_transfer() {
        if (lastTransferPower != 0) {
            motor_transfer.setPower(0);
            lastTransferPower = 0;
        }
    }

    public void jos() {
        servo.setPosition(jos);
    }

    public void sus() {
        servo.setPosition(sus);
    }
    public double a_transfer() {
        return motor_transfer.getCurrent(CurrentUnit.AMPS);
    }

    public double a_intake() {
        return motor_intake.getCurrent(CurrentUnit.AMPS);
    }
//    public boolean bile() {
//        if(senzor_mij.getDistance(DistanceUnit.CM) < 2.65 && !senzor_intake.getState()) {
//            return true;
//        }
//
//        return false;
//    }

    public double s_mij() {
        return senzor_mij.getDistance(DistanceUnit.CM);
    }

    public boolean s_intake() {
        return senzor_intake.getState();
    }

    public double get_ptransfer() {return motor_transfer.getPower();}
    public double get_pintake() {return motor_intake.getPower();}
    public boolean sensor() {return senzor_intake.getState();}
}
