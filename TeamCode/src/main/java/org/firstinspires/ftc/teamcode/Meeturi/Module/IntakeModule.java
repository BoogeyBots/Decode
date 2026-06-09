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

    public IntakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    DcMotorEx motor_intake, motor_transfer;
    private double lastIntakePower = 0;
    private double lastTransferPower = 0;

    public void init() {
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");

        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void trage_intake(double power) {
        double target = power;
        if (Math.abs(target - lastIntakePower) > 0.005) {
            motor_intake.setPower(target);
            lastIntakePower = target;
        }
    }

    public void scuipa_intake(double power) {
        double target = -power;
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
}
