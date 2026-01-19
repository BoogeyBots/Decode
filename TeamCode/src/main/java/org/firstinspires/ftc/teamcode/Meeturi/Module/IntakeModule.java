package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public void init_teleOP() {
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        senzor_intake = hardwareMap.get(DigitalChannel.class, "senzor_intake");
        senzor_mij = hardwareMap.get(DistanceSensor.class, "senzor_mij");

        senzor_intake.setMode(DigitalChannel.Mode.INPUT);

        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hardwareMap.get(Servo.class, "servo_intake");
        sus();
    }

    public void init_auto() {
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        senzor_intake = hardwareMap.get(DigitalChannel.class, "senzor_intake");
        senzor_mij = hardwareMap.get(DistanceSensor.class, "senzor_mij");
        servo = hardwareMap.get(Servo.class, "servo_intake");

        senzor_intake.setMode(DigitalChannel.Mode.INPUT);

    }

    public void trage_intake(double power) {
        motor_intake.setPower(power);
    }

    public void scuipa_intake(double power) {
        motor_intake.setPower(-power);
    }

    public void stop_intake() {
        motor_intake.setPower(0);
    }

    public void trage_transfer(double power) {
        motor_transfer.setPower(power);
    }

    public void scuipa_transfer(double power) {
        motor_transfer.setPower(-power);
    }

    public void stop_transfer() {
        motor_transfer.setPower(0);
    }

    public void jos() {
        servo.setPosition(jos);
    }

    public void sus() {
        servo.setPosition(sus);
    }
    public boolean bile() {
        if(senzor_mij.getDistance(DistanceUnit.CM) < 2.65 && !senzor_intake.getState()) {
            return true;
        }

        return false;
    }
}