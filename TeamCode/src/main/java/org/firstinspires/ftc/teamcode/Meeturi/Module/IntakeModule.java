package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeModule extends Constants.intake {
    HardwareMap hardwareMap;
    public IntakeModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    DcMotorEx motor_intake, motor_transfer;
   // DistanceSensor senzor_intake;
    Servo servo;
    public void init_teleOP() {
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
       // senzor_intake = hardwareMap.get(DistanceSensor.class, "senzor_intake");

        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hardwareMap.get(Servo.class, "servo_intake");
        sus();
    }

    public void init_auto() {
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
       // senzor_intake = hardwareMap.get(DistanceSensor.class, "senzor_intake");

        servo = hardwareMap.get(Servo.class, "servo_intake");
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

}