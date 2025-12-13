package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;

@Configurable
@TeleOp
public class Outtake_test extends LinearOpMode {
    DcMotorEx motor_sus, motor_jos, motor_intake, motor_transfer;
    Servo s, s1;
    public static double targe, p, i, d, f, velocity, pow;
    PIDFController controller = new PIDFController(p, i, d, f);
    IntakeModule intake;
    @Override
    public void runOpMode() throws InterruptedException {
        motor_sus = hardwareMap.get(DcMotorEx.class, "motorDR");
        motor_jos = hardwareMap.get(DcMotorEx.class, "motorST");
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        s = hardwareMap.get(Servo.class, "servo_rampa");
        s1 = hardwareMap.get(Servo.class, "servo_blocaj");


        motor_jos.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);


        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            update();
            panelsTelemetry.addData("Velocity", velocity);
            panelsTelemetry.addData("Target", targe);
            panelsTelemetry.update(telemetry);
        }

    }

    public void update() {
        motor_transfer.setPower(pow);
        motor_intake.setPower(pow);
        controller.setPIDF(p, i, d, f);
        velocity = motor_sus.getVelocity();
        double output = controller.calculate(velocity, targe);

        motor_sus.setPower(output);
        motor_jos.setPower(output);
    }
}





