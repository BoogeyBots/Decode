package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Configurable
@TeleOp
public class Outtake_test extends LinearOpMode {
    DcMotorEx motor_sus, motor_jos, motor_opus;
    public static double targe, p, i, d, f, velocity, v, s, a;
    PIDFController controller = new PIDFController(p, i, d, f);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(s, v, a);
    @Override
    public void runOpMode() throws InterruptedException {
        motor_sus = hardwareMap.get(DcMotorEx.class, "motor_sus");
        motor_jos = hardwareMap.get(DcMotorEx.class, "motor_jos");
        motor_opus = hardwareMap.get(DcMotorEx.class, "motor_opus");

        motor_sus.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_opus.setDirection(DcMotorSimple.Direction.REVERSE);

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
        controller.setPIDF(p, i, d, f); //kp=4
        feedforward = new SimpleMotorFeedforward(s, v, a);
        velocity = motor_sus.getVelocity();
        double PID_output = controller.calculate(velocity, targe); //-2100
        double FF_output = feedforward.calculate(targe);
        double output = PID_output + FF_output;

        motor_sus.setPower(output);
        motor_jos.setPower(output);
        motor_opus.setPower(output);
    }
}





