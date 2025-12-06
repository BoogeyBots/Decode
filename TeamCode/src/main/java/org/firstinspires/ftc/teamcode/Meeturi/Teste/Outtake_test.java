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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;

@Configurable
@TeleOp
public class Outtake_test extends LinearOpMode {
    DcMotorEx motor_sus, motor_jos, motor_opus;
    Servo s, s1;
    public static double targe, p, i, d, f, velocity, pos_rampa = 0.1, pos_blocaj = 0.55, pow;
    PIDFController controller = new PIDFController(p, i, d, f);
    IntakeModule intake;
    @Override
    public void runOpMode() throws InterruptedException {
        motor_sus = hardwareMap.get(DcMotorEx.class, "motor_sus");
        motor_jos = hardwareMap.get(DcMotorEx.class, "motor_jos");
        motor_opus = hardwareMap.get(DcMotorEx.class, "motor_opus");
        s = hardwareMap.get(Servo.class, "servo_rampa");
        s1 = hardwareMap.get(Servo.class, "servo_blocaj");

        intake = new IntakeModule(hardwareMap);

        motor_sus.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_jos.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.init_teleOP();

        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            update();
            intake.trage(pow);
            s.setPosition(pos_rampa);
            s1.setPosition(pos_blocaj);
            panelsTelemetry.addData("Velocity", velocity);
            panelsTelemetry.addData("Target", targe);
            panelsTelemetry.update(telemetry);
        }

    }

    public void update() {
        controller.setPIDF(0.001, 0, 0, 0.0004);
        velocity = motor_opus.getVelocity();
        double output = controller.calculate(velocity, targe);

        motor_sus.setPower(output);
        motor_jos.setPower(output);
        motor_opus.setPower(output);
    }
}





