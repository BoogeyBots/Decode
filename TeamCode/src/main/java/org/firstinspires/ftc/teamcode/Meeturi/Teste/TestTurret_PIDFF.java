package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.TICKS_PER_DEGREE;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Configurable
public class TestTurret_PIDFF extends LinearOpMode {
    public static double ks, kv, kp, kd, targett;
    CRServo servo_right, servo_left;
    PIDController controller = new PIDController(kp, 0, kd);
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks, kv);
    DcMotorEx encoder;
    ElapsedTime timer, timer_intake, loop;

    @Override
    public void runOpMode() throws InterruptedException {
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        encoder = hardwareMap.get(DcMotorEx.class, "motor_intake");

        encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        servo_right.setDirection(DcMotorSimple.Direction.REVERSE);
        servo_left.setDirection(DcMotorSimple.Direction.REVERSE);

        loop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        controller.reset();

        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            loop.reset();
            controller.setPID(kp, 0, kd);
            ff = new SimpleMotorFeedforward(ks, kv);

            double turretCurrentPos = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

            double PID_output = controller.calculate(turretCurrentPos, targett);
            double ff_output = ff.calculate(targett);

            servo_left.setPower(PID_output + ff_output);
            servo_right.setPower(PID_output + ff_output);

            panelsTelemetry.addData("Error", targett - turretCurrentPos);
            panelsTelemetry.addData("Power PID", PID_output);
            panelsTelemetry.addData("Power FF", ff_output);
            panelsTelemetry.addData("Loop", loop.milliseconds());
            panelsTelemetry.update(telemetry);

        }

    }
}
