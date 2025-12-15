package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.nominalvoltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.voltage;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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
    DcMotorEx motordr, motorst, motor_intake, motor_transfer;
    Servo s, s1;
    public static double targe, p, ks, kv, ka, velocity, pow, manual_pow;
    public static boolean manual = false;
    PIDFController controller = new PIDFController(p, 0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    IntakeModule intake;
    @Override
    public void runOpMode() throws InterruptedException {
        motordr = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorst = hardwareMap.get(DcMotorEx.class, "motorST");
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        s = hardwareMap.get(Servo.class, "servo_rampa");
        s1 = hardwareMap.get(Servo.class, "servo_blocaj");


        motorst.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);


        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            motor_transfer.setPower(pow);
            motor_intake.setPower(pow);

            velocity = motordr.getVelocity();

            update();
            panelsTelemetry.addData("Velocity", velocity);
            panelsTelemetry.addData("Target", targe);
            panelsTelemetry.addData("Voltage", voltage);
            panelsTelemetry.update(telemetry);

            if(manual == true) {
                motordr.setPower(manual_pow);
                motorst.setPower(manual_pow);
            }
        }

    }

    public void update() {
        controller.setPIDF(p, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        double PID_output = controller.calculate(velocity, targe);
        double ff_output = feedforward.calculate(targe);

        double output = PID_output + ff_output;

        if(targe != 0) {
            motordr.setPower(output * (nominalvoltage / voltage));
            motorst.setPower(output * (nominalvoltage / voltage));
        }

        else {
            motordr.setPower(0);
            motorst.setPower(0);
        }
    }
}





