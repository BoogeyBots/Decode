package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp
public class PIDvsPIDFF extends LinearOpMode {
    public static double kp, ki, kd, ks = 0.12, kv = 0.000553144, ka = 0.005;
    public static double targf, vell;
    public static boolean PIDFF, transfer;
    double vv;
    PIDController controller = new PIDController(kp, ki, kd);
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks, kv, ka);
    DcMotorEx motorST, motorDR, motor_intake, motor_transfer;
    @Override
    public void runOpMode() throws InterruptedException {
        motorDR = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");

        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");

        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        motorST.setDirection(DcMotorSimple.Direction.REVERSE);

        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            vv  = hardwareMap.voltageSensor.iterator().next().getVoltage();

            update_PIDFF();

            if(transfer) {
                motor_intake.setPower(1);
                motor_transfer.setPower(1);
            }

            else {
                motor_intake.setPower(0);
                motor_transfer.setPower(0);
            }

            panelsTelemetry.addData("Velocity fara PID", vell);
            panelsTelemetry.addData("Velocity cu PID", vell + 300);
            panelsTelemetry.addData("Target", targf);
            panelsTelemetry.update(telemetry);
        }
    }

    public void update_PID() {
        vell = motorDR.getVelocity();
        controller.setPID(kp, ki, kd);
        double PID_output = controller.calculate(vell, targf);

        motorST.setPower(PID_output);
        motorDR.setPower(PID_output);
    }

    public void update_PIDFF() {
        vell = motorDR.getVelocity();

        controller.setPID(0.0087, 0, 0);
        ff =  new SimpleMotorFeedforward(ks, kv, ka);

        double PID_output = controller.calculate(vell, targf);
        double ff_output = ff.calculate(targf);
        double output = PID_output + ff_output;

        motorST.setPower(output * (10.7/vv));
        motorDR.setPower(output * (10.7/vv));
    }

}
