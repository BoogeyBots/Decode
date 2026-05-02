package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
@Configurable
public class Outtake extends LinearOpMode {
    DcMotorEx motor;
    public static double ks, kv, ka = 0.0005, kp;
    public static double target, manual_pow;
    public static boolean manual = false;
    double voltage, velocity, nominalVoltage = 9.3;
    PIDController controller = new PIDController(kp, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();

        while (opModeIsActive()) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            velocity = motor.getVelocity();
            //ks
            //kv - powerManual / velocity  -20 +20    0.3, 0.5, 0.8
            if(manual_pow > 0.01) {
                motor.setPower(manual_pow);
            }

            else {
                controller.setPID(kp, 0, 0);
                feedforward = new SimpleMotorFeedforward(ks, kv, ka);

                double PID_output = controller.calculate(velocity, target);
                double ff_output = feedforward.calculate(target);

                if(target != 0) {
                    motor.setPower((PID_output + ff_output) * (nominalVoltage / voltage));
                }
                else motor.setPower(0);

            }

            telemetry.addData("Velocity", velocity);
        }
    }
}
