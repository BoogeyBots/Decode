package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@TeleOp
public class DigitalSensor_test extends LinearOpMode {
    DigitalChannel sensor;
    boolean bool;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        sensor.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {
            bool = sensor.getState();

            telemetry.addData("Status", bool);
            telemetry.update();
        }

    }
}
