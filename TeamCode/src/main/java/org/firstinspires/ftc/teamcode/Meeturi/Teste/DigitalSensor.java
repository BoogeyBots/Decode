package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@TeleOp
public class DigitalSensor extends LinearOpMode {
    DigitalChannel sensor1, sensor2;
    boolean bool1, bool2;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor1 = hardwareMap.get(DigitalChannel.class, "sensor_intake");
        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor_blocaj");
        sensor1.setMode(DigitalChannel.Mode.INPUT);
        sensor2.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {
            bool1 = sensor1.getState();
            bool2 = sensor2.getState();

            telemetry.addData("Status_intake", bool1);
            telemetry.addData("Status_blocaj", bool2);
            telemetry.update();
        }

    }
}