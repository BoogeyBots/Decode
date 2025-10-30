package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
@TeleOp
public class OuttakeTest extends LinearOpMode {
    public static double power, pos;
    @Override
    public void runOpMode() throws InterruptedException {
        OuttakeModule outtake = new OuttakeModule(hardwareMap);
        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        outtake.init_teleOP();

        waitForStart();

        while (opModeIsActive()) {
            outtake.update();
            outtake.servo_custom();

            panelsTelemetry.addData("Velocity 1", outtake.get_velocity_sus());
            panelsTelemetry.addData("Target", outtake.get_target());
            panelsTelemetry.update(telemetry);
        }
    }
}