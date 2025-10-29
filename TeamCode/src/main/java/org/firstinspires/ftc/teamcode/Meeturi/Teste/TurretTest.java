package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;

@TeleOp
public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TurretModule turret = new TurretModule(hardwareMap);
        turret.init();

        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            panelsTelemetry.addData("Pinpoint ", turret.getHeading());
            panelsTelemetry.addData("Ticks", turret.getHeading() * 121.3629);
            panelsTelemetry.addData("Error", turret.getErrore());
            panelsTelemetry.addData("Power", turret.getPower());
            panelsTelemetry.addData("kP", turret.getkP());
            turret.update();
            panelsTelemetry.update(telemetry);
        }

    }
}
