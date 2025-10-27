package org.firstinspires.ftc.teamcode.Meeturi.Teste;

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

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pinpoint ", turret.getHeading());
            telemetry.addData("Error ", turret.getError());
            turret.update();
            telemetry.update();
        }

    }
}
