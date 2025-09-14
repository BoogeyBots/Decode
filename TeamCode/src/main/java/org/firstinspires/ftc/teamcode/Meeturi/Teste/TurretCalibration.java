package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;

import java.util.List;

@TeleOp
public class TurretCalibration extends LinearOpMode {
    public static double kp = 0.5, deadband = 0.01;
    TurretModule turret = new TurretModule(hardwareMap);
    double heading, error, power, tkp;
    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            read();

            turret.calibrare(kp, deadband);
            turret.update();

        }

        telemetry.addData("Power", power);
        telemetry.addData("Heading", heading);
        telemetry.addData("Error", error);
        telemetry.addData("tkp", tkp);
        telemetry.update();
    }

    public void read() {
        heading = turret.getHeading();
        error = turret.getError();
        power = turret.getPower();
        tkp = turret.getkP();
    }
}
