package org.firstinspires.ftc.teamcode.Meeturi;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;

public class Robot {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;

    public Robot(HardwareMap hardwareMap) {
        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);
    }

    void controlIntake() {
        switch(intake.state) {
            case TAKE:
                intake.actions.take();
                break;
            
            case EJECT:
                intake.actions.eject();
                break;

            case OFF:
                intake.actions.off();
                break;
        }
    }

    void controlOuttake() {
        switch(outtake.state) {
            case SHOOT:
                outtake.actions.shoot();
                break;

            case OFF:
                outtake.actions.off();
                break;
        }
    }

    void controlTurret() {
        switch(turret.state) {
            case CONTROLLED:
                break;

            case FOLLOWING:
                break;
        }
        turret.actions.update();
    }

    public void controlAll() {
        controlIntake();
        controlOuttake();
        controlTurret();
    }
}
