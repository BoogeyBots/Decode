package org.firstinspires.ftc.teamcode.Meeturi.Auto;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.error;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;

import android.graphics.Point;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto1 extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    static TelemetryManager panelsTelemetry;
    boolean turretActivated = false;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double x_startPose = 89.94, y_startPose = 133.77, heading_startPose = 270;
    public static double x_preload = 89.94, y_preload = 83, heading_preload = 270;
    public static double x_collect1 = 128, y_collect1 = 83, heading_collect1 = 180;
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose));
    private final Pose scorePose = new Pose(x_preload, y_preload, Math.toRadians(heading_preload));
    private final Pose collect1 = new Pose(x_collect1, y_collect1, Math.toRadians(heading_collect1));

    private Path scorePreload;
    private PathChain move1;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(heading_startPose));

        move1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, collect1))
                .setConstantHeadingInterpolation(Math.toRadians(heading_collect1))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(100);

                break;

            case 1:
                if(target_velocity < velocity - 20 && error < 3) {
                    outtake.aproape();
                }

                break;


            case 2:
                if(follower.atPose(scorePose, 1, 1)) {

                }

                break;

            case 3:
                if(pathTimer.getElapsedTime() > 5) {
                    intake.scuipa(1);
                }

                if(pathTimer.getElapsedTime() > 8) {
                    intake.trage(1);
                }

                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);

        intake.init_auto();
        outtake.init_auto_aproape();
        turret.init_auto();

        buildPaths();
        follower.setStartingPose(startPose);


        //follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update();
        autonomousPathUpdate();

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getPose().getHeading());
        turret.update_auto(x, y, h);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
        telemetry.addData("error", turret.getErrore());
        telemetry.addData("power", turret.getPower());
        telemetry.addData("kp", turret.getkP());
        telemetry.addData("Grade", turret.gra());
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void init_loop() {
        //turret.update_auto(Math.toDegrees(follower.getPose().getHeading()));
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
