package org.firstinspires.ftc.teamcode.Meeturi.Auto;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.activated;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.error;

import com.bylazar.configurables.annotations.Configurable;
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
public class Auto_red extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    static TelemetryManager panelsTelemetry;
    boolean turretActivated = false;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 5; //în secunde
    public static double x_startPose = 89.94, y_startPose = 133.77, heading_startPose = 270;
    public static double x_preload = 89.94, y_preload = 83, heading_preload = 270;
    public static double x_collect1 = 131, y_collect1 = 83, heading_collect1 = 180;
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose));
    private final Pose scorePose = new Pose(x_preload, y_preload, Math.toRadians(heading_preload));
    private final Pose collect1 = new Pose(x_collect1, y_collect1, Math.toRadians(heading_collect1));

    private Path scorePreload;
    private PathChain rand1, trage1;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(heading_startPose));

        rand1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, collect1))
                .setConstantHeadingInterpolation(Math.toRadians(heading_collect1))
                .build();

        trage1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(heading_collect1), Math.toRadians(heading_startPose)) //probabil Linear, nush, vedem
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);

                break;

            case 1:
                if(follower.atPose(scorePose, 2, 2)) {
                    activated = true;
                    setPathState(2);
                }

                break;

            case 2:
                if(target_velocity <= velocity - 20 && error <= 3 && activated) {
                    outtake.deblocat();
                    setPathState(3);
                }

                break;

            case 3:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(4);
                }

                break;

            case 4:
                follower.followPath(rand1, true);
                intake.trage(1);
                setPathState(5);

                break;

            case 5:
                if(follower.atPose(collect1, 1, 1)) {
                    follower.followPath(trage1);
                    intake.trage(0.5); //să nu forțeze blocajul, sper
                    setPathState(6);
                }

                break;

            case 6:
                if(follower.atPose(scorePose, 1,1)) {
                    activated = true;
                    setPathState(7);
                }

                break;

            case 7:
                if(target_velocity <= velocity - 20 && error <= 3 && activated) {
                    outtake.deblocat();
                    setPathState(8);
                }

                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    y_collect1 -= 24;
                    buildPaths();
                    if(y_collect1 == 9) {
                        setPathState(1000);
                    }
                    else {
                        setPathState(4);
                    }
                }

                break;

                // teoretic de aici avem loop, cu pozitii decalate

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
        outtake.update();

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
        target_velocity = 0;
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

    public void numaitrag() {
        outtake.blocat();
        target_velocity = 0;
        activated = false;
    }
}
