package org.firstinspires.ftc.teamcode.Amin.AutonoameNatio.Blue;

import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_ARUNCA_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_NORMAL_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.initialBLUEDR;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub_simple_part1BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub_simple_part2BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_rata_vectorBLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.park_mai_putin_csfBLUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Amin.LocalizareFrt;
import org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.Detectie;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(group = "blue")
public class BlueDrStoragePark1Cub extends LinearOpMode {
    SampleMecanumDrive robot;
    OpenCvCamera camera;
    private ElapsedTime timp = new ElapsedTime();
    private final long sleep = 650;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        robot = new SampleMecanumDrive(hardwareMap);

        int cameraViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraViewId);

        Detectie detectie = new Detectie();

        camera.setPipeline(detectie);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        robot.setPoseEstimate(initialBLUEDR);

        waitForStart();
        while (opModeIsActive()) {
            switch (detectie.getA()) {
                case 1:
                    mijloc();
                    break;
                case -1:
                    jos();
                    break;
                default:
                    sus();
                    break;
            }
        }
        LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();

    }

    private void sus() throws InterruptedException {
        Trajectory la_rate_dute = robot.trajectoryBuilder(initialBLUEDR)
//                .lineToConstantHeading(la_rata_vectorBLUE)
                .lineToLinearHeading(la_rata_vectorBLUE)
                .build();
        robot.followTrajectory(la_rate_dute);
        robot.invarteRataAlbastru();

        TrajectorySequence la_hub_traj = robot.trajectorySequenceBuilder(la_rate_dute.end())
                .lineToConstantHeading(la_hub_simple_part1BLUE)
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.9, () -> {
                    robot.brat.setPower(0);
                })
                .lineToLinearHeading(la_hub_simple_part2BLUE)
                .build();

        robot.followTrajectorySequence(la_hub_traj);

        robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

        Trajectory park_traj = robot.trajectoryBuilder(la_hub_traj.end(), true)
                .lineToLinearHeading(park_mai_putin_csfBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.6, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(park_traj);


        stop();

    }

    private void mijloc() throws InterruptedException {
        Trajectory la_rate_dute = robot.trajectoryBuilder(initialBLUEDR)
//                .lineToConstantHeading(la_rata_vectorBLUE)
                .lineToLinearHeading(la_rata_vectorBLUE)
                .build();
        robot.followTrajectory(la_rate_dute);
        robot.invarteRataAlbastru();

        TrajectorySequence la_hub_traj = robot.trajectorySequenceBuilder(la_rate_dute.end())
                .lineToConstantHeading(la_hub_simple_part1BLUE)
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(1.320, () -> {
                    robot.brat.setPower(0);
                })
                .lineToLinearHeading(la_hub_simple_part2BLUE)
                .build();

        robot.followTrajectorySequence(la_hub_traj);

        robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

        Trajectory park_traj = robot.trajectoryBuilder(la_hub_traj.end(), true)
                .lineToLinearHeading(park_mai_putin_csfBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.4, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(park_traj);


        stop();

    }

    private void jos() throws InterruptedException{
        Trajectory la_rate_dute = robot.trajectoryBuilder(initialBLUEDR)
//                .lineToConstantHeading(la_rata_vectorBLUE)
                .lineToLinearHeading(la_rata_vectorBLUE)
                .build();
        robot.followTrajectory(la_rate_dute);
        robot.invarteRataAlbastru();

        TrajectorySequence la_hub_traj = robot.trajectorySequenceBuilder(la_rate_dute.end())
                .lineToConstantHeading(la_hub_simple_part1BLUE)
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(1.243, () -> {
                    robot.brat.setPower(0);
                })
                .lineToLinearHeading(la_hub_simple_part2BLUE)
                .build();

        robot.followTrajectorySequence(la_hub_traj);

        robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

        Trajectory park_traj = robot.trajectoryBuilder(la_hub_traj.end(), true)
                .lineToLinearHeading(park_mai_putin_csfBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.3, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(park_traj);


        stop();

    }

}
