package org.firstinspires.ftc.teamcode.Amin;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp()
public class SenzorDeCuloareTest extends LinearOpMode {
    SampleMecanumDrive robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            ColorSensor colorSensor = robot.color;

            if (
                    !(robot.color.red() <= 75)
//                            &&
//                            (robot.color.green() >= 70 && robot.color.red() <= 80)
//                            &&
//                            (robot.color.blue() <= 70 && robot.color.blue() <= 80)
            ) {
                telemetry.addLine("am gasit ceva frt, nush ce, dar am gasit");
            }

            telemetry.addData("Clear", robot.color.alpha());
            telemetry.addData("Red  ", robot.color.red());
            telemetry.addData("Green", robot.color.green());
            telemetry.addData("Blue ", robot.color.blue());

            telemetry.addLine("-------------");

            /**
             * red
             * green
             * blue
             *
             *
             * */

            telemetry.update();
        }
    }
}
