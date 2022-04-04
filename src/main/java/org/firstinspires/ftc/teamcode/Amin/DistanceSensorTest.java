package org.firstinspires.ftc.teamcode.Amin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor sensorRange;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        waitForStart();
        while (opModeIsActive()) {
            if (sensorRange.getDistance(DistanceUnit.CM) <= 11.5) {
                telemetry.addData("", "Hai frt ca am vz ceva totusi");
                telemetry.update();
            }
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));

            telemetry.update();

        }

    }
}
