package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

public class Components {
    public static DcMotor leftDriveA = null;
    public static DcMotor leftDriveB = null;
    public static DcMotor rightDriveA = null;
    public static DcMotor rightDriveB = null;
    public static DcMotorEx shooterA = null;
    public static DcMotorEx shooterB = null;
    public static DcMotor collector = null;
    public static DcMotor contServoArm = null;
    public static Servo servoArm = null;
    public static ColorSensor groundSensor = null;
    public static ColorSensor upperSensor = null;
    public static ColorSensor lowerSensor = null;
}
