package org.firstinspires.ftc.teamcode;

public class Variables {

    static final double COUNTS_PER_REVOLUTION_GOBILDa = 384.5;
    static final double COUNTS_PER_DEGREES_GOBILDA = COUNTS_PER_REVOLUTION_GOBILDa / 360;

    static final double COUNTS_PER_REVOLUTION_CORE = 288;
    static final double COUNTS_PER_DEGREE_CORE = (COUNTS_PER_REVOLUTION_CORE) / 360;

    static final double COUNTS_PER_REVOLUTION = 1680;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / 360;

}