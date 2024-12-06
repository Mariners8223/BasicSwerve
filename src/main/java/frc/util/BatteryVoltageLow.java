package frc.util;

public class BatteryVoltageLow extends RuntimeException {
    public BatteryVoltageLow(String message) {
        super(message);
    }
}
