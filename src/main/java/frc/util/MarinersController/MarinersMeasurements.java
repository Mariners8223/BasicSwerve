package frc.util.MarinersController;

import java.util.function.Supplier;

public class MarinersMeasurements {
    private Supplier<Double> position;
    private Supplier<Double> velocity;
    private Supplier<Double> acceleration;


    private double currentPosition;
    private double currentVelocity;
    private double currentAcceleration;


    private double dt;


    public void update(double dt){
        this.dt = dt;

        currentPosition = position.get();
        currentVelocity = velocity.get();
        currentAcceleration = acceleration.get();
    }


    public double getPosition(){
        return currentPosition;
    }

    public double getVelocity(){
        return currentVelocity;
    }

    public double getAcceleration(){
        return currentAcceleration;
    }

    public void setPositionSupplier(Supplier<Double> position, double gearRatio){
        this.position = () -> position.get() * gearRatio;
    }

    public void setVelocitySupplier(Supplier<Double> velocity, double gearRatio){
        this.velocity = () -> velocity.get() * gearRatio;
    }

    public void setAccelerationSupplier(Supplier<Double> acceleration, double gearRatio){
        this.acceleration = () -> acceleration.get() * gearRatio;
    }

    public MarinersMeasurements(Supplier<Double> position, Supplier<Double> velocity, Supplier<Double> acceleration, double gearRatio){
        setPositionSupplier(position, gearRatio);
        setVelocitySupplier(velocity, gearRatio);
        setAccelerationSupplier(acceleration, gearRatio);
    }

    public MarinersMeasurements(Supplier<Double> position, double gearRatio){
        setPositionSupplier(position, gearRatio);

        createVelocitySupplier(position);
        createAccelerationSupplier(velocity);
    }

    public MarinersMeasurements(Supplier<Double> position, Supplier<Double> velocity, double gearRatio){
        setPositionSupplier(position, gearRatio);
        setVelocitySupplier(velocity, gearRatio);

        createAccelerationSupplier(velocity);
    }

    private void createVelocitySupplier(Supplier<Double> position){
        double[] lastPosition = {position.get()};

        velocity = () -> {
            double currentPosition = position.get();

            double velocity = (currentPosition - lastPosition[0]) / dt;

            lastPosition[0] = currentPosition;

            return velocity;
        };
    }

    private void createAccelerationSupplier(Supplier<Double> velocity){
        double[] lastVelocity = {velocity.get()};

        acceleration = () -> {
            double currentVelocity = velocity.get();

            double acceleration = (currentVelocity - lastVelocity[0]) / dt;

            lastVelocity[0] = currentVelocity;

            return acceleration;
        };
    }
}
