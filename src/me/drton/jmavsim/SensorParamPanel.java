package me.drton.jmavsim;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

/**
 * @file SensorParamPanel.java
 * Sensor Control Parameter Panel
 *
 * This panel is used for the sensor test and analysis
 *
 * @author SungTae Moon <munhoney@gmail.com>
 */

public class SensorParamPanel extends  JPanel {
    private JSpinner accelSpinner;
    private JSpinner gyroSpinner;
    private JPanel mainPanel;
    private JSpinner magSpinner;
    private JSpinner presSpinner;
    private JSpinner gpsSpinner;
    private JSpinner massSpinner;

    protected Sensors sensors = null;

    public SensorParamPanel() {

        this.add(mainPanel);

        accelSpinner.setModel(new SpinnerNumberModel(0.0f, 0.0f, 1.0f, 0.01f));
        gyroSpinner.setModel(new SpinnerNumberModel(0.0f, 0.0f, 1.0f, 0.01f));
        gpsSpinner.setModel(new SpinnerNumberModel(0.0f, 0.0f, 100.0f, 1.0f));
        magSpinner.setModel(new SpinnerNumberModel(0.0f, 0.0f, 1.0f, 0.001f));
        presSpinner.setModel(new SpinnerNumberModel(0.0f, 0.0f, 1.0f, 0.01f));
        massSpinner.setModel(new SpinnerNumberModel(0.0f, 0.0f, 5.0f, 0.1f));

        accelSpinner.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                Double value = (Double)accelSpinner.getValue();
                sensors.setParameter("noise_Acc", value.floatValue());
            }
        });

        gyroSpinner.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                Double value = (Double)gyroSpinner.getValue();
                sensors.setParameter("noise_Gyo", value.floatValue());
            }
        });

        magSpinner.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                Double value = (Double)magSpinner.getValue();
                sensors.setParameter("noise_Mag", value.floatValue());
            }
        });

        presSpinner.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                Double value = (Double)presSpinner.getValue();
                sensors.setParameter("noise_Prs", value.floatValue());
            }
        });

        gpsSpinner.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                Double value = (Double)gpsSpinner.getValue();
                sensors.setParameter("gpsNoiseStdDev", value.floatValue());
            }
        });

        massSpinner.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                Double value = (Double)massSpinner.getValue();
                sensors.setParameter("mass", value.floatValue());
            }
        });

    }

    public JPanel panel() {
        return mainPanel;
    }

    public void setSensor(Sensors sensors) {
        this.sensors = sensors;

        // init value
        accelSpinner.setValue(new Double(sensors.param("noise_Acc")));
        gyroSpinner.setValue(new Double(sensors.param("noise_Gyo")));
        magSpinner.setValue(new Double(sensors.param("noise_Mag")));
        presSpinner.setValue(new Double(sensors.param("noise_Prs")));
        gpsSpinner.setValue(new Double(sensors.param("gpsNoiseStdDev")));
        massSpinner.setValue(new Double(sensors.param("mass")));

    }

}
