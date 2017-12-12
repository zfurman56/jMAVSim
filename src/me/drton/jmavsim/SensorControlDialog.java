package me.drton.jmavsim;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;


import java.awt.*;

public class SensorControlDialog  extends JDialog implements ActionListener {

    protected Sensors sensors = null;


    private JTextField gpsNoiseText = new JTextField(10);
    private JButton gpsNoiseButton = new JButton("GPSNoise");


    private JTextField gyroNoiseText = new JTextField(10);
    private JButton gyroNoiseButton = new JButton("GyroNoise");


    public SensorControlDialog(JFrame frame) {
        super(frame, "Sensor Control Dialog");

        setLayout(new GridLayout(3,2));

        Panel gpsPanel = new Panel();
        gpsPanel.setLayout(new FlowLayout());
        gpsPanel.add(new JLabel("GPSNoise"));
        gpsPanel.add(gpsNoiseText);
        gpsPanel.add(gpsNoiseButton);

        Panel gyroPanel = new Panel();
        gyroPanel.setLayout(new FlowLayout());
        gyroPanel.add(new JLabel("GyroNoise"));
        gyroPanel.add(gyroNoiseText);
        gyroPanel.add(gyroNoiseButton);


        add(gpsPanel);
        add(gyroPanel);

        this.gpsNoiseButton.addActionListener(this);

    }

    public void setSensor(Sensors sensors) {
        this.sensors = sensors;
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        String str = e.getActionCommand();

        if ( str == "GPSNoise") {
            System.out.println("AAA");
            sensors.setParameter("GPSNoise", 0.001f);
        }
    }





}
