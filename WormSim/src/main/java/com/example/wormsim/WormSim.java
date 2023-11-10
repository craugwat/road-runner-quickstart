package com.example.wormsim;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import java.awt.Button;
import java.awt.Canvas;
import java.awt.Color;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.Window;
import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.imageio.ImageIO;

public class WormSim {
    SimMecanumDrive drive = new SimMecanumDrive( new Pose2d(0, 0, 0));
    TrajectoryActionBuilder trajActBld = drive.actionBuilder(drive.pose);
//
//    TrajSplineTest trajSpineTest = new TrajSplineTest();

    // extending Frame class to our class AWTExample1
    // CAW - taken from https://www.javatpoint.com/java-awt
    public static class AWTExample1 extends Frame {

        // initializing using constructor
        AWTExample1() {

            // creating a button
            Button b = new Button("Click Me!!");

            // setting button position on screen
            b.setBounds(30,100,80,30);

            // adding button into frame
            add(b);
            add(new MyCanvas());

            // frame size 300 width and 300 height
            setSize(800,800);

            // setting the title of Frame
            setTitle("This is our basic AWT example");

            // no layout manager
            setLayout(null);

            // now frame will be visible, by default it is not visible
            setVisible(true);
        }
    }


        // main method
        public static void main(String args[]) {

// creating instance of Frame class
            AWTExample1 f = new AWTExample1();

        }


}

// class which inherits the Canvas class
// to create Canvas
class MyCanvas extends Canvas
{
    // class constructor
    public MyCanvas() {
        setBackground (Color.GRAY);

        setSize(800, 800);
    }

    // paint() method to draw inside the canvas
    public void paint(Graphics g)
    {

        // adding specifications
        g.setColor(Color.red);
        g.fillOval(75, 75, 150, 75);

        BufferedImage image = null;
        try {
            image = ImageIO.read(getClass().getClassLoader().getResource("background/season-2023-centerstage/field-2023-official.png"));
        }
        catch (IOException e) {
            e.printStackTrace();
        }

        g.drawImage(image,0,0,800,800,this);
        //imageUpdate(image,0,0, 0, 800, 800);
    }
}

