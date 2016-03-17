// (c) Copyright Microsoft Corporation.
// This source is subject to the Microsoft Public License (Ms-PL).
// Please see http://go.microsoft.com/fwlink/?LinkID=131993 for details.
// All other rights reserved.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using Coding4Fun.Kinect.Wpf;

// AsebaCMD Libraries
using System.Diagnostics;
using System.Threading;


namespace SkeletalTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Variables :
        // General Kinect Variables
        bool closing = false;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];

        // Joystick Variables
        //{640, 480} Color Camara frame Dimensions        
        double actHeight = 0.1;// Height [m] above the head that the left hand has to overcome to activate initialPose capture
        bool getInitPos = true;// 'true': Determines the initial pose should be captured

        // Imaginary square(along the X-Z plane) centered at 'initPos', among which
        // the displacement distance of the right hand would be computed. Additionally movement 
        // in the X,Z inside this square dont have any effect in the robots position & movement
        // outside the Y coords of the cube represent an un-active user
        double[] centerCube = new double[3] {0.0,0.0,0.0}; // in [m]
        SkeletonPoint initPos;

        // Displacements of hand w.r.t. the initialPose & variable indicating if user is activelly "wanting" to move the robot
        double distX;
        double distZ;
        double velFact = 2400; // factor by which the magnitud of the vector is *, in order to scale it to Adequate velocity values of the robot
        bool isActive; 

        // Asebacmd variables
        string cd = @"C:\Users\IVAN\Documents\MEGAsync\Swiss Thesis\CoreRobot\ASEBA\ASEBA External Interfaces\ASEBAExt Code\asebaMYbuild\asebaEXE";
        //"C:\Program Files (x86)\AsebaStudio";// @ must be added to comply with the Unicode characters
        bool startCMD = true;
        Process cmd = new Process(); // CMD Process object   

        // Epuck variables
        double ang = 0;
        double hyp = 0;
        int eventNo = 0; // Event number to emit in ASEBA
        int timerCommand = 0; // "timer" which avoids over-sending control commands to the robot
        double pastAng = 0; // sotres the last angle sent to the robot

        public MainWindow()
        {
            InitializeComponent();
        }
                            
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);

        }

        // KINECT INITIALIZATION AND ENABLING
        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            KinectSensor old = (KinectSensor)e.OldValue;

            StopKinect(old);

            KinectSensor sensor = (KinectSensor)e.NewValue;

            if (sensor == null)
            {
                return;
            }
            
            // INITIALIZE KINECT W/'PARAMETERS' TO APPLY SMOOTHING TO THE SKELETONG TRACKING MOVEMENT

            //var parameters = new TransformSmoothParameters
            //{
            //    Smoothing = 0.3f,
            //    Correction = 0.0f,
            //    Prediction = 0.0f,
            //    JitterRadius = 1.0f,
            //    MaxDeviationRadius = 0.5f
            //};
            //sensor.SkeletonStream.Enable(parameters);

            sensor.SkeletonStream.Enable();

            sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(sensor_AllFramesReady);
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

            try
            {
                sensor.Start();
            }
            catch (System.IO.IOException)
            {
                kinectSensorChooser1.AppConflictOccurred();
            }
        }

        //METHOD ("MAIN") CONSTANTLY EXECUTED (AFTER EACH FRAME) THAT CAPTURES SKELETON DATA
        // AND POSITION TAGS IN THE JOINTS POSITIONS
        void sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            if (closing)// if the application is closing, simply return and get NO skeleton
            {
                return;
            }

            //Console.WriteLine(0);

            //Get a skeleton (THE FIRST)
            Skeleton first = GetFirstSkeleton(e);

            if (first == null)
            {
                return;
            }

            // Obtain (if any) hands displacement and send it to robots controller
            getHandDisplacement(first, e);       

        }

        // FROM ALL THE RETURNED SKELETON DATA (BY DEFAULT 6) CAPTURE THE FIRST
        // CORRECTLY TRACKED SKELETON
        Skeleton GetFirstSkeleton(AllFramesReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrameData = e.OpenSkeletonFrame())
            {
                if (skeletonFrameData == null)
                {
                    return null;
                }


                skeletonFrameData.CopySkeletonDataTo(allSkeletons);

                //get the first tracked skeleton
                Skeleton first = (from s in allSkeletons
                                  where s.TrackingState == SkeletonTrackingState.Tracked
                                  select s).FirstOrDefault();

                return first;

            }
        }

        // CAPTURE JOINT COORDS AND TRANSFORM THEM INTO A COLOR IMAGE COORDS
        // IN ORDER TO POSITION TAGS @ THOSE COORDS.
        void getHandDisplacement(Skeleton first, AllFramesReadyEventArgs e)
        {
            // Get Current Joint Pose
            SkeletonPoint rightHPos = first.Joints[JointType.HandRight].Position;
            SkeletonPoint leftHPos = first.Joints[JointType.HandLeft].Position;
            SkeletonPoint headPos = first.Joints[JointType.Head].Position;
       
            // Capture Initial Virtual Joystick Position

            if ((leftHPos.Y > headPos.Y + actHeight))
            {
                if (getInitPos)// indicated algo. must capture initial right hand position
                {
                    getInitPos = false;
                    initPos = rightHPos;
                    initCMD();// Establish CMD Process initial connection  
                    Console.WriteLine("START CONNECTION");
                }

                else // if algo is active (getInitPos == false) and user raises left hand --> Deactivate Robot Control
                {
                    // reset control boolean variables
                    getInitPos = true;
                    startCMD = true;

                    // Restart Robots position
                    ang = 0;
                    hyp = 0;
                    cmd.StandardInput.WriteLine("asebacmd usermsg " + eventNo + " " + ang + " " + hyp);
                    cmd.StandardInput.Flush();
                    Console.WriteLine("\nAngle [deg]: " + ang + "   Speed : " + hyp);
                    Console.WriteLine("... DISCONNECTING 1 ...");
                    // Terminate CMD Process
                    cmd.StandardInput.Close();
                    Console.WriteLine("... DISCONNECTING 2 ...");
                    cmd.WaitForExit();

                    Console.WriteLine("DELETE CONNECTION");
                }

                Thread.Sleep(2000);
           }

            // Compute displacement distances            
            if (!getInitPos)
            {

                if (!startCMD)// If robots commands are ready to send (i.e. Asebacmd started)
                {
                    // Compute Joystick Displacement
                    initPos = leftHPos; // Instead of using the captures rightHand initialPos, use currentPos of leftHand as the center (pivot)
                    distX = ((Math.Abs(rightHPos.X - initPos.X) > centerCube[0]) ? 1 : 0) * (rightHPos.X - initPos.X);
                    distZ = ((Math.Abs(rightHPos.Z - initPos.Z) > centerCube[1]) ? 1 : 0) * -(rightHPos.Z - initPos.Z);
                    isActive = true;// (Math.Abs(rightHPos.Y - initPos.Y) < centerCube[2]);

                    // Compute robot control variables
                    ang = 0;
                    if (distX + distZ != 0) // Validation which prevents that Atan2(0,0) = 180, which isnt adecuate for the application
                    {
                        ang = Convert.ToInt32((180 / Math.PI) * Math.Atan2(distZ, distX));// Compute angle and convert from Rad => Deg
                        ang = ((ang > 0) ? 1 : 0) * ang + ((ang < 0) ? 1 : 0) * (360 + ang); // transform from [-180,180] to [0,360]   
                        //Console.WriteLine(ang);
                    }

                    pastAng = ang * (isActive ? 1 : 0) + pastAng * (!isActive ? 1 : 0); // captura pastAngle if user is active, else consert last pastAngle
                    if (!isActive)// Unactive User => Conserve same angle and stop robot (i.e. speed == 0)
                    {
                        ang = pastAng;
                        distX = 0;
                        distZ = 0;
                        Console.WriteLine("INACTIVE USER");
                    }                  

                    // The robots speed is proportional to the length of the hypothenusa described by the user in the ZX plane 
                    // 1000 is simply a  scaling factor
                    hyp = Convert.ToInt32(velFact * Math.Sqrt(Math.Pow(distX, 2) + Math.Pow(distZ, 2))); 

                    // Send CMD robot command to ASEBA

                    // Used to not over-send commands to the robot, since between command and command the kinect program
                    // must wait enough time to ensure that the E-puck has reached the target Angle. This is required
                    // because the E-puck's actual angle is merelly being estimated by the time it has spent rotating.
                    // This was done since encoder odometry which is a bit better isn't easely done through ASEBA --> this 
                    // framework hasproblems with the E-pucks encoder values and only uses 16-bit integers. Preventing the
                    // programmer from using floats, and even if one could scale all values by a factor (to avoid floats), 
                    // do to the limitation that the max.integer value is +- 32000, it isnt that viable.
                    timerCommand += 1;
                    eventNo = 0;
                    if (timerCommand >= 45)
                    {

                        timerCommand = 0;
                        cmd.StandardInput.WriteLine("asebacmd usermsg " + eventNo + " " + ang + " " + hyp);
                        cmd.StandardInput.Flush();
                        //Console.WriteLine("\n\n COMMAND " + distX + "   " + distZ);
                        //Console.WriteLine("asebacmd usermsg " + eventNo + " " + ang + " " + hyp);
                        Console.WriteLine("\nAngle [deg]: " + ang + "   Speed : " + hyp);
                    }

                    // ********** DEBUGING **********
                    //Console.WriteLine("\nCOORDS:");
                    //Console.WriteLine(distX);
                    //Console.WriteLine(distZ);

                    //Console.WriteLine("\n\n EPUCK");
                    //Console.WriteLine(ang);
                    //Console.WriteLine(hyp);

                    //Console.WriteLine("\n\n COMMAND");
                    //Console.WriteLine("usrmsg " + eventNo + " " + ang + " " + hyp);
                }                                
            }

        }

        private void initCMD()
        {
            cmd.StartInfo.FileName = "cmd.exe";
            cmd.StartInfo.RedirectStandardInput = true;
            cmd.StartInfo.RedirectStandardOutput = true;
            cmd.StartInfo.CreateNoWindow = true;
            cmd.StartInfo.UseShellExecute = false;
            cmd.Start();

            // Stablish Current Directory
            startCMD = false;
            cmd.StandardInput.WriteLine("cd " + cd);
            cmd.StandardInput.Flush();

            // ********** DEBUGING **********
            //cmd.StartInfo.CreateNoWindow = false;
            //cmd.StandardInput.Close();
            //cmd.WaitForExit();
            //Console.WriteLine(cmd.StandardOutput.ReadToEnd()); 
        }

        private void StopKinect(KinectSensor sensor)
        {

            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    //stop sensor 
                    sensor.Stop();

                    //stop audio if not null
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }


                }
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            closing = true;
            StopKinect(kinectSensorChooser1.Kinect);

            // Terminate CMD Process
            cmd.StandardInput.Close();
            cmd.WaitForExit();
        }

        private void kinectSkeletonViewer1_Loaded(object sender, RoutedEventArgs e)
        {

        }
        
    }
}
