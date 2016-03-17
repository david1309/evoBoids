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
        // General Kinect Variables
        bool closing = false;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];

        // VirtualPointer Variables
        bool init = false;// true: start gesture capturing ; false: don't capture users gestures
        double actHeight = 0.1;// Height [m] above the head that the left hand has to overcome to activate initialPose capture
        double stopThr = 0.40; // Y - axis separation distance [m] between hand and shoulder that indicates a Stop command gesture
        double pointThr = 0.0825; // X - axis separation distance [m] between hand and shoulder that indicates a Rotation command gesture

        short command; // command : 0-> Stop , 1 -> Straight motion , 2 -> CW rot. 3, -> CCW rot.
        string commandS; // String version of variable 'command'  
        int commandCount = 0; // counts how many commands have been sent to the robot 8after commanadCount is > Ncounts, the cmd communication with asebaSwtich is reset

        // Asebacmd variables
        string cd = @"C:\Users\IVAN\Documents\MEGAsync\Swiss Thesis\CoreRobot\ASEBA\ASEBA External Interfaces\ASEBAExt Code\asebaMYbuild\asebaEXE";// @ must be added to comply with the Unicode characters
        int eventNo = 0; // Event number to emit in ASEBA
        bool startCMD = true; // true: CMD proccess has to be init ; false: CMD process is already running
        Process cmd = new Process(); // CMD Process object   


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

        //"MAIN" METHOD CONSTANTLY EXECUTED (AFTER EACH FRAME) THAT CAPTURES SKELETON DATA
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
            SkeletonPoint rightSPos = first.Joints[JointType.ShoulderRight].Position;
            SkeletonPoint headPos = first.Joints[JointType.Head].Position;
       
            // Capture Initial Virtual Joystick Position

            if ((leftHPos.Y > headPos.Y + actHeight))
            {
                if (!init)// indicated algo. must start gesture capturing
                {
                    init = true;
                    initCMD();// Establish CMD Process initial connection  
                    Console.WriteLine("START CONNECTION");
                }

                else // if algo is active (init == true) and user raises left hand --> Deactivate Robot Control
                {
                    // reset control boolean variables
                    init = false;
                    startCMD = true;

                    // Force Robot to STOP
                    command = 0;
                    cmd.StandardInput.WriteLine("asebacmd usermsg " + eventNo + " " + command);

                    // Terminate CMD Process
                    cmd.StandardInput.Flush();                   
                    cmd.StandardInput.Close();
                    Console.WriteLine("... DISCONNECTING ...");
                    cmd.WaitForExit();
                    Console.WriteLine("DELETE CONNECTION");
                }

                Thread.Sleep(2000);
           }

            // Captures Gestures and Emit Robot control command            
            if (init && !startCMD )// If robots commands are ready to send (i.e. Asebacmd started)
            {
                // Obtain users command
                if (Math.Abs(rightSPos.Y - rightHPos.Y) > stopThr) // STOP
                {
                    command = 0;
                    commandS = "Stop";
                }

                else if (Math.Abs(rightSPos.X - rightHPos.X) > pointThr) // POINTING ROTATION
                {
                    if ((rightSPos.X - rightHPos.X) < 0) // CW ROTATION
                    {
                        command = 2;
                        commandS = "Rot. CW";
                    }

                    else  // CCW ROTATION
                    {
                        command = 3;
                        commandS = "Rot. CCW";
                    }
                }

                else // STRAIGHT
                {
                    command = 1;
                    commandS = "Straight";
                }

                // Send CMD robot command to ASEBA
                cmd.StandardInput.WriteLine("asebacmd usermsg " + eventNo + " " + command);
                cmd.StandardInput.Flush();
                Console.WriteLine("\nCommand: " + commandS );
                commandCount += 1;

                 //When N - number of commands have been emited, reset connection between Kinect Client and ASEBASwitch. This "trick" has to be done to mitigate the Bug
                 //that ASEBA for WINDOWS has. 
                 //Bug Hypothesis: ASEBASwitch doesnt correctly move data out of its buffer once a stream ( command ) hast been sent to ASEBAStudio. This causes the buffer
                 //to eventually exceed its size causing an "error while writing" error. This might produce a disconnection between the Kinect and the Switch, causing the user
                 //to loose control of the Robot
                if (commandCount > 10) {
                    startCMD = true;
                    // Terminate CMD Process
                    cmd.StandardInput.Flush();                   
                    cmd.StandardInput.Close();
                    cmd.WaitForExit();
                    // Restart CMD Process   
                    initCMD();
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
