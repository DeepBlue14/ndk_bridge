/* [db] 2012.04.05
 * dan@cs.uml.edu
 */
#include <iostream>
#include <cstdio>
#include <string>
#include <signal.h>
#include <stdio.h>
#include <math.h>

/* Haptic Includes */
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#include <boost/thread.hpp>

/* ROS Includes */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <haptic_msgs/OmniStatus.h>
#include <haptic_msgs/Calibrate.h>

#define Btime boost::posix_time
#define Bperiod Btime::time_duration


/* Added by mlunderville */
/* Define physical constants (len = mm, ang = rad) */
#define THIGH_LEN 133.35
#define SHIN_LEN 133.35
#define PI 3.1415926535
#define THIGH_ANG_OFFSET PI/12
#define SHIN_ANG_OFFSET 5*PI/8
/* End added by mlunderville */



HDCallbackCode HDCALLBACK OmniStateCallback(void *pUserData);
/* ROS Settings */
std::string statusTopic;
std::string feedbackTopic;

HHD hHD = HD_INVALID_HANDLE;
HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;
bool running = true;

using namespace std;


class OmniState{
public:
    boost::mutex state_mutex;
    hduVector3Dd position;
    hduVector3Dd velocity;
    hduVector3Dd jointangles;
    hduVector3Dd gimbalangles;
    hduMatrix transform;
    hduVector3Dd inp_vel1, inp_vel2, inp_vel3;
    hduVector3Dd out_vel1;
    hduVector3Dd out_vel2;
    hduVector3Dd out_vel3;
    hduVector3Dd pos_hist1;
    hduVector3Dd pos_hist2;
    hduVector3Dd rot;
    hduVector3Dd joints;
    hduVector3Dd force;
    bool greybutton;
    bool whitebutton;
};


class OmniDev{
private:
	enum DevMode { UNINIT, INIT, ARMED, CALI };
	DevMode m_omniMode;
	OmniState *m_state;
	HHD m_hHD;
	HDSchedulerHandle m_gSchedulerCallback;
    HDErrorInfo m_error;
public:
	OmniDev(OmniState *state) : m_hHD(HD_INVALID_HANDLE), m_gSchedulerCallback(HD_INVALID_HANDLE)
	{
		m_state = state;
		m_omniMode = UNINIT;
	}

	bool init()
	{
	    cout << "Initializing Haptic" << endl;
	    m_hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	    if( HD_DEVICE_ERROR(m_error = hdGetError())){
			hduPrintError(stderr, &m_error, "Failed to initialize haptic device");
			ROS_ERROR("Failed to initialize haptic device");
			return false;
	    }
		hdEnable(HD_FORCE_OUTPUT);
		hdStartScheduler();
		m_omniMode = INIT;
	    return true;
	}
	bool uninit()
	{
		/* This is our final act... in which we segfault cause sensable sucks */
		hdStopScheduler();
		hdDisable(HD_FORCE_OUTPUT);
		if(m_hHD != HD_INVALID_HANDLE){
			cout << "exit hdDisableDevice" << endl;
			hdDisableDevice(hdGetCurrentDevice());
			m_hHD = HD_INVALID_HANDLE;
			return true;
		}
		return false;
	}
	bool start()
	{
		if( m_omniMode != INIT){
			return false;
		}
		ROS_INFO("Starting Haptic Device");
		m_gSchedulerCallback = hdScheduleAsynchronous(OmniStateCallback, m_state, HD_MAX_SCHEDULER_PRIORITY);
		if( HD_DEVICE_ERROR(m_error = hdGetError()) ){
			ROS_ERROR("Failed to start the scheduler");
			return false;
		}
		m_omniMode = ARMED;
		return true;
	}
	bool stop()
	{
		if( m_omniMode != ARMED){
			return false;
		}
		ROS_INFO("Stoping Haptic Device");
		hdUnschedule(m_gSchedulerCallback);
		while(hdWaitForCompletion(m_gSchedulerCallback, HD_WAIT_CHECK_STATUS)){
			usleep(1000);
		}

		m_omniMode = INIT;
		return true;
	}

	bool calibrate()
	{
		int calibrationStyle;
		int supportedCalibrationStyles;
		/* Check that we are in the right state first */
		if( m_omniMode != INIT){
			ROS_ERROR("OMNI must be in state \"init\" for calibration.");
			return false;
		}

		/* Test to see what kind of calibration we support */
		hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
		if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET){
			calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		}
		if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL){
			calibrationStyle = HD_CALIBRATION_INKWELL;
	    }
	    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO){
		   calibrationStyle = HD_CALIBRATION_AUTO;
	    }
	    if(calibrationStyle != HD_CALIBRATION_INKWELL){
	    	ROS_ERROR("We only support devices with inkwell calibration");
	    	return false;
	    }

	    hdStartScheduler();
	    if(HD_DEVICE_ERROR(m_error = hdGetError())){
	    	ROS_ERROR("Failed to start haptic schedule");
	    	return false;
	    }

	    HDenum status;
//	    hdScheduleSynchronous(CalibrationStatusCallback, &status, HD_DEFAULT_SCHEDULER_PRIORITY);
	    if(status == HD_CALIBRATION_NEEDS_MANUAL_INPUT){
	    	ROS_INFO("Calibrating.. (put stylus in well)\n");
	    }
	    return true;
	}// end calibration routine

};


class OmniNode{
public:
    ros::NodeHandle *m_n;
    ros::Publisher m_statusWriter;
    ros::Subscriber m_reader;
    ros::ServiceServer m_calibrateService;
    OmniState *_state;
    OmniDev *m_dev;
    boost::thread *m_pubthread;

    enum RosNodeState { UNCONNECTED, CONNECTED, DISCONNECTED, CLOSE };
    RosNodeState m_connection;
    OmniNode(OmniDev *dev)
    {
    	m_dev = dev;
    }
    ~OmniNode()
    {
    	delete m_n;
    }
    void init(OmniState* state)
    {
        _state = state;
        m_connection = UNCONNECTED;

    }

    void startROSComms()
    {
    	/*** Initialize ROS Communication ***/
		ROS_INFO("Starting ROS Communications");
    	m_n = new ros::NodeHandle();
        /* Read in Feedback (type geometry_msgs::Vector3) */
        ros::param::param(string("~feedback_topic"), feedbackTopic,string( "/haptic/feedback"));
        m_reader = m_n->subscribe(feedbackTopic.c_str(), 1000, &OmniNode::ROSFeedbackCallback, this);
        /* Write out Omni Status */
		ros::param::param(string("~status_topic"), statusTopic, string("/haptic/status"));
        m_statusWriter = m_n->advertise<haptic_msgs::OmniStatus>(statusTopic.c_str(), 1000);
        /* Start the service */
        m_calibrateService = m_n->advertiseService("/haptic/Calibrate",&OmniNode::calibrateService,this);
        /* TODO: Initialize the rest of the state */
		ROS_INFO("ROS Communications Established");
    }
    void closeROSComms()
    {
    	m_reader.shutdown();
    	m_statusWriter.shutdown();
    	if(ros::isShuttingDown()){
			cout << "ROS is Down" << endl;
		}
		if(!ros::isStarted()){
			cout << "should restart" << endl;
		}
    }

    void mainThread()
    {
    	while(m_connection != CLOSE){
			if(!ros::master::check() && m_connection != CLOSE){
				ROS_INFO("Waiting for ROSCore");
			}
			while(!ros::master::check() && m_connection != CLOSE){
				checkRunning();
				usleep(1000);
			}
			if(m_connection == CLOSE){
				break;
			}
			startROSComms();
			m_connection = CONNECTED;
			ROSPublisherThread();
			checkRunning();
			if(m_connection == DISCONNECTED){
				ROS_INFO("Lost ROS Communication");
				closeROSComms();

				ROS_INFO("ROS Comms Down");
				m_connection = UNCONNECTED;
			}
    	}
    	ROS_INFO("Shutting down ROS Node");
    }

    void checkRunning()
    {
    	if( !ros::master::check() && m_connection == CONNECTED){
    		m_connection = DISCONNECTED;
    	}
    	if( !running ){
    		m_connection = CLOSE;
    	}
    }

    /*** ROS Force Feedback Callback ***/
    void ROSFeedbackCallback(const geometry_msgs::Vector3ConstPtr& force)
    {
        /* Set forces, with dampening */
        boost::mutex::scoped_lock lock(_state->state_mutex);
        _state->force[0] = force->x; // - 0.001*_state->velocity[0];
        _state->force[1] = force->y; // - 0.001*_state->velocity[1];
        _state->force[2] = force->z; // - 0.001*_state->velocity[2];
    }

    void FeedbackSafety()
    {
    	if(m_reader.getNumPublishers() == 0){
    		boost::mutex::scoped_lock lock(_state->state_mutex);
    		_state->force[0] = 0.0;
    		_state->force[1] = 0.0;
    		_state->force[2] = 0.0;
    	}
    }

    /* Boost Thread that goes and publishes shit */
    void ROSPublisherThread()
    {
        ros::AsyncSpinner spinner(2);
        spinner.start();
	int check_count = 0;
	
        while(m_connection == CONNECTED){
            haptic_msgs::OmniStatus status;
            geometry_msgs::PoseStamped pose;
            geometry_msgs::Vector3 force;
            geometry_msgs::Vector3 velocity;
            std_msgs::Float32 turet_angle;
            std_msgs::Float32 thigh_angle;
            std_msgs::Float32 shin_angle;
            std_msgs::Float32 roll_angle;
            std_msgs::Float32 pitch_angle;
            std_msgs::Float32 yaw_angle;
            geometry_msgs::Vector3 axis;
            std_msgs::Float32 angle;
            geometry_msgs::Transform transform;
            pose.header.frame_id = "omni";
            pose.header.stamp = ros::Time::now();
            {
                boost::mutex::scoped_lock lock(_state->state_mutex);

		/* Copy information from state */
                force.x = _state->force[0];
                force.y = _state->force[1];
                force.z = _state->force[2];
                velocity.x = _state->velocity[0];
                velocity.y = _state->velocity[1];
                velocity.z = _state->velocity[2];
                turet_angle.data = _state->jointangles[0];
                thigh_angle.data = _state->jointangles[1];
                shin_angle.data = _state->jointangles[2];
		yaw_angle.data = _state->gimbalangles[0]; //joint 4
                pitch_angle.data = _state->gimbalangles[1]; //joint 5
                roll_angle.data = _state->gimbalangles[2]; //joint6 (pen twist)

		/* Calculate "new" coordinates */
		float adj_turet_ang, adj_thigh_ang, adj_shin_ang;
		float newx, newy, newz;

		adj_turet_ang = -1 * _state->jointangles[0];
		adj_thigh_ang = _state->jointangles[1] + THIGH_ANG_OFFSET;
		adj_shin_ang = _state->jointangles[2] - SHIN_ANG_OFFSET;

		newx = THIGH_LEN*cos(adj_thigh_ang)*sin(adj_turet_ang) + SHIN_LEN*cos(adj_shin_ang)*sin(adj_turet_ang);
		newy = THIGH_LEN*sin(adj_thigh_ang) + SHIN_LEN*sin(adj_shin_ang);
		newz = THIGH_LEN*cos(adj_thigh_ang)*cos(adj_turet_ang) + SHIN_LEN*cos(adj_shin_ang)*cos(adj_turet_ang);

                pose.pose.position.x = newx;
                pose.pose.position.y = newy;
                pose.pose.position.z = newz;		

                //Transform
                /*
                hduVector3Dd RotationXVector(1,0,0);
                hduVector3Dd RotationYVector(0,1,0);
                hduVector3Dd RotationZVector(0,0,1);
                hduQuaternion QuaternionX(RotationXVector,_state->gimbalangles[0]);
                hduQuaternion QuaternionY(RotationYVector,_state->gimbalangles[1]);
                hduQuaternion QuaternionZ(RotationZVector,_state->gimbalangles[2]);
				//OrientationQuaternion has the orientation of the gimbal
                hduQuaternion OrientationQuaternion = QuaternionX * QuaternionY * QuaternionZ;
                // this has the global orientation of end effector
                hduMatrix direction = _state->transform.createRotationTranslation(OrientationQuaternion,_state->position);
                transform.rotation.x = OrientationQuaternion[0];
                transform.rotation.y = OrientationQuaternion[1];
                transform.rotation.z = OrientationQuaternion[2];
                transform.rotation.w = OrientationQuaternion[3];
				*/
                //hduQuaternion directionQuaternion(direction);
                //direction.getRotation(directionQuaternion);
                //transform.rotation.x = directionQuaternion[0];
                //transform.rotation.y = directionQuaternion[1];
                //transform.rotation.z = directionQuaternion[2];
                //transform.rotation.w = directionQuaternion[3];

                /*
                transform.rotation.x = q[0];
                transform.rotation.y = q[1];
                transform.rotation.z = q[2];
                transform.rotation.w = q[3];
                */

                /*
                hduVector3Dd toolRotationXVector(1,0,0);
                hduVector3Dd toolRotationYVector(0,1,0);
                hduVector3Dd toolRotationZVector(0,0,1);
                hduQuaternion toolQuaternionX(toolRotationXVector,_state->gimbalangles[0]);
                hduQuaternion toolQuaternionY(toolRotationYVector,_state->gimbalangles[1]);
                hduQuaternion toolQuaternionZ(toolRotationZVector,_state->gimbalangles[2]);
                hduQuaternion toolRotationQuaternion = toolQuaternionX * toolQuaternionY * toolQuaternionZ;
				transform.rotation.x = toolRotationQuaternion[0];
                transform.rotation.y = toolRotationQuaternion[1];
                transform.rotation.z = toolRotationQuaternion[2];
                transform.rotation.w = toolRotationQuaternion[3];
                */

                hduQuaternion quat;
                _state->transform.getRotation(quat);
                transform.rotation.x = quat[0];
                transform.rotation.y = quat[1];
                transform.rotation.z = quat[2];
                transform.rotation.w = quat[3];
                hduVector3Dd hduaxis;
                double hduangle;
                quat.toAxisAngle(hduaxis,hduangle);
                axis.x = hduaxis[0];
                axis.y = hduaxis[1];
                axis.z = hduaxis[2];
                angle.data = hduangle;

            }
            status.pose = pose;	    
            status.force = force;
            status.velocity = velocity;
            status.grey_button = _state->greybutton;
            status.white_button = _state->whitebutton;
            status.turet_angle = turet_angle;
            status.thigh_angle = thigh_angle;
            status.shin_angle = shin_angle;
            status.yaw_angle = yaw_angle;
            status.pitch_angle = pitch_angle;
            status.roll_angle = roll_angle;
            status.endeffector = transform;
            status.orientation_axis = axis;
            status.orientation_angle = angle;
            m_statusWriter.publish(status);

	    usleep(1000);

	    if (check_count == 100) {
	      checkRunning();
	      check_count = 0;
	    }
	    check_count++;
        }
        spinner.stop();
        /* TODO: Buttons state? */
		ROS_INFO("Exiting ROS Publisher Thread");
    }

    bool calibrateService(haptic_msgs::Calibrate::Request &req, haptic_msgs::Calibrate::Response &res)
    {
    	m_dev->stop();
    	ROS_INFO("Calibrating...");
    	usleep(1000000);
    	m_dev->start();
    	return true;
    }
};


/** Main Scheduler Callback for rendering the forces ***/
HDCallbackCode HDCALLBACK OmniStateCallback(void *pUserData)
{
    OmniState *state = static_cast<OmniState *>(pUserData);

    hdBeginFrame(hdGetCurrentDevice());
    {
        boost::mutex::scoped_lock lock(state->state_mutex);
        hdGetDoublev(HD_CURRENT_POSITION,   state->position);
        hdGetDoublev(HD_CURRENT_TRANSFORM, state->transform);
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, state->jointangles);
        hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, state->gimbalangles);

		hduVector3Dd vel_buff(0,0,0);
		vel_buff = (state->position*3 - 4*state->pos_hist1 + state->pos_hist2)/0.002;
		state->velocity = (0.2196*(vel_buff+state->inp_vel3)+0.6588*(state->inp_vel1 + state->inp_vel2))/1000.0 - (-2.7488*state->out_vel1+2.5282*state->out_vel2 - 0.7776*state->out_vel3); // cutoff freq of 20Hz
		/* track history */
		state->pos_hist2 = state->pos_hist1;
		state->pos_hist1 = state->position;
		/* track input velocity */
		state->inp_vel3 = state->inp_vel2;
		state->inp_vel2 = state->inp_vel1;
		state->inp_vel1 = vel_buff;
		/* track output velocity */
		state->out_vel3 = state->out_vel2;
		state->out_vel2 = state->out_vel1;
		state->out_vel1 = state->velocity;
    }
    hdSetDoublev(HD_CURRENT_FORCE, state->force);

    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
    state->greybutton = (buttons & HD_DEVICE_BUTTON_1) ? true : false;
    state->whitebutton = (buttons & HD_DEVICE_BUTTON_2) ? true : false;

    hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if( HD_DEVICE_ERROR(error = hdGetError()) ){
        hduPrintError(stderr, &error, "Error during main scheduler callback\n");
        if(hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    return HD_CALLBACK_CONTINUE;
}

/* Automatic calibration */
void HHD_Auto_Calibration()
{
   int calibrationStyle;
   int supportedCalibrationStyles;
   HDErrorInfo error;

   hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
   if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET){
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL){
		calibrationStyle = HD_CALIBRATION_INKWELL;
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_AUTO){
       calibrationStyle = HD_CALIBRATION_AUTO;
   }

   if(calibrationStyle != HD_CALIBRATION_INKWELL){
	   ROS_ERROR("We only support devices with inkwell calibration");
   }

   hdStartScheduler();
   if(HD_DEVICE_ERROR(error = hdGetError())){
	   ROS_ERROR("Failed to start haptic schedule");
   }
//   HDenum status;
//   hdScheduleSynchronous(CalibrationStatusCallback, &status, HD_DEFAULT_SCHEDULER_PRIORITY);
//   if(status == HD_CALIBRATION_NEEDS_MANUAL_INPUT){
//	   ROS_INFO("Calibrating.. (put stylus in well)\n");
//   }

   do{
	   hdUpdateCalibration(calibrationStyle);
       if (HD_DEVICE_ERROR(error = hdGetError())){
    	   ROS_ERROR("Reset encoders failed.");
	       hduPrintError(stderr, &error, "Reset encoders reset failed.");
	       break;
       }
   }while(hdCheckCalibration() != HD_CALIBRATION_OK);

   ROS_INFO("\n\nCalibration complete.\n");
}

void exitHandler(int sig)
{

    ROS_INFO("Signaling Shutdown");
	running = false;
}

int main(int argc, char** argv)
{
	signal(SIGABRT,exitHandler);
	signal(SIGALRM,exitHandler);
 	signal(SIGTERM,exitHandler);
	signal(SIGINT,exitHandler);

	/* Initialize Shared Omni State*/
	cout << "Initializing State" << endl;
    OmniState *state = new OmniState();

    /* Initialize Haptic Device */
    cout << "Initializing Haptic" << endl;
    OmniDev dev(state);
    dev.init();

    /* Initialize ROS */
    cout << "Initializing ROS" << endl;
    ros::init(argc, argv, "omni_node", ros::init_options::NoSigintHandler);
    OmniNode omni_ros(&dev);
    omni_ros.init(state);


    /* Start Haptic Device */
    dev.start();
	signal(SIGINT,exitHandler);
    //HHD_Auto_Calibration();

    /* Start ROS */
	ROS_INFO("Starting ROS");
    boost::thread *rosthread = new boost::thread(boost::bind(&OmniNode::mainThread,&omni_ros));

    ROS_INFO("Let the party begin!");
    while(running){
    	omni_ros.FeedbackSafety();
    	usleep(100);
    }
    if(!(rosthread->timed_join(Btime::seconds(3)))){
    	cout << "Ros Thread did not die like it should" << endl;
    }
    dev.stop();
    /* This is our final act... in which we segfault case sensable sucks */
    cout << "here goes..." << endl;
    dev.uninit();

    return 0;
}
