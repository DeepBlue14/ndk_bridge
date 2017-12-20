/**
 * File: test.cpp
 * Author: James Kuczynski <jkuczyns@cs.uml.edu>
 * File Description: Simple "chatter" ROS node which publishes a message every time it receives
 *                   one.
 * Created: 01/03/2017
 * Last Modified: 01/17/2017
 */

// Android-specific lib used for converting between Java/C++ data types (i.e. Java String --> jstring --> std::string
#include <jni.h>
#include <android/log.h>

// C++ headers
#include <string>
#include <sstream>
#include <vector>
#include <set>
#include <fstream>

#include <boost/thread.hpp>

// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>


std::string msgToPub = "hello ROS!"; /// The message to publish
int loopCount = 0; /// global counter to keep track of msgs received
ros::Publisher chatter_pub; /// ROS publisher
ros::Subscriber sub;
/**
 * Reference to the JavaVM to get access to Java function tables (so we can call Java functions
 * from C++) by accessing the JNI environment (JNIEnv).  A reference to the JNIEnv should NOT
 * be saved directly, since Java maintains ownership of this data type.  Thus, the Java garbage
 * collector can delete it even if the C++-side is still using it.
 */
static JavaVM* g_jvm;
jobject g_obj; /// Reference to the instance of the JCxxComm class so we can call its methods.

//destroy check vars
typedef struct JMethod
{
    bool isLookupFirstCheck = true; //check to see if we have already queried the method
    jclass clazz; // Reference to the Java class
    jmethodID methodId; // Reference to the Java class method
} JMethod;
JMethod g_destroyJMeth; // Data to use the destroyRequested Java method from C++
JMethod g_updateUiJMeth; // Data to use the updateUi Java method from C++

enum RosNodeState { UNCONNECTED, CONNECTED, DISCONNECTED, CLOSE };
RosNodeState m_connection;
bool running = true;

ros::NodeHandle* m_n;


/**
 * Use the Android logging system from C++
 */
void log(const char* msg, ...)
{
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "ROSCPP_NDK_EXAMPLE", msg, args);
    va_end(args);
}


JMethod jmethLookup(const char* package, const char* clazz, const char* method, const char* args = 0)
{
    ;//TODO: implement
}


bool destroyRequested()
{
    /*
     * Call Java method to update the UI.
     * TODO: Should the look-up be done only once?
     */

    //---------------------------------------------------
    JNIEnv* env = new JNIEnv();

    jint rs = g_jvm->AttachCurrentThread(&env, NULL);
    (rs == JNI_OK) ? log("SUCCESSFULLY passed JNIEnv ref") : log("FAILED to pass JNIEnv reff");

    if(g_destroyJMeth.isLookupFirstCheck)
    {
        g_destroyJMeth.clazz = env->FindClass("edu/uml/cs/danrosjcxxprac/UniversalDat");
        g_destroyJMeth.methodId = env->GetStaticMethodID(g_destroyJMeth.clazz, "destroyRequested", "()Z");
        if(g_destroyJMeth.methodId == 0)
            log("ERROR: attempting to get methodID");
        g_destroyJMeth.isLookupFirstCheck = false;
    }


    jboolean response = env->CallStaticBooleanMethod(g_destroyJMeth.clazz, g_destroyJMeth.methodId);
    bool desRec = (jboolean) response;
    //---------------------------------------------------

    return desRec;
}


/**
 * Regular roscpp callback function.
 */
void callback(const std_msgs::StringConstPtr& msg)
{
    log("%s", msg->data.c_str() );

    /*
     * Call Java method to update the UI.
     * TODO: Should the look-up be done only once?
     */
    //---------------------------------------------------
    JNIEnv* env = new JNIEnv();

    jint rs = g_jvm->AttachCurrentThread(&env, NULL);
    (rs == JNI_OK) ? log("SUCCESSFULLY passed JNIEnv ref") : log("FAILED to pass JNIEnv reff");

    jstring jstr = env->NewStringUTF(msg->data.c_str() );

    if(g_updateUiJMeth.isLookupFirstCheck)
    {
        g_updateUiJMeth.clazz = env->FindClass("edu/uml/cs/danrosjcxxprac/JCxxComm");
        g_updateUiJMeth.methodId = env->GetMethodID(g_updateUiJMeth.clazz, "updateUi", "(Ljava/lang/String;)V");
    }
    //class clazz = env->FindClass("edu/uml/cs/danrosjcxxprac/JCxxComm");
    //jmethodID messageMe = env->GetMethodID(clazz, "updateUi", "(Ljava/lang/String;)V");
    env->CallVoidMethod(g_obj, g_updateUiJMeth.methodId, jstr);
    //---------------------------------------------------

    // Regular ROS sub/pub stuff
    std_msgs::String msgo;
    std::stringstream ss;
    ss << msgToPub.c_str() << loopCount++;
    msgo.data = ss.str();
    chatter_pub.publish(msgo);
    log(msg->data.c_str());
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


void startROSComms()
{
    /*** Initialize ROS Communication ***/
    log("Starting ROS Communications");
    m_n = new ros::NodeHandle();

    chatter_pub = m_n->advertise<std_msgs::String>("a_chatter", 1000);
    sub = m_n->subscribe("chatter", 1000, callback);

    log("ROS Communications Established");
}


void closeROSComms()
{
    chatter_pub.shutdown();
    sub.shutdown();

    if(ros::isShuttingDown()){
        log("ROS is Down");
    }
    if(!ros::isStarted()){
        log("should restart");
    }
}


/**
 * main function of this node.
 */
int main()
{
    int argc = 3;
    // TODO: don't hardcode ip addresses
    char* argv[] = {const_cast<char*>("nothing_important"),
                    const_cast<char*>("__master:=http://robot-brain2:11311"),
                    const_cast<char*>("__ip:=10.0.7.145")}; //10.0.7.145 //10.10.10.184
    m_connection = UNCONNECTED;
    ros::init(argc, &argv[0], "android_ndk_native_cpp");

    while(m_connection != CLOSE){
        if(!ros::master::check() && m_connection != CLOSE){
            log("Waiting for ROSCore");
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
        //ROSPublisherThread(); //TODO: james commented this out
        checkRunning();
        if(m_connection == DISCONNECTED){
            log("Lost ROS Communication");
            closeROSComms();

            log("ROS Comms Down");
            m_connection = UNCONNECTED;
        }
    }
    log("Shutting down ROS Node");
    /*
    int argc = 3;
    // TODO: don't hardcode ip addresses
    char* argv[] = {const_cast<char*>("nothing_important"),
                    const_cast<char*>("__master:=http://robot-brain2:11311"),
                    const_cast<char*>("__ip:=10.0.7.145")}; //10.0.7.145 //10.10.10.184

    m_connection = UNCONNECTED;


    ros::init(argc, &argv[0], "android_ndk_native_cpp");

    std::string master_uri = ros::master::getURI();

    (ros::master::check() ) ? log("found rosmaster: ") : log("FAILED to find rosmaster: ");
    log(master_uri.c_str() );

    // Create nodehandle, subscribers and publishers
    ros::NodeHandle n;
    chatter_pub = n.advertise<std_msgs::String>("a_chatter", 1000);
    ros::Subscriber sub = n.subscribe("chatter", 1000, callback);
    ros::WallRate loop_rate(100);

    // Check if ros is fine, and if the (Java) main activity's onDestroy() method.
    while(ros::ok() && !destroyRequested() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    return EXIT_SUCCESS;
}


#ifdef __cplusplus
    extern "C" {
#endif

/**
 * Mutator function to update the message to publish
 * @see #msgToPub
 */
void Java_edu_uml_cs_danrosjcxxprac_JCxxComm_setMsgToPub(JNIEnv* env, jobject /*this*/, jstring msg)
{
    const char* nativeString = (*env).GetStringUTFChars(msg, (jboolean *) false);
    std::string tmp(nativeString);
    msgToPub = std::string(nativeString);
    (*env).ReleaseStringUTFChars(msg, nativeString);
}


/**
 * Passes references to the environmnet (for function look-up table stuff) and the instance of
 * JCxxComm.
 */
void Java_edu_uml_cs_danrosjcxxprac_JCxxComm_init(JNIEnv* env, jobject obj)
{
    jint rs = env->GetJavaVM(&g_jvm);
    (rs == JNI_OK) ? log("JVM ref successfully passed to C") : log("FAILED to pass JVM ref");

    g_obj = reinterpret_cast<jobject>(env->NewGlobalRef(obj) );
}


/**
 * Initiate the regular ROS code by calling the C++ thread's main function
 */
void Java_edu_uml_cs_danrosjcxxprac_JRos_startRosNode(JNIEnv* env, jobject obj)
{
    jint rs = env->GetJavaVM(&g_jvm);
    (rs == JNI_OK) ? log("JVM ref successfully passed to C") : log("FAILED to pass JVM ref");

    g_obj = reinterpret_cast<jobject>(env->NewGlobalRef(obj) );
    main();
}


#ifdef __cplusplus
    }
#endif