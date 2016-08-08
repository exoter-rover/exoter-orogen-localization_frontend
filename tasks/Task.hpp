/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LOCALIZATION_FRONTEND_TASK_TASK_HPP
#define LOCALIZATION_FRONTEND_TASK_TASK_HPP

#include "localization_frontend/TaskBase.hpp"

/** General Libraries **/
#include <math.h> /** math library (for natural Log among others) **/
#include <vector> /** std vector **/
#include <fstream>
#include <string> /** This should be already in sstream **/


/** Eigen **/
#include <Eigen/Core>/** Eigen core library **/
#include <Eigen/StdVector> /** For STL container with Eigen types **/
#include <Eigen/Dense> /** Algebra and transformation matrices **/

/** Framework Library includes **/
#include <localization/filters/FIR.hpp>

/** Boost **/
#include <boost/circular_buffer.hpp> /** For circular buffers **/
#include <boost/shared_ptr.hpp> /** Shared pointers **/

/** Rock libraries **/
#include "frame_helper/FrameHelper.h" /** Rock lib for manipulate frames **/

/** ExoTer libraries **/
#include <exoter_dynamics/ReactionForces.hpp>

/* URDF */
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>

/** 3D Kinematics **/
#include <threed_odometry/KinematicKDL.hpp> /** KDL model **/

/* Base types **/
#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/Frame.hpp>
#include <base/samples/IMUSensors.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace localization_frontend {

    /** Current counter of samples arrived to each port **/
    struct CounterInputPorts
    {
        void reset()
        {
            jointsSamples = 0;
            imuSamples = 0;
            orientationSamples = 0;
            referencePoseSamples = 0;
            return;
        }

       	unsigned int jointsSamples; /** counter for encoders samples**/
     	unsigned int imuSamples; /** counter of inertial sensors samples **/
     	unsigned int orientationSamples; /** counter of orientation samples **/
     	unsigned int referencePoseSamples; /** counter of pose information coming from external measurement **/
    };

    /** Number of samples to process in the callback function **/
    struct NumberInputPorts
    {
        void reset()
        {
            jointsSamples = 0;
            imuSamples = 0;
            orientationSamples = 0;
            referencePoseSamples = 0;
            return;
        }

        unsigned int jointsSamples; /** number of encoders samples for the re-sampling**/
        unsigned int imuSamples; /** number of inertial sensors samples **/
        unsigned int orientationSamples; /** number of orientation samples **/
        unsigned int referencePoseSamples; /** number of pose information coming from external measurement **/
    };

    /** Input port samples arrived ON/OFF flags **/
    struct FlagInputPorts
    {
        void reset()
        {
            jointsSamples = false;
            imuSamples = false;
            orientationSamples = false;
            referencePoseSamples = false;
            return;
        }

        bool jointsSamples;//Encoders
        bool imuSamples;//Inertial sensors
        bool orientationSamples;//Orientation
        bool referencePoseSamples;//Initial pose
    };

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Body to Laser transformation: laser frame expressed with respect to the body frame in Euler angles.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','localization_frontend::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;

    protected:
        static const int  DEFAULT_CIRCULAR_BUFFER_SIZE = 2; /** Default number of objects to store regarding the inputs port **/
        static const unsigned int  FILTER_ORDER = 25; /** FIR filter Order **/
        static const unsigned int  FILTER_VECTOR_SIZE = 3; /** Vector dimension in the FIR filter **/

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/

        /** Initial position for the world to navigation transform **/
        bool initPosition, initAttitude;

        /** Number of samples to process in the input ports callback function **/
        NumberInputPorts number;

        /** Current counter of samples arrived to each input port **/
        CounterInputPorts counter;

        /** Data arrived ON/OFF Flag **/
        FlagInputPorts flag;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        double proprioceptive_output_frequency;

        std::vector<std::string> all_joint_names;

        std::vector<std::string> zero_position_joint_names;

        std::vector<std::string> zero_speed_joint_names;

        localization_frontend::NamedVectorString mimic_joint_names;

        localization_frontend::NamedVectorString translation_joint_names;

        std::vector<std::string> filter_joint_names;

        /** FIR filter configuration structure **/
        FilterCoefficients filterConfig;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** Number of physical joints according to the model and task properties  **/
        int number_robot_joints;

        /** Wheel radius **/
        double wheel_radius;

        /** Frame helper **/
        frame_helper::FrameHelper frameHelperLeft, frameHelperRight;

        /** Low-pass filter for Passive Joints */
        boost::shared_ptr< localization::FIR<FILTER_ORDER, FILTER_VECTOR_SIZE > > low_pass_filter;

        /** Reaction forces **/
        ::exoter_dynamics::ReactionForces exoter_rf;

        /** Robot Kinematic Model **/
        boost::shared_ptr< threed_odometry::KinematicKDL > robot_kinematics;

        /***********************************/
        /** Input ports dependent buffers **/
        /***********************************/

        /** Buffer for raw inputs port samples (the desired filter frequency for proprioceptive inputs) **/
        boost::circular_buffer<base::samples::Joints> cbJointsSamples;
        boost::circular_buffer<base::samples::IMUSensors> cbImuSamples;
        boost::circular_buffer<base::samples::RigidBodyState> cbOrientationSamples;
        boost::circular_buffer<base::samples::RigidBodyState> cbReferencePoseSamples;

        /** Buffer for filtered Inputs port samples (Store the samples and compute the velocities) **/
        boost::circular_buffer<base::samples::Joints> jointsSamples; /** Encoder Status information  **/
        boost::circular_buffer<base::samples::IMUSensors> imuSamples; /** IMU samples **/
        boost::circular_buffer<base::samples::RigidBodyState> orientationSamples; /** IMU samples **/
        boost::circular_buffer<base::samples::RigidBodyState> referencePoseSamples; /** Pose information (init and debug)**/

        /***************************/
        /** Output port variables **/
        /***************************/

        /** Joints state of the robot **/
        base::samples::Joints jointsSamplesOut;

        /** Calibrated and compensated inertial values **/
        base::samples::IMUSensors inertialSamplesOut;

        /** Ground truth out coming for an external system (if available like Vicon or GPS) */
        base::samples::RigidBodyState referenceOut;

        /** Delta pose ground truth out coming for an external system (if available like Vicon or GPS) */
        base::samples::RigidBodyState delta_referenceOut;

        /** Calculated initial navigation frame pose expressed in world frame */
        base::samples::RigidBodyState world2navigationRbs;

        /** Calculated the world_osg to intermediate world frame */
        base::samples::RigidBodyState world_osg2worldRbs;

        /** Undistorted camera images **/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> rightFrame;

    protected:

        /************************/
        /** Callback functions **/
        /************************/

        virtual void pose_reference_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_reference_samples_sample);

        virtual void inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample);

        virtual void orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

        virtual void joints_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample);

        virtual void left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample);

        virtual void right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample);

        virtual void point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample);



    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "localization_frontend::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** @brief Get the correct value from the input ports buffers
	    */
	    void inputPortSamples();

        /** @brief Compute Cartesian and Model velocities 
	     */
	    void calculateVelocities();

        /** @brief Port out the values
	    */
        void outputPortSamples();

        /** @brief Delta Distance calculation
	    */
        void distanceForExteroceptive();

        /** @brief search joint by names and get back information
         */
        bool searchURDFJointNames(urdf::LinkConstSharedPtr link, const std::string &name_to_search,
                                Eigen::Vector3d &translation);

        /** @brief unpack the joint positions for the forward kinematic model
         */
        void joints_samplesUnpack(const ::base::samples::Joints &original_joints,
                                const std::vector<std::string> &order_names,
                                std::vector<double> &joint_positions);

        /** @brief Compute the reaction forces weighting matrix
         */
        void computeWeightingMatrixDiagonal(const ::base::samples::Joints &robot_joints, const Eigen::Quaterniond &orientation,
                Eigen::Matrix<double, ::exoter_dynamics::NUMBER_OF_WHEELS, 1> &forces, base::VectorXd &matrix_diagonal);

     public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
}

#endif

