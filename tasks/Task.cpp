/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace localization_frontend;

Task::Task(std::string const& name)
    : TaskBase(name)
{

    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initPosition = false;
    initAttitude = false;

    counter.reset();
    number.reset();
    flag.reset();

    /***************************/
    /** Output port variables **/
    /***************************/
    world2navigationRbs.invalidate();
    world_osg2worldRbs.invalidate();
    referenceOut.invalidate();

    /**********************************/
    /*** Internal Storage Variables ***/
    /**********************************/

    /** Default size for the circular_buffer of the raw port samples **/
    cbJointsSamples = boost::circular_buffer<base::samples::Joints>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbImuSamples = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbOrientationSamples = boost::circular_buffer<base::samples::RigidBodyState> (DEFAULT_CIRCULAR_BUFFER_SIZE);

    /** Default size for the circular_buffer for the filtered port samples **/
    jointsSamples = boost::circular_buffer<base::samples::Joints>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    imuSamples = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    orientationSamples = boost::circular_buffer<base::samples::RigidBodyState> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    referencePoseSamples = boost::circular_buffer<base::samples::RigidBodyState> (DEFAULT_CIRCULAR_BUFFER_SIZE);

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::reference_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &reference_pose_samples_sample)
{
    referencePoseSamples.push_front(reference_pose_samples_sample);

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER REFERENCE-POSE]Received referencePoseSamples sample at("<<reference_pose_samples_sample.time.toMicroseconds()<<") **\n";
    #endif

    if (!initPosition)
    {
        /** Set position **/
        world2navigationRbs.position = referencePoseSamples[0].position;
        world2navigationRbs.orientation = referencePoseSamples[0].orientation;
	
        world2navigationRbs.velocity.setZero();

	/** Assume well known starting position **/
	world2navigationRbs.cov_position = Eigen::Matrix3d::Zero();
	world2navigationRbs.cov_velocity = Eigen::Matrix3d::Zero();
	
	#ifdef DEBUG_PRINTS
	Eigen::Matrix <double,3,1> euler; /** In Euler angles **/
	euler[2] = referencePoseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
	euler[1] = referencePoseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
	euler[0] = referencePoseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
 	std::cout<<"** [EXOTER REFERENCE-POSE]referencePoseSamples at ("<<referencePoseSamples[0].time.toMicroseconds()<< ")**\n";
	std::cout<<"** position(world_frame)\n"<< referencePoseSamples[0].position<<"\n";
	std::cout<<"** Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
	#endif

	/** Initial angular velocity **/
	world2navigationRbs.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	world2navigationRbs.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	world2navigationRbs.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
	initPosition = true;
    }
    else if (initAttitude)
    {
        /** Transform the reference pose world_body to navigation_body **/
        Eigen::Affine3d Tworld_body = referencePoseSamples[0].getTransform();
        referencePoseSamples[0].setTransform(world2navigationRbs.getTransform().inverse() * Tworld_body);
    }

   flag.referencePoseSamples = true;
}

void Task::inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation part of the transformation in quaternion form **/

    /** Get the transformation (transformation) Tbody_imu which is body = Tbody_imu imu **/
    if (!_imu2body.get(ts, tf, false))
	return;

    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from Body to imu (transforming samples from imu to body)

    base::samples::IMUSensors imusample;

    /** A new sample arrived to the port **/
    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER INERTIAL_SAMPLES] counter.imuSamples("<<counter.imuSamples<<") at ("<<inertial_samples_sample.time.toMicroseconds()<< ")**\n";
    std::cout<<"acc(imu_frame):\n"<<inertial_samples_sample.acc<<"\n";
    std::cout<<"acc(quat body_frame ):\n"<<qtf * inertial_samples_sample.acc<<"\n";
    std::cout<<"acc(Rot body_frame):\n"<< tf.rotation() * inertial_samples_sample.acc<<"\n";
    std::cout<<"acc(Trans body_frame):\n"<< tf * inertial_samples_sample.acc<<"\n";
    std::cout<<"gyro(imu_frame):\n"<<inertial_samples_sample.gyro<<"\n";
    std::cout<<"gyro(quat body_frame):\n"<<qtf * inertial_samples_sample.gyro<<"\n";
    std::cout<<"mag(imu_frame):\n"<<inertial_samples_sample.mag<<"\n";
    std::cout<<"mag(quat body_frame):\n"<<qtf * inertial_samples_sample.mag<<"\n";
    #endif

    /** Convert the IMU values in the body frame **/
    imusample.time = inertial_samples_sample.time;
    imusample.acc = qtf * inertial_samples_sample.acc;
    imusample.gyro = qtf * inertial_samples_sample.gyro;
    imusample.mag = qtf * inertial_samples_sample.mag;

    /** Push the corrected inertial values into the buffer **/
    cbImuSamples.push_front(imusample);

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER INERTIAL_SAMPLES] Corrected inertial ("<<counter.imuSamples<<") at ("<<inertial_samples_sample.time.toMicroseconds()<< ")**\n";
    std::cout<<"acc(body_frame):\n"<<imusample.acc<<"\n";
    std::cout<<"gyro(body_frame):\n"<<imusample.gyro<<"\n";
    std::cout<<"mag(body_frame):\n"<<imusample.mag<<"\n";
    #endif

    /** Set the flag of IMU values valid to true **/
    if (!flag.imuSamples && (cbImuSamples.size() == cbImuSamples.capacity()))
        flag.imuSamples = true;

    counter.imuSamples++;

}

void Task::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation of the transformation in quaternion form **/

    /** Get the transformation (transformation) Tbody_imu**/
    if (!_imu2body.get(ts, tf, false))
	return;

    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from Body to imu (transforming samples from imu to body)

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER ORIENTATION_SAMPLES] counter.orientationSamples("<<counter.orientationSamples<<") at ("<<orientation_samples_sample.time.toMicroseconds()<< ")**\n";
    #endif

    /** Push one sample into the buffer **/
    cbOrientationSamples.push_front(orientation_samples_sample);

    /** Transform the orientation world(osg)_imu to world_body **/
    cbOrientationSamples[0].orientation = orientation_samples_sample.orientation * qtf.inverse(); // Tworld_body = Tworld_imu * (Tbody_imu)^-1
    cbOrientationSamples[0].cov_orientation = orientation_samples_sample.cov_orientation * tf.rotation().inverse(); // Tworld_body = Tworld_imu * (Tbody_imu)^-1

    if(!initAttitude)
    {
        Eigen::Quaterniond attitude = cbOrientationSamples[0].orientation;

        /** Check if there is initial pose connected **/
        if (_reference_pose_samples.connected() && initPosition)
        {
            double heading = 0.00;

            /** Check if the reference pose has a valid orientation **/
            if(base::samples::RigidBodyState::isValidValue(referencePoseSamples[0].orientation))
            {
                heading = referencePoseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
            }
            else
            {
                RTT::log(RTT::Warning)<<"Initial Heading from External Reference is not Valid."<<RTT::endlog();
            }

            /** Align the Yaw from the referencePoseSamples, Pitch and Roll from orientationSamples **/
            attitude = Eigen::Quaternion <double>(
                    Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(attitude.toRotationMatrix().eulerAngles(2,1,0)[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(attitude.toRotationMatrix().eulerAngles(2,1,0)[2], Eigen::Vector3d::UnitX()));

            attitude.normalize();

            /** Compute the world_osg to world frame **/
            world_osg2worldRbs.orientation = cbOrientationSamples[0].orientation * attitude.inverse();//Tworld_osg_body * (Tworld_body)^-1

            initAttitude = true;
        }
        else if (!_reference_pose_samples.connected())
        {
            /** Set zero position **/
            world2navigationRbs.position.setZero();

            /** Assume well known starting position **/
            world2navigationRbs.cov_position = Eigen::Matrix <double, 3 , 3>::Zero();
            world2navigationRbs.cov_velocity = Eigen::Matrix <double, 3 , 3>::Zero();

            /** Compute the world_osg to world frame **/
            world_osg2worldRbs.orientation.setIdentity();

            initPosition = true;
            initAttitude = true;
        }

        /** Set the initial attitude to the world to navigation transform **/
        if (initAttitude)
        {
            /** Store the value as the initial one for the world to navigation **/
            world2navigationRbs.orientation = attitude;
            world2navigationRbs.angular_velocity.setZero();

            /** Assume very well know initial attitude **/
            world2navigationRbs.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
            world2navigationRbs.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();

            /** Position for the world_osg to world **/
            world_osg2worldRbs.position.setZero();

            /** Assume very well know initial attitude **/
            world_osg2worldRbs.cov_position = Eigen::Matrix <double, 3 , 3>::Zero();
            world_osg2worldRbs.cov_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
            world_osg2worldRbs.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
            world_osg2worldRbs.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();

            #ifdef DEBUG_PRINTS
            Eigen::Vector3d euler;
            euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
            euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
            euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
            std::cout<< "******** [EXOTER ORIENTATION_SAMPLES]\n";
            std::cout<< "******** Initial Attitude *******"<<"\n";
            std::cout<< "Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
            #endif


            if (initPosition)
            {
                if (state() != RUNNING)
                    state(RUNNING);
            }
        }
    }
    else
    {
        /** Transform the orientation world_body to navigation_body **/
        Eigen::Quaterniond qnavigation_world((world_osg2worldRbs.orientation * world2navigationRbs.orientation).inverse());
        cbOrientationSamples[0].orientation = qnavigation_world * cbOrientationSamples[0].orientation; // Tnavigation_body = (Tworld_navigation)^-1 * Tworld_body
        cbOrientationSamples[0].cov_orientation = qnavigation_world.toRotationMatrix() * cbOrientationSamples[0].cov_orientation; // Tnavigation_body = (Tworld_navigation)^-1 * Tworld_body
    }

    /** Set the flag of IMU values valid to true **/
    if (!flag.orientationSamples && (cbOrientationSamples.size() == cbOrientationSamples.capacity()))
        flag.orientationSamples = true;

    counter.orientationSamples++;
}

void Task::joints_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    /** A new sample arrived to the Input port **/
    cbJointsSamples.push_front(joints_samples_sample);
    counter.jointsSamples++;

    if (!flag.jointsSamples && (cbJointsSamples.size() == cbJointsSamples.capacity()))
    	flag.jointsSamples = true;
    else
	flag.jointsSamples = false;

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER ENCODERS-SAMPLES] counter.jointsSamples("<<counter.jointsSamples<<") at ("<<joints_samples_sample.time.toMicroseconds()
	<<") received FR ("<<joints_samples_sample[0].position<<")**\n";
    #endif

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER ENCODERS-SAMPLES] [COUNTERS] jointsCounter ("<<counter.jointsSamples<<") imuCounter("<<counter.imuSamples<<") orientationSamples("<<counter.orientationSamples<<") **\n";
    std::cout<<"** [EXOTER ENCODERS-SAMPLES] [FLAGS] initAttitude ("<<initAttitude<<") initPosition("<<initPosition<<") **\n";
    std::cout<<"** [EXOTER ENCODERS-SAMPLES] [FLAGS] flagJoints ("<<flag.jointsSamples<<") flagIMU("<<flag.imuSamples<<") flagOrient("<<flag.orientationSamples<<") **\n";
    #endif

    if (state() == RUNNING)
    {
        if (flag.imuSamples && flag.orientationSamples && flag.jointsSamples)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"[ON] ** [EXOTER ENCODERS-SAMPLES] ** [ON] ("<<jointsSamples[0].time.toMicroseconds()<<")\n";
       	    #endif

            /** Get the correct values from the input ports buffers  **/
            this->inputPortSamples();

            /** Calculate velocities from the input ports **/
            this->calculateJointsVelocities();

            /** Out port the information of the  **/
            this->outputPortSamples ();

            /** Reset back the counters and the flags **/
            counter.reset();
            flag.reset();

        }

        /** Sanity check: Reset counter in case of inconsistency **/
        if (counter.jointsSamples > cbJointsSamples.size())
            counter.jointsSamples = 0;
        if (counter.imuSamples > cbImuSamples.size())
            counter.imuSamples = 0;
        if (counter.orientationSamples > cbOrientationSamples.size())
            counter.orientationSamples = 0;
    }
}

void Task::left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER LEFT-CAMERA] Frame at: "<<left_frame_sample->time.toMicroseconds()<<"\n";
    #endif

    /** Undistorted image depending on meta data information **/
    ::base::samples::frame::Frame *frame_ptr = leftFrame.write_access();
    frame_ptr->time = left_frame_sample->time;
    frame_ptr->init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), left_frame_sample->getFrameMode());
    frameHelperLeft.convert(*left_frame_sample, *frame_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
    leftFrame.reset(frame_ptr);

    /** Write the camera frame into the port **/
    _left_frame_out.write(leftFrame);

    return;
}

void Task::right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER RIGHT-CAMERA] Frame at: "<<right_frame_sample->time.toMicroseconds()<<"\n";
    #endif

    /** Undistorted image depending on meta data information **/
    ::base::samples::frame::Frame *frame_ptr = rightFrame.write_access();
    frame_ptr->time = right_frame_sample->time;
    frame_ptr->init(right_frame_sample->size.width, right_frame_sample->size.height, right_frame_sample->getDataDepth(), right_frame_sample->getFrameMode());
    frameHelperRight.convert(*right_frame_sample, *frame_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
    rightFrame.reset(frame_ptr);

    /** Write the camera frame into the port **/
    _right_frame_out.write(rightFrame);

    return;
}

void Task::point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample)
{
    /** Get the transformation from the transformer **/
    Eigen::Affine3d tf;

    /** Get the transformation (transformation) Tbody_laser which transforms laser pints two body points **/
    if(!_laser2body.get( ts, tf ))
    {
        RTT::log(RTT::Warning)<<"[ EXOTER POINT CLOUD FATAL ERROR] No transformation provided for the transformer."<<RTT::endlog();
        return;
    }

    base::samples::Pointcloud pointcloud;

    /** Transform the point cloud in body frame **/
    pointcloud.time = point_cloud_samples_sample.time;
    pointcloud.points.resize(point_cloud_samples_sample.points.size());
    pointcloud.colors = point_cloud_samples_sample.colors;
    register int k = 0;
    for (std::vector<base::Point>::const_iterator it = point_cloud_samples_sample.points.begin();
        it != point_cloud_samples_sample.points.end(); it++)
    {
        pointcloud.points[k] = tf * (*it);
        k++;
    }

    /** Write the point cloud into the port **/
    _point_cloud_samples_out.write(pointcloud);

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /************************/
    /** Read configuration **/
    /************************/
    proprioceptive_output_frequency = _proprioceptive_output_frequency.value();
    jointNames = _jointNames.value();
    filterConfig = _filter_config.value();
    filter_jointNames = _filter_jointNames.value();

    /*******************************************/
    /** Initial world to navigation transform **/
    /*******************************************/

    /** Set the initial world to navigation frame transform (transformation for transformer) **/
    world2navigationRbs.invalidate();
    world2navigationRbs.sourceFrame = _navigation_source_frame.get();
    world2navigationRbs.targetFrame = _navigation_target_frame.get();

    /******************************************/
    /** Initial world_osg to world transform **/
    /******************************************/
    world_osg2worldRbs.invalidate();
    world_osg2worldRbs.sourceFrame = _world_source_frame.get();
    world_osg2worldRbs.targetFrame = _world_target_frame.get();

    /******************************************/
    /** Use properties to Configure the Task **/
    /******************************************/

    /** Set the Input ports counter to Zero **/
    counter.reset();

    /** Set the number of samples between each sensor input (if there are not coming at the same sampling rate) */
    if (proprioceptive_output_frequency != 0.00)
    {
	number.imuSamples = (1.0/_inertial_samples_period.value())/proprioceptive_output_frequency;
	number.orientationSamples = (1.0/_orientation_samples_period.value())/proprioceptive_output_frequency;
	number.jointsSamples = (1.0/_joints_samples_period.value())/proprioceptive_output_frequency;
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER CONFIGURE] cbJointsSamples has init capacity "<<cbJointsSamples.capacity()<<" and size "<<cbJointsSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] cbImuSamples has init capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] cbOrientationSamples has capacity "<<cbOrientationSamples.capacity()<<" and size "<<cbOrientationSamples.size()<<"\n";
    #endif

    /** Set the capacity of the circular_buffer according to the sampling rate **/
    cbJointsSamples.set_capacity(number.jointsSamples);
    cbImuSamples.set_capacity(number.imuSamples);
    cbOrientationSamples.set_capacity(number.orientationSamples);

    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER CONFIGURE] cbJointsSamples has capacity "<<cbJointsSamples.capacity()<<" and size "<<cbJointsSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] cbImuSamples has capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] cbOrientationSamples has capacity "<<cbOrientationSamples.capacity()<<" and size "<<cbOrientationSamples.size()<<"\n";
    #endif

    for(register unsigned int i=0; i<cbJointsSamples.size(); ++i)
    {
	cbJointsSamples[i].resize(jointNames.size());
    }

    /** Initialize the samples for the filtered buffer joint values **/
    for(register unsigned int i=0;i<jointsSamples.size();i++)
    {
	/** Sizing the joints **/
	jointsSamples[i].resize(jointNames.size());
    }

    /** Initialize the samples for the filtered buffer imuSamples values **/
    for(register unsigned int i=0; i<imuSamples.size();++i)
    {
	/** IMU Samples **/
	imuSamples[i].acc[0] = base::NaN<double>();
	imuSamples[i].acc[1] = base::NaN<double>();
	imuSamples[i].acc[2] = base::NaN<double>();
	imuSamples[i].gyro = imuSamples[0].acc;
	imuSamples[i].mag = imuSamples[0].acc;
    }

    /** Initialize the samples for the filtered buffer referencePoseSamples values **/
    for(register unsigned int i=0; i<referencePoseSamples.size(); ++i)
    {
	/** Pose Init **/
	referencePoseSamples[i].invalidate();
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER CONFIGURE] jointsSamples has capacity "<<jointsSamples.capacity()<<" and size "<<jointsSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] imuSamples has capacity "<<imuSamples.capacity()<<" and size "<<imuSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] orientationSamples has capacity "<<orientationSamples.capacity()<<" and size "<<orientationSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] referencePoseSamples has capacity "<<referencePoseSamples.capacity()<<" and size "<<referencePoseSamples.size()<<"\n";
    #endif

    /** Output Joints state vector **/
    jointsSamplesOut.resize(jointNames.size());
    jointsSamplesOut.names = jointNames;


    /** Output images **/
    ::base::samples::frame::Frame *lFrame = new ::base::samples::frame::Frame();
    ::base::samples::frame::Frame *rFrame = new ::base::samples::frame::Frame();

    leftFrame.reset(lFrame);
    rightFrame.reset(rFrame);

    lFrame = NULL; rFrame = NULL;

    /** Frame Helper **/
    frameHelperLeft.setCalibrationParameter(_left_camera_parameters.value());
    frameHelperRight.setCalibrationParameter(_right_camera_parameters.value());

    /*********************/
    /** Low-Pass Filter **/
    /*********************/
    Eigen::Matrix <double, FILTER_ORDER+1, 1> bCoeff;
    bCoeff =  filterConfig.feedForwardCoeff;

    /** Create the Low-pass filter with the right coefficients **/
    low_pass_filter.reset(new localization::FIR<FILTER_ORDER, FILTER_VECTOR_SIZE> (bCoeff));

    /** Information of the configuration **/
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Frequency of IMU samples[Hertz]: "<<(1.0/_inertial_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Frequency of Orientation samples[Hertz]: "<<(1.0/_orientation_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Frequency of Joints samples[Hertz]: "<<(1.0/_joints_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Frequency of Reference System [Hertz]: "<<(1.0/_reference_pose_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Output Frequency for Proprioceptive Inputs[Hertz]: "<<proprioceptive_output_frequency<<RTT::endlog();

    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] number.jointsSamples: "<<number.jointsSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] number.imuSamples: "<<number.imuSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] number.orientationSamples: "<<number.orientationSamples<<RTT::endlog();

    if (filterConfig.filterOn)
        RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Low-Pass Filter [ON]"<<RTT::endlog();
    else
        RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Low-Pass Filter [OFF]"<<RTT::endlog();

    if (filter_jointNames.size() != FILTER_VECTOR_SIZE)
    {
        RTT::log(RTT::Warning)<<"[Localization Front-End] [FATAL ERROR] The number of joints to perform the filter has to be "<<FILTER_VECTOR_SIZE<<RTT::endlog();
        RTT::log(RTT::Warning)<<"[Localization Front-End] [FATAL ERROR] Otherwise change the FILTER_VECTOR_SIZE constant in the code."<<RTT::endlog();
        return false;
    }

    for(std::vector<std::string>::const_iterator it_name = filter_jointNames.begin(); it_name != filter_jointNames.end(); it_name++)
    {
        std::vector<std::string>::const_iterator it = find(jointNames.begin(), jointNames.end(), *it_name);
        if (it == jointNames.end())
            throw std::runtime_error("[Localization Front-End] [FATAL ERROR]: Joints names for filter must be contained in all joints names property.");
    }


    if ((number.jointsSamples == 0)||(number.imuSamples == 0))
    {
        RTT::log(RTT::Warning)<<"[Localization Front-End] [FATAL ERROR] Output frequency cannot be higher than sensors frequency."<<RTT::endlog();
        return false;
    }


    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    /** Initial position state **/
    if (state() != INITIAL_POSITIONING)
    {
        if (!initPosition && !initAttitude)
            state(INITIAL_POSITIONING);
    }

    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}


void Task::inputPortSamples()
{
    unsigned int cbJointsSize = cbJointsSamples.size();
    unsigned int cbImuSize = cbImuSamples.size();
    unsigned int cbOrientationSize = cbOrientationSamples.size();

    /** Local variable of the ports **/
    base::samples::Joints joint;
    base::samples::IMUSensors imu;
    base::samples::RigidBodyState orientation;

    /** Sizing the joints **/
    joint.resize(jointNames.size());

    #ifdef DEBUG_PRINTS
    std::cout<<"[GetInportValue] cbJointsSamples has capacity "<<cbJointsSamples.capacity()<<" and size "<<cbJointsSamples.size()<<"\n";
    std::cout<<"[GetInportValue] cbOrientationSamples has capacity "<<cbOrientationSamples.capacity()<<" and size "<<cbOrientationSamples.size()<<"\n";
    std::cout<<"[GetInportValue] cbImuSamples has capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    #endif

    /** ********* **/
    /**  Joints   **/
    /** ********* **/

    /** Process the buffer **/
    for (register size_t j = 0; j<joint.size(); ++j)
    {
        for (register size_t i = 0; i<cbJointsSize; ++i)
        {
	    joint[j].speed += cbJointsSamples[i][j].speed;
	    joint[j].effort += cbJointsSamples[i][j].effort;
	}
        joint[j].position = cbJointsSamples[0][j].position;
        joint[j].speed /= cbJointsSize;
        joint[j].effort /= cbJointsSize;
    }

    if (cbJointsSize > 0)
    {
        /** Get the joints names **/
        joint.names = cbJointsSamples[0].names;

        /** Set the time **/
        joint.time = (cbJointsSamples[cbJointsSize-1].time + cbJointsSamples[0].time)/2.0;

        /** ****** **/
        /**  FILTER   **/
        /** ****** **/

        /** Low-pass Filter **/
        if (filterConfig.filterOn)
        {
            /** Get the joints to filter **/
            register int idx = 0;
            Eigen::Matrix<double, FILTER_VECTOR_SIZE, 1> filter_jointVector;
            for(std::vector<std::string>::const_iterator it = filter_jointNames.begin(); it != filter_jointNames.end(); it++)
            {
                filter_jointVector[idx] = joint.getElementByName(*it).position;
                idx++;
            }

            /** Filter step **/
            filter_jointVector = this->low_pass_filter->perform(filter_jointVector);

            /** Set the filtered joints **/
            idx = 0;
            for(std::vector<std::string>::const_iterator it = filter_jointNames.begin(); it != filter_jointNames.end(); it++)
            {
                joint[*it].position = filter_jointVector[idx];
                idx++;
            }
        }


        /** Push the result in the buffer **/
        jointsSamples.push_front(joint);
    }


    /** ******************* **/
    /** Orientation samples **/
    /** ******************* **/
    if (cbOrientationSize > 0)
    {
        orientation.orientation = cbOrientationSamples[0].orientation;
        orientation.cov_orientation = cbOrientationSamples[0].cov_orientation;

        /** Set the time **/
        orientation.time = (cbOrientationSamples[cbOrientationSize-1].time + cbOrientationSamples[0].time)/2.0;

        /** Push the result into the buffer **/
        orientationSamples.push_front(orientation);

    }

    /** ************ **/
    /** IMU samples **/
    /** ************ **/
    imu.acc.setZero();
    imu.gyro.setZero();
    imu.mag.setZero();

    /** Process the buffer **/
    for (register unsigned int i=0; i<cbImuSize; ++i)
    {
	imu.acc += cbImuSamples[i].acc;
	imu.gyro += cbImuSamples[i].gyro;
	imu.mag += cbImuSamples[i].mag;
    }

    /** Set the time **/
    if (cbImuSamples.size() > 0)
    {
        imu.time = (cbImuSamples[cbImuSize-1].time + cbImuSamples[0].time)/2.0;

        /** Set the mean of this time interval **/
        imu.acc /= cbImuSize;
        imu.gyro /= cbImuSize;
        imu.mag /= cbImuSize;

        /** Push the result into the buffer **/
        imuSamples.push_front(imu);
    }

    /*****************************/
    /** Store the Joint values  **/
    /*****************************/
    for (register int i=0; i<static_cast<int> ((joint.size())); ++i)
    {
        jointsSamplesOut[i].position = joint[i].position;
    }

    /** Set all counters to zero **/
    counter.reset();

    return;
}

void Task::calculateJointsVelocities()
{
    double delta_t = (1.0/proprioceptive_output_frequency);

    /** Joint velocities for the vector **/
    register int jointIdx = 0;
    base::samples::Joints joints = jointsSamples[0];
    base::samples::Joints prev_joints;

    if (static_cast<int>(jointsSamples.size()) > 1)
    {
        prev_joints = jointsSamples[1];
    }
    else
    {
        prev_joints = jointsSamples[0];
    }

    for(std::vector<std::string>::const_iterator it = joints.names.begin();
        it != joints.names.end(); it++)
    {

        base::JointState const &joint_state(joints[*it]);

        /** Calculate speed in case there is not speed information **/
        if (!joint_state.hasSpeed())
        {
            base::JointState const &prev_joint_state(prev_joints[*it]);

            #ifdef DEBUG_PRINTS
            base::Time jointsDelta_t = joints.time - prev_joints.time;

            std::cout<<"[CALCULATING_VELO] ********************************************* \n";
            std::cout<<"[CALCULATING_VELO] Joint Name:"<<*it <<"\n";
            std::cout<<"[CALCULATING_VELO] Encoder Timestamp New: "<< joints.time.toMicroseconds() <<" Timestamp Prev: "<<prev_joints.time.toMicroseconds()<<"\n";
            std::cout<<"[CALCULATING_VELO] Delta time(joints): "<< jointsDelta_t.toSeconds()<<"\n";
            std::cout<<"[CALCULATING_VELO] ********************************************* \n";
            #endif

            /** At least two values to perform the derivative **/
            jointsSamplesOut[jointIdx].speed = (joint_state.position - prev_joint_state.position)/delta_t;
        }
        else
        {
            jointsSamplesOut[jointIdx].speed = joint_state.speed;
        }

        #ifdef DEBUG_PRINTS
        std::cout<<"[CALCULATING_VELO] ["<<jointIdx<<"] joint speed: "<< jointsSamplesOut[jointIdx].speed <<"\n";
        std::cout<<"[CALCULATING_VELO] ["<<jointIdx<<"] jointsSamples old velocity: "<<(joints[jointIdx].position - prev_joints[jointIdx].position)/delta_t<<"\n";
        #endif

        jointIdx++;
    }

    return;
}

void Task::outputPortSamples()
{
    std::vector<Eigen::Affine3d> fkRobot;

    /*******************************************/
    /** Port out the Output Ports information **/
    /*******************************************/

    /** Joint samples out **/
    jointsSamplesOut.time = jointsSamples[0].time;
    _joints_samples_out.write(jointsSamplesOut);

    /** Calibrated and compensate inertial values **/
    inertialSamplesOut = imuSamples[0];
    _inertial_samples_out.write(inertialSamplesOut);

    /** Orientation Samples  **/
    _orientation_samples_out.write(orientationSamples[0]);

    /** Ground Truth if available **/
    if (_reference_pose_samples.connected())
    {
        /** Port Out the info coming from the ground truth **/
        referenceOut = referencePoseSamples[0];
        referenceOut.velocity = world2navigationRbs.orientation.inverse() * referencePoseSamples[0].velocity; //velocity in navigation frame
        referenceOut.time = jointsSamples[0].time;
        _reference_pose_samples_out.write(referenceOut);

        /** Delta increments of the ground truth at delta_t given by the output_frequency **/
        referenceOut.position = referencePoseSamples[0].position - referencePoseSamples[1].position;
        referenceOut.cov_position = referencePoseSamples[0].cov_position + referencePoseSamples[1].cov_position;
        referenceOut.velocity = world2navigationRbs.orientation.inverse() * (referencePoseSamples[0].velocity - referencePoseSamples[1].velocity);//in navigation frame
        referenceOut.cov_velocity = world2navigationRbs.orientation.inverse() * (referencePoseSamples[0].cov_velocity + referencePoseSamples[1].cov_velocity);
        _reference_delta_pose_samples_out.write(referenceOut);
    }

    /** Port-out the estimated world 2 navigation transform **/
    world2navigationRbs.time = jointsSamplesOut.time;//timestamp;
    _world_to_navigation_out.write(world2navigationRbs);

    /** Port-out the estimated world_osg 2 world transform **/
    world_osg2worldRbs.time = jointsSamplesOut.time;//timestamp;
    _world_osg_to_world_out.write(world_osg2worldRbs);

    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER OUTPUT_PORTS]: world2navigationRbs.position\n"<<world2navigationRbs.position<<"\n";
    std::cout<<"[EXOTER OUTPUT_PORTS]: world2navigationRbs.velocity\n"<<world2navigationRbs.velocity<<"\n";
    std::cout<<"[EXOTER OUTPUT_PORTS]: referenceOut.position\n"<<referenceOut.velocity<<"\n";
    std::cout<<"[EXOTER OUTPUT_PORTS] ******************** END ******************** \n";
    #endif

    /** The Debug OutPorts information **/
    if (_output_debug.value())
    {
        _angular_position.write(jointsSamplesOut[13].position);
        _angular_rate.write(jointsSamplesOut[13].speed); //!Front Left
    }

    return;
}

