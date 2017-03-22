/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

#include <base/Matrix.hpp>

using namespace localization_frontend;

Task::Task(std::string const& name)
    : TaskBase(name)
{

    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    counter.reset();
    number.reset();
    flag.reset();

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
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::inertial_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation part of the transformation in quaternion form **/

    /** Get the transformation (transformation) Tbody_imu which is body = Tbody_imu imu **/
    tf = _body_to_imu_transformation.value();

    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from Body to imu (transforming samples from imu to body)

    /** Increment counter **/
    counter.imuSamples++;

    /** Set the flag of Inertial samples values valid to true **/
    if (counter.imuSamples == cbImuSamples.capacity())
        flag.imuSamples = true;

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

}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation of the transformation in quaternion form **/

    /** Get the transformation (transformation) Tbody_imu**/
    tf = _body_to_imu_transformation.value();

    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from Body to imu (transforming samples from imu to body)

    /** Increment counter **/
    counter.orientationSamples++;

    /** Set the flag of orientation samples values valid to true **/
    if (counter.orientationSamples == cbOrientationSamples.capacity())
        flag.orientationSamples = true;

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER ORIENTATION_SAMPLES] counter.orientationSamples("<<counter.orientationSamples<<") at ("<<orientation_samples_sample.time.toMicroseconds()<< ")**\n";
    #endif

    /** Push one sample into the buffer **/
    cbOrientationSamples.push_front(orientation_samples_sample);

    #ifdef DEBUG_PRINTS
    Eigen::Vector3d euler = base::getEuler(cbOrientationSamples[0].orientation);
    std::cout<< "******** [EXOTER ORIENTATION_SAMPLES]\n";
    std::cout<< "******** RECEIVED ATTITUDE *******"<<"\n";
    std::cout<< "Roll: "<<euler[2]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[0]*R2D<<"\n";
    #endif


    /** Transform the orientation world(osg)_imu to world(osg)_body **/
    cbOrientationSamples[0].orientation = orientation_samples_sample.orientation * qtf.inverse(); // Tworld(osg)_body = Tworld(osg)_imu * (Tbody_imu)^-1
    cbOrientationSamples[0].cov_orientation = tf.inverse().rotation() * orientation_samples_sample.cov_orientation * tf.inverse().rotation().transpose(); // Tworld(osg)_body = Tworld(osg)_imu * (Tbody_imu)^-1
    base::guaranteeSPD< Eigen::Matrix<double, 3, 3> > (cbOrientationSamples[0].cov_orientation);

    #ifdef DEBUG_PRINTS
    euler = base::getEuler(cbOrientationSamples[0].orientation);
    std::cout<< "******** [EXOTER ORIENTATION_SAMPLES]\n";
    std::cout<< "******** RECEIVED ATTITUDE IN BODY*******"<<"\n";
    std::cout<< "Roll: "<<euler[2]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[0]*R2D<<"\n";
    #endif
}

void Task::joints_samplesCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    base::samples::Joints joints;
    joints.resize(all_joint_names.size());
    joints.time = joints_samples_sample.time;
    joints.names = all_joint_names;

    /** A new sample arrived to the Input port **/
    for(std::vector<std::string>::const_iterator it = all_joint_names.begin(); it != all_joint_names.end(); it++)
    {
        try
        {
            joints[*it] = joints_samples_sample.getElementByName(*it);
        } catch(base::samples::Joints::InvalidName ex){
            joints[*it].position = 0.00;
            joints[*it].speed = base::NaN<double>();
        }
    }

    cbJointsSamples.push_front(joints);

    /** Increment counter **/
    counter.jointsSamples++;

    /** Set the flag of joints samples values valid to true **/
    if (counter.jointsSamples == cbJointsSamples.capacity())
        flag.jointsSamples = true;


    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER JOINTS-SAMPLES] counter.jointsSamples("<<counter.jointsSamples<<") at ("<<joints_samples_sample.time.toMicroseconds()
        <<") received FR position ("<<joints_samples_sample[0].position<<")**\n";
    #endif

    #ifdef DEBUG_PRINTS
    std::cout<<"** [EXOTER JOINTS-SAMPLES] [COUNTERS] jointsCounter ("<<counter.jointsSamples<<") imuCounter("<<counter.imuSamples<<") orientationSamples("<<counter.orientationSamples<<") **\n";
    std::cout<<"** [EXOTER JOINTS-SAMPLES] [FLAGS] flagJoints ("<<flag.jointsSamples<<") flagIMU("<<flag.imuSamples<<") flagOrient("<<flag.orientationSamples<<") **\n";
    #endif

    if (state() == RUNNING)
    {
        if (flag.imuSamples && flag.orientationSamples && flag.jointsSamples)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"[ON] ** [EXOTER JOINTS-SAMPLES] ** [ON] ("<<jointsSamples[0].time.toMicroseconds()<<")\n";
       	    #endif

            /** Get the correct values from the input ports buffers  **/
            this->inputPortSamples();

            /** Calculate velocities from the input ports **/
            this->calculateVelocities();

            /** Out port the information of the proprioceptive sensors **/
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
    all_joint_names = _all_joint_names.value();
    zero_position_joint_names = _zero_position_joint_names.value();
    zero_speed_joint_names = _zero_speed_joint_names.value();
    mimic_joint_names = _mimic_joint_names.value();
    translation_joint_names = _translation_joint_names.value();
    filterConfig = _filter_config.value();
    filter_joint_names = _filter_joint_names.value();

    /******************************************/
    /** Use properties to Configure the Task **/
    /******************************************/

    /** Set the Input ports counter to Zero **/
    counter.reset();

    /** Set the number of samples between each sensor input (if there are not coming at the same sampling rate) */
    if (proprioceptive_output_frequency != 0.00)
    {
        number.imuSamples = (1.0/_inertial_samples_period.value())/proprioceptive_output_frequency;
        number.jointsSamples = (1.0/_joints_samples_period.value())/proprioceptive_output_frequency;
        number.orientationSamples = (1.0/_orientation_samples_period.value())/proprioceptive_output_frequency;
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
    	cbJointsSamples[i].resize(all_joint_names.size());
    }

    /** Initialize the samples for the filtered buffer joint values **/
    for(register unsigned int i=0; i<jointsSamples.size(); i++)
    {
    	/** Sizing the joints **/
	    jointsSamples[i].resize(all_joint_names.size());
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

    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER CONFIGURE] jointsSamples has capacity "<<jointsSamples.capacity()<<" and size "<<jointsSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] imuSamples has capacity "<<imuSamples.capacity()<<" and size "<<imuSamples.size()<<"\n";
    std::cout<<"[EXOTER CONFIGURE] orientationSamples has capacity "<<orientationSamples.capacity()<<" and size "<<orientationSamples.size()<<"\n";
    #endif

    /** Output Joints state vector **/
    jointsSamplesOut.resize(all_joint_names.size());
    jointsSamplesOut.names = all_joint_names;


    /**************************/
    /***** Read URDF file *****/
    /**************************/
    std::string urdf_file = _urdf_file.value();
    std::string xml_string;
    const char * urdf_char = urdf_file.c_str();
    std::fstream xml_file(urdf_char, std::fstream::in);
    while ( xml_file.good() )
    {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
    }
    xml_file.close();

    urdf::ModelInterfaceSharedPtr robot = urdf::parseURDF(xml_string);
    if (!robot)
    {
        throw std::runtime_error("[Localization Front-End] [Info] Configuration could not parse URDF model\n");
    }

    /******************/
    /** Wheel Radius **/
    /******************/
    Eigen::Vector3d wheel_radius_offset;
    if (this->searchURDFJointNames(robot->getRoot(), _wheel_radius_joint_names.value()[0], wheel_radius_offset))
    {
        /** Minus sign because it is in z-axis negative direction with respect to body in URDF **/
        this->wheel_radius = -wheel_radius_offset[2];
        RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Wheel radius [meter]: "<<this->wheel_radius<<RTT::endlog();
    }
    else
    {
        throw std::runtime_error("[Localization Front-End] [Info] Unable to find Wheel radius joint names in given URDF\n");
    }

    /****************************/
    /** Robot Kinematics Model **/
    /****************************/
    this->number_robot_joints =  _all_joint_names.value().size() - _slip_joint_names.value().size() - _contact_joint_names.value().size();
    this->robot_kinematics.reset(new threed_odometry::KinematicKDL (urdf_file, _contact_point_segments.value(),
                            _contact_angle_segments.value(), this->number_robot_joints,
                            _slip_joint_names.value().size(), _contact_joint_names.value().size()));

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
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Output Frequency for Proprioceptive Inputs[Hertz]: "<<proprioceptive_output_frequency<<RTT::endlog();

    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] number.jointsSamples: "<<number.jointsSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] number.imuSamples: "<<number.imuSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] number.orientationSamples: "<<number.orientationSamples<<RTT::endlog();

    if (filterConfig.filterOn)
        RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Low-Pass Filter [ON]"<<RTT::endlog();
    else
        RTT::log(RTT::Warning)<<"[Localization Front-End] [Info] Low-Pass Filter [OFF]"<<RTT::endlog();

    if (filter_joint_names.size() != FILTER_VECTOR_SIZE)
    {
        RTT::log(RTT::Warning)<<"[Localization Front-End] [FATAL ERROR] The number of joints to perform the filter has to be "<<FILTER_VECTOR_SIZE<<RTT::endlog();
        RTT::log(RTT::Warning)<<"[Localization Front-End] [FATAL ERROR] Otherwise change the FILTER_VECTOR_SIZE constant in the code."<<RTT::endlog();
        RTT::log(RTT::Warning)<<"[Localization Front-End] [FATAL ERROR] This is because low_pass_filter was designed as template class ."<<RTT::endlog();
        return false;
    }

    for(std::vector<std::string>::const_iterator it_name = filter_joint_names.begin(); it_name != filter_joint_names.end(); it_name++)
    {
        std::vector<std::string>::const_iterator it = find(all_joint_names.begin(), all_joint_names.end(), *it_name);
        if (it == all_joint_names.end())
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
    double cbJointsSize = static_cast<double>(cbJointsSamples.size());
    double cbImuSize =  static_cast<double>(cbImuSamples.size());
    double cbOrientationSize =  static_cast<double>(cbOrientationSamples.size());

    /** Local variable of the ports **/
    base::samples::Joints joint;
    base::samples::IMUSensors imu;
    base::samples::RigidBodyState orientation;

    /** Sizing the joints **/
    joint.resize(all_joint_names.size());

    #ifdef DEBUG_PRINTS
    std::cout<<"[GetInportValue] cbJointsSamples has capacity "<<cbJointsSamples.capacity()<<" and size "<<cbJointsSamples.size()<<"\n";
    std::cout<<"[GetInportValue] cbImuSamples has capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    std::cout<<"[GetInportValue] cbOrientationSamples has capacity "<<cbOrientationSamples.capacity()<<" and size "<<cbOrientationSamples.size()<<"\n";
    #endif

    /** ********* **/
    /**  Joints   **/
    /** ********* **/

    /** Process the buffer **/
    for (register size_t j = 0; j<joint.size(); ++j)
    {
        joint[j].position = 0.00;
        joint[j].speed = 0.00;
        joint[j].effort = 0.00;
        joint[j].raw = 0.00;

        for (register size_t i = 0; i<cbJointsSamples.size(); ++i)
        {
            joint[j].speed += cbJointsSamples[i][j].speed;
            joint[j].effort += cbJointsSamples[i][j].effort;
            joint[j].raw += cbJointsSamples[i][j].raw;
        }

        joint[j].position = cbJointsSamples[0][j].position;
        joint[j].speed /= cbJointsSize;
        joint[j].effort /= cbJointsSize;
        joint[j].raw /= cbJointsSize;
    }

    if (cbJointsSize > 0.0)
    {
        /** Get the joints names **/
        joint.names = cbJointsSamples[0].names;

        /** Set the time **/
        joint.time = (cbJointsSamples[cbJointsSize-1].time + cbJointsSamples[0].time)/2.0;

        /** ********* **/
        /**  FILTER   **/
        /** ********* **/

        /** Low-pass Filter **/
        if (filterConfig.filterOn)
        {
            /** Get the joints to filter **/
            register int idx = 0;
            Eigen::Matrix<double, FILTER_VECTOR_SIZE, 1> filter_jointVector;
            for(std::vector<std::string>::const_iterator it = filter_joint_names.begin(); it != filter_joint_names.end(); it++)
            {
                filter_jointVector[idx] = joint.getElementByName(*it).position;
                idx++;
            }

            /** Filter step **/
            filter_jointVector = this->low_pass_filter->perform(filter_jointVector);

            /** Set the filtered joints **/
            idx = 0;
            for(std::vector<std::string>::const_iterator it = filter_joint_names.begin(); it != filter_joint_names.end(); it++)
            {
                joint[*it].position = filter_jointVector[idx];
                idx++;
            }
        }

        /** *************** **/
        /**  MIMIC JOINTS   **/
        /** *************** **/
        register int idx = 0;
        for(std::vector<std::string>::const_iterator it = mimic_joint_names.names.begin(); it != mimic_joint_names.names.end(); it++)
        {
            joint[*it].position = -joint.getElementByName(mimic_joint_names.elements[idx]).position;
            idx++;
        }

        /** ********************* **/
        /**  TRANSLATION JOINTS   **/
        /** ********************* **/
        idx = 0;
        for(std::vector<std::string>::const_iterator it = translation_joint_names.names.begin(); it != translation_joint_names.names.end(); it++)
        {
            joint[*it].position = this->wheel_radius * joint.getElementByName(translation_joint_names.elements[idx]).position;
            joint[*it].speed = this->wheel_radius * joint.getElementByName(translation_joint_names.elements[idx]).speed;
            idx++;
        }

        /** Push the result in the buffer **/
        jointsSamples.push_front(joint);
    }


    /** ******************* **/
    /** Orientation samples **/
    /** ******************* **/
    if (cbOrientationSize > 0.0)
    {
        Eigen::Matrix3d cov_orientation; cov_orientation.setZero();
        double w = 0.00;
        double x = 0.00;
        double y = 0.00;
        double z = 0.00;

        /** Process the buffer **/
        for (register unsigned int i=0; i<cbOrientationSamples.size(); ++i)
        {
            w += cbOrientationSamples[i].orientation.w();
            x += cbOrientationSamples[i].orientation.x();
            y += cbOrientationSamples[i].orientation.y();
            z += cbOrientationSamples[i].orientation.z();

            cov_orientation += cbOrientationSamples[i].cov_orientation;
        }

        w = w/cbOrientationSize; y = y/cbOrientationSize;
        x = x/cbOrientationSize; z = z/cbOrientationSize;
        orientation.orientation = Eigen::Quaterniond(w, x, y, z);
        orientation.orientation.normalize();

        orientation.cov_orientation = cov_orientation/cbOrientationSize;

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
    for (register unsigned int i=0; i<cbImuSamples.size(); ++i)
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


    /** Set all counters to zero **/
    counter.reset();

    return;
}

void Task::calculateVelocities()
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

        #ifdef DEBUG_PRINTS
        std::cout<<"[CALCULATING_VELO] ********************************************* \n";
        std::cout<<"[CALCULATING_VELO] Joint Name:"<<*it <<"\n";
        #endif

        /** Calculate speed in case there is not speed information **/
        if (!joint_state.hasSpeed())
        {
            base::JointState const &prev_joint_state(prev_joints[*it]);
            base::Time jointsDelta_t = joints.time - prev_joints.time;

            #ifdef DEBUG_PRINTS

            std::cout<<"[CALCULATING_VELO] Encoder Timestamp New: "<< joints.time.toMicroseconds() <<" Timestamp Prev: "<<prev_joints.time.toMicroseconds()<<"\n";
            std::cout<<"[CALCULATING_VELO] Delta time(joints): "<< jointsDelta_t.toSeconds()<<"\n";
            std::cout<<"[CALCULATING_VELO] Delta proprioceptive_output_frequency: "<< delta_t<<"\n";
            std::cout<<"[CALCULATING_VELO] Delta position(joints): "<< prev_joint_state.position<<" - "<<joint_state.position<<" = "<< prev_joint_state.position-joint_state.position <<"\n";
            #endif

            /** At least two values to perform the derivative **/
            joints[jointIdx].speed = (joint_state.position - prev_joint_state.position)/std::max(jointsDelta_t.toSeconds(),delta_t);
        }
        else
        {
            joints[jointIdx].speed = joint_state.speed;
        }

        #ifdef DEBUG_PRINTS
        std::cout<<"[CALCULATING_VELO] ["<<jointIdx<<"] joint speed: "<< jointsSamplesOut[jointIdx].speed <<"\n";
        std::cout<<"[CALCULATING_VELO] ["<<jointIdx<<"] jointsSamples old velocity method: "<<(joints[jointIdx].position - prev_joints[jointIdx].position)/delta_t<<"\n";
        std::cout<<"[CALCULATING_VELO] ********************************************* \n";
        #endif

        /** Set to Zero position joints **/
        std::vector<std::string>::const_iterator zerop = find(zero_position_joint_names.begin(), zero_position_joint_names.end(), *it);
        if (zerop != zero_position_joint_names.end())
        {
            joints[jointIdx].position = 0.00;
        }

        /** Set to Zero speed joints **/
        std::vector<std::string>::const_iterator zeros = find(zero_speed_joint_names.begin(), zero_speed_joint_names.end(), *it);
        if (zeros != zero_speed_joint_names.end())
        {
            joints[jointIdx].speed = 0.00;
        }


        jointIdx++;
    }
    /*****************************/
    /** Store the Joint values  **/
    /*****************************/
    jointsSamplesOut = joints;

    /** Mean velocities for the Reference Pose **/
    return;
}

bool Task::searchURDFJointNames(urdf::LinkConstSharedPtr link, const std::string &name_to_search,
                                Eigen::Vector3d &translation)
{
    double r, p, y;
    for (std::vector< std::shared_ptr <urdf::Link> >::const_iterator child =
            link->child_links.begin(); child != link->child_links.end();
            ++child)
    {
        (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(r,p,y);

        //std::cout<<"segment name: "<<link->name<<"\n";
        //std::cout<<"joint name: "<<(*child)->parent_joint->name<<"\n";
        if (link->name.compare(name_to_search) == 0)
        {
            translation[0] = (*child)->parent_joint->parent_to_joint_origin_transform.position.x;
            translation[1] = (*child)->parent_joint->parent_to_joint_origin_transform.position.y;
            translation[2] = (*child)->parent_joint->parent_to_joint_origin_transform.position.z;

            //std::cout<<"FOUND\n"<<translation<<"\n";
            return true;
        }

        if(searchURDFJointNames(*child, name_to_search, translation))
            return true;
    }

    return false;
}

void Task::joints_samplesUnpack(const ::base::samples::Joints &original_joints,
                                const std::vector<std::string> &order_names,
                                std::vector<double> &joint_positions)
{
    for(std::vector<std::string>::const_iterator it = order_names.begin(); it != order_names.end(); it++)
    {
        base::JointState const &state(original_joints[*it]);

        /** Avoid NaN values in position **/
        if (std::isfinite(state.position))
            joint_positions.push_back(state.position);
        else
            joint_positions.push_back(0.00);

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
    base::samples::Joints jointsOut;
    jointsOut = jointsSamplesOut;
    jointsOut.time = jointsSamples[0].time;

    _joints_samples_out.write(jointsOut);

    /** Calibrated and compensate inertial values **/
    inertialSamplesOut = imuSamples[0];
    _inertial_samples_out.write(inertialSamplesOut);

    /** Orientation Samples  **/
    orientationSamples[0].sourceFrame = _orientation_source_frame.get();
    orientationSamples[0].targetFrame = _orientation_target_frame.get();
    _orientation_samples_out.write(orientationSamples[0]);

    return;
}

