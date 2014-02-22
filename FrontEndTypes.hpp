#ifndef FRONT_END_TYPES_H
#define FRONT_END_TYPES_H

#include <vector>
#include <base/time.h>
#include <base/eigen.h>

namespace localization_frontend
{

    /****************/
    /** Properties **/
    /****************/

    /** Processing Configuration **/
    struct Configuration
    {
        double output_frequency;//It cannot be higher that the sensor values of the aggregator (transformer).

        bool align_world_to_navigation_frame; // Set at the same location the world imaginary frame and
                                            //the navigation frame (relative frame to start robot driving/odometry).
                                            //This is useful when GPS gives high values to the starting position
                                            //with respect to the origin (world_frame). Note: only the position of a pose.

        std::vector<std::string> jointsNames; //complete vector of names for the joints of the complete robot (passive and active).
    };

}
#endif
