//
// Created by astro on 2021/4/25.
//

#include "engineer_middleware/middleware.h"
#include <unistd.h>
namespace engineer_middleware {
Middleware::Middleware(ros::NodeHandle &nh) :
    nh_(nh), action_(nh_, "move_arm", boost::bind(&Middleware::executeCB, this, _1), false) {
}

}