#ifndef moveOperationClass
#define moveOperationClass

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>


class MoveOperationClass {
    public:
        MoveOperationClass(const std::string group) {
            move = new moveit::planning_interface::MoveGroupInterface(group);
        };

        ~MoveOperationClass() {
           delete move;
       }

       moveit::planning_interface::MoveGroupInterface *move;
};

#endif