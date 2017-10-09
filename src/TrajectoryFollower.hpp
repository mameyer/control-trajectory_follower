#ifndef TRAJECTORYFOLLOWER_HPP
#define TRAJECTORYFOLLOWER_HPP

#include "TrajectoryFollowerTypes.hpp"
#include "SubTrajectory.hpp"
#include <base/commands/Motion2D.hpp>

#include "Controller.hpp"

namespace trajectory_follower
{

/**
 * TrajectoryFollower class combines reference pose finder and
 * trajectory controller
 **/
class TrajectoryFollower
{
public:
    /** Default constructor */
    TrajectoryFollower();

    /** Contructor which takes in config.
     *
     * Before using the follower make sure that the object is created
     * using the correct config and this contructor, otherwise the
     * controller will cause runtime error */
    TrajectoryFollower(const FollowerConfig& followerConfig);

    /** Sets a new trajectory
     *
     * Here it checks for the initial stability of the trajectory
     */
    void setNewTrajectory(const SubTrajectory &trajectory, const base::Pose& robotPose);

    /**
     * Marks the current trajectory as traversed
     *
     * Stops the current trajectory following and removes the trajectory
     */
    void removeTrajectory()
    {
        followerStatus = TRAJECTORY_FINISHED;
    }

    /**
     * Generates motion commands that should make the robot follow the
     * trajectory
     */
    FollowerStatus traverseTrajectory(base::commands::Motion2D &motionCmd, const base::Pose &robotPose);

    /** Computes the reference pose and the error relative to this pose */
    void computeErrors(const base::Pose& robotPose);

    /** Returns the current follower data */
    const FollowerData& getData() {
        return followerData;
    }

    bool checkTurnOnSpot();
    bool checkTrajectoryFinished();

private:
    bool configured;
    ControllerType controllerType;
    bool pointTurn;
    double pointTurnDirection;
    bool nearEnd;
    double dampingCoefficient;
    base::Pose currentPose;
    base::Pose lastPose;
    double lastPosError;
    double currentCurveParameter;
    double distanceError;
    double angleError, lastAngleError;
    double posError;
    double splineReferenceErrorCoefficient;
    FollowerData followerData;
    FollowerStatus followerStatus, lastFollowerStatus;
    SubTrajectory trajectory;
    FollowerConfig followerConf;
    Controller *controller;
};

}

#endif // TRAJECTORYFOLLOWER_HPP
