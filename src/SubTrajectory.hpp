#ifndef TRAJECTORY_FOLLOWER_SUBTRAJECTORY
#define TRAJECTORY_FOLLOWER_SUBTRAJECTORY

#include <base/Pose.hpp>
#include <base/geometry/Spline.hpp>
#include <vector>
#include <base/Trajectory.hpp>
#include <stdexcept>
#include <string>

namespace trajectory_follower {

enum SubTrajectoryKind
{
    TRAJECTORY_KIND_NORMAL, //a normal trajectory
    TRAJECTORY_KIND_RESCUE //a special trajectory that is used to get out of unstable situations
};
    
class SubTrajectory
{
public:
    base::Pose2D startPose;
    base::Pose2D goalPose;
    double speed;
    base::geometry::Spline<3> posSpline;
    base::geometry::Spline<1> orientationSpline;
    
    /**The kind of this trajectory. This does not influece how the trajectory 
     * is followed. It is merely metadata. Some tasks may or may not behave differently
     * if the kind is not NORMAL. */
    SubTrajectoryKind kind;
    
    SubTrajectory();

    SubTrajectory(const base::Trajectory &trajectory);
    
    static double angleLimit(double angle);
    
    base::Trajectory toBaseTrajectory() const;

    /**
     * This method interpolates a point turn SubTrajectory
     * from a given set of angles.
     * */
    void interpolate(base::Pose2D start, const std::vector<base::Angle> &angles);

    /**
     * This method tries to interpolate from a set of poses.
     * For the direction of orientation change it always assumes
     * the shortest distance.
     *
     * */
    void interpolate(const std::vector<base::Pose2D> &poses);

    /**
     * This method interpolates a Spline from a set of poses.
     * For the direction of orientation a vector of orientation
     * differences is given. This enables us to encode things like
     * Pose(0,0,0) -> Pose(1,1,0) With a full turn inbetween.
     * */
    void interpolate(const std::vector< base::Pose2D >& poses, const std::vector< double >& orientationDiff);

    /**
     * Returns the Pose2D for param d;
     * The Orientation NOT be normalized.
     * This is helpful for the calculation of steering parameters.
     * */
    base::Pose2D getIntermediatePoint(double d);

    /**
     * Returns the param corresponding to the closest point
     * of the POSITION of the given pose.
     * This method may be given an initial guess, to speed
     * up the search.
     * */
    double getClosestPoint(const base::Pose2D &pose, double guess) const;

    /**
     * Returns the param corresponding to the closest point
     * of the POSITION of the given pose.
     * */
    double getClosestPoint(const base::Pose2D &pose) const;

    /**
     * Returns the Pose2D for param d;
     * The Orientation will be normalized to -M_PI >=< M_PI
     * */
    base::Pose2D getIntermediatePointNormalized(double d);

    /**
     * Returns the distance between the start and the endParam
     * in POSITION.
     * */
    double getDist(double startParam, double endParam) const;

    double getClosestPoint(const base::Pose2D &pose, double guess, double start, double end);
    void setGeometricResolution(double geometricResolution);
    double getDistToGoal(double startParam) const;
    void error(const Eigen::Vector2d& pos, double currentHeading, double curveParam, double &distanceError, double &headingError, double &splineHeadingError);
    double advance(double curveParam, double length);
    double getCurvature(double param);
    double getCurvatureMax();
    double getVariationOfCurvature(double param);
    const base::Pose2D &getStartPose() const;
    const base::Pose2D &getGoalPose() const;
    const double &getSpeed() const;
    double getStartParam() const;
    double getEndParam() const;
    double getGeometricResolution() const;
    bool driveForward() const;
    void setSpeed(double speed);
    double splineHeading(double param);
};

class Lateral : public SubTrajectory {
public:
    Lateral();
    Lateral(const base::Pose2D &currentPose, const base::Position2D &end, const double &speed);
    Lateral(const base::Pose2D &currentPose, const double &angle, const double &length, const double &speed);
};

class Ackermann : public SubTrajectory
{
public:
    Ackermann();
    Ackermann(const std::vector< base::Pose2D >& poses, const double &speed);
};

class PointTurn : public SubTrajectory
{
public:
    PointTurn();
    PointTurn(const base::Pose2D &pose, const std::vector<base::Angle> &angles, const double &speed);
};

}

#endif