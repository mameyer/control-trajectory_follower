#pragma once

#include <base/commands/Motion2D.hpp>

namespace trajectory_follower
{

class Controller {
public:
    Controller()
        : configured(false)
    {
    }
    
    virtual ~Controller();

    virtual base::commands::Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature) =0;
    virtual void reset() =0;

protected:
    bool configured;
    base::commands::Motion2D motionCommand;
};
}
