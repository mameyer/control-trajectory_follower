#include <iostream>
#include "SubTrajectoryVisualization.hpp"
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/OsgViz.hpp>

using namespace vizkit3d;

struct SubTrajectoryVisualization::Data {
    std::vector<trajectory_follower::SubTrajectory> data;
};


SubTrajectoryVisualization::SubTrajectoryVisualization()
    : p(new Data)
{
}

SubTrajectoryVisualization::~SubTrajectoryVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> SubTrajectoryVisualization::createMainNode()
{
    return new osg::PositionAttitudeTransform();
}

void SubTrajectoryVisualization::updateMainNode ( osg::Node* node )
{
    osg::PositionAttitudeTransform* geode = static_cast<osg::PositionAttitudeTransform*>(node);
    geode->removeChildren(0, geode->getNumChildren());

    osgviz::PrimitivesFactory* fac = osgviz::OsgViz::getInstance()->getModuleInstance<osgviz::PrimitivesFactory>("PrimitivesFactory");
    std::vector<osg::Vec3> osgPoints;
    const osg::Vec4 arColor(1, 0, 1, 1);
    
    for (trajectory_follower::SubTrajectory& traj : p->data)
    {   
        double stepSize = (traj.getEndParam() - traj.getStartParam()) / (traj.posSpline.getCurveLength() / 0.05);
        double pos = 0;
        const double oriStep = 5 * stepSize;
        
        for (double param = traj.getStartParam(); param <= traj.getEndParam(); param += stepSize)
        {
            pos += stepSize;
            
            base::Pose2D splinePoint = traj.getIntermediatePoint(param);
            osgPoints.emplace_back(splinePoint.position.x(), splinePoint.position.y(), 0);
            
            if (pos > oriStep)
            {
                double ori = splinePoint.orientation;
                
                auto ar = fac->createArrow(arColor, true);
                ar.get()->setPosition(splinePoint.position.x(), splinePoint.position.y(), 0);
                ar.get()->setAttitude(osg::Quat(M_PI/2, osg::Vec3d(1,0,0)));
                ar.get()->rotate(ori + M_PI /2, osg::Vec3d(0,0,1));
                
                if (!traj.orientationSpline.isEmpty())
                {
                    ar.get()->rotate(traj.splineHeading(param), osg::Vec3d(0,0,1));
                }
                
                geode->addChild(ar);
                
                pos = 0;
            }
        }
    }
    
    const osg::Vec4 color(1, 1, 0, 1);
    auto prim = fac->createLinesNode(color, osgPoints);
    geode->addChild(prim);
}

void SubTrajectoryVisualization::updateDataIntern(std::vector<trajectory_follower::SubTrajectory> const& value)
{
    p->data = value;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SubTrajectoryVisualization)

