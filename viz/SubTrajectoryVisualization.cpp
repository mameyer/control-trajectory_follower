#include <iostream>
#include "SubTrajectoryVisualization.hpp"
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/OsgViz.hpp>

#include <osg/LineWidth>

using namespace vizkit3d;

struct SubTrajectoryVisualization::Data {
    std::vector<trajectory_follower::SubTrajectory> data;
};


SubTrajectoryVisualization::SubTrajectoryVisualization()
    : p(new Data), line_width( 4.0 ), color(1., 1., 0., 1.), rescueColor(1., 0., 0., 1.)
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
    geode = static_cast<osg::PositionAttitudeTransform*>(node);
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
        
        osg::Vec4 currentColor = traj.kind == trajectory_follower::TRAJECTORY_KIND_RESCUE? rescueColor : color;
        
        auto prim = fac->createLinesNode(color, osgPoints);
        geode->addChild(prim);
        
    }
    
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(line_width);
    stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);    
}

void SubTrajectoryVisualization::updateDataIntern(std::vector<trajectory_follower::SubTrajectory> const& value)
{
    p->data = value;
}

void SubTrajectoryVisualization::setColor(QColor color)
{
    this->color = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("Color");
    setDirty();
}

void SubTrajectoryVisualization::setRescueColor(QColor color)
{
    this->rescueColor = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("RescueColor");
    setDirty();
}

QColor SubTrajectoryVisualization::getColor() const
{
    QColor c;
    c.setRgbF(color.x(), color.y(), color.z(), color.w());
    return c;
}

QColor SubTrajectoryVisualization::getRescueColor() const
{
    QColor c;
    c.setRgbF(rescueColor.x(), rescueColor.y(), rescueColor.z(), rescueColor.w());
    return c;
}

double SubTrajectoryVisualization::getLineWidth()
{
    return line_width;
}

void SubTrajectoryVisualization::setLineWidth(double line_width)
{
    this->line_width = line_width;
    if(geode)
    {
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        osg::LineWidth* linewidth = new osg::LineWidth();
        linewidth->setWidth(line_width);
        stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    }
    emit propertyChanged("LineWidth");
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SubTrajectoryVisualization)