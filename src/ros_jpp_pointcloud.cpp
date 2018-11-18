#include <ros/ros.h>
#include <jaus/core/Component.h>
#include <jaus/environment/range/RangeSensor.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros_jpp/point_types.h"

class JROS_PointCloud
{
  public:
    using VPoint = velodyne_pointcloud::PointXYZIR;
    using Point3D = JAUS::Point3D;
    using UShort = JAUS::UShort;
    using Component = JAUS::Component;
    using RangeSensor = JAUS::RangeSensor;

    JROS_PointCloud(const ros::NodeHandle& nh, const ros::NodeHandle& pn):
                    m_nh(nh),m_pn(pn),m_npts(1000)
    {
        m_component.AddService(&m_range_sensor);
        //m_component.LoadSettings("settings/jaus/services.xml");
        m_component.DiscoveryService()->SetSubsystemIdentification(
                JAUS::Subsystem::Vehicle,"ROS_Vehicle"
        );
        if(m_component.InitializeWithUniqueID() == false)
        {
          std::cout << "Failed to Initialize Component["
                    <<m_component.GetComponentID().ToString()<<"]\n";
        }
        else
        {
          std::cout << "Component ["
                    <<m_component.GetComponentID().ToString()
                    << "] Initialized!\n";
          if (!pn.getParam("max_pts", m_npts))
            ROS_ERROR("max_pts not set, using 1000 by default");
          if (m_npts < 0)
          {
            ROS_ERROR("max_pts = %d is not valid, using 1000 by default", m_npts);
            m_npts = 1000;
          }

          //subscribe to ROS sensor_msgs::PointCloud2 topic
          m_sub_pc2 = m_nh.subscribe("velodyne_points", 1
                            , &JROS_PointCloud::cbPC2Arrived, this);
        }
    }

    ///convert from ROS pointcloud msg to jaus range sensor msg
    static void fromROSPointCloud2(Point3D::List* scan
      , const sensor_msgs::PointCloud2ConstPtr& pc2, const size_t& k_max_pts)
    {
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*pc2,pcl_pc2);
      pcl::PointCloud<VPoint> pcl_pc;
      pcl::fromPCLPointCloud2(pcl_pc2,pcl_pc);
      const size_t raw_pts = pcl_pc.size();
      const size_t npts = raw_pts > k_max_pts ? k_max_pts : raw_pts;
      scan->reserve(npts);
      for (int i=0;i<npts;++i)
      {
          //only show pts of ring 29-35
          //if (abs(pcl_pc.points[i].ring - 32) > 10)
          //  continue;
          //if (i >= npts)
          //  break;
          scan->push_back(Point3D(
                pcl_pc.points[i].x,
                pcl_pc.points[i].y,
                pcl_pc.points[i].z
          ));
      }
      std::cout<<"sending "<<scan->size()<<" pts to OCU\n";
    }

  protected:
    virtual void cbPC2Arrived(const sensor_msgs::PointCloud2ConstPtr& pc2)
    {
        Point3D::List scan;
        //convert from ROS msg to JAUS msg
        fromROSPointCloud2(&scan, pc2, m_npts);
        //send to JAUS with sensor info
        UShort sensorID = 0;
        Point3D sensor_translation(0,0,0);
        Point3D sensor_orientation(0,0,0);
        m_range_sensor.SetCartesianRangeScan(sensorID
                                            , sensor_translation
                                            , sensor_orientation
                                            , scan);
    }
    ros::NodeHandle m_nh,m_pn;

  private:
    ros::Subscriber m_sub_pc2;
    Component m_component;
    RangeSensor m_range_sensor;
    int m_npts;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaus_ros_node");
  ros::NodeHandle nh, pn("~");
  JROS_PointCloud jp(nh, pn);
  ros::spin();
  return 0;
}
