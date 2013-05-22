#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


template <typename PointType>
class PclPyramid
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    PclPyramid (const std::string& device_id = "", double threshold = 0.01)
      : viewer ("4143 pcl pyramid"),
        device_id_ (device_id)
    {
     grid_.setFilterFieldName ("z");
     grid_.setFilterLimits (0.0f, 3.0f); // filter anything past 3 meters
     grid_.setLeafSize (0.01f, 0.01f, 0.01f); // filter to 1 cm

      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PLANE); //rjs
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setMaxIterations (1000);
      seg_.setDistanceThreshold (threshold);

      extract_.setNegative (true);  // remove plane

      seg1_.setOptimizeCoefficients (true);
      seg1_.setModelType (pcl::SACMODEL_LINE); //rjs
      seg1_.setMethodType (pcl::SAC_RANSAC);
      seg1_.setMaxIterations (1000);
      seg1_.setDistanceThreshold (.1); // find line within ...

      extract1_.setNegative (true); // remove line
    }

    void 
    cloud_cb_ (const CloudConstPtr& cloud)
    {
      set (cloud);
    }

    void
    set (const CloudConstPtr& cloud)
    {
      //lock while we set our cloud;
      boost::mutex::scoped_lock lock (mtx_);
      cloud_  = cloud;
    }

    CloudPtr
    get ()
    {
      //lock while we swap our cloud and reset it.
      boost::mutex::scoped_lock lock (mtx_);
      CloudPtr temp_cloud (new Cloud);
      CloudPtr temp_cloud2 (new Cloud);
      CloudPtr temp_cloud3 (new Cloud);

      grid_.setInputCloud (cloud_);
      grid_.filter (*temp_cloud);

      pcl::ModelCoefficients::Ptr planecoefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());
      std::vector<pcl::ModelCoefficients> linecoefficients1;
      pcl::ModelCoefficients model;
      model.values.resize (4);
      pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());

      seg_.setInputCloud (temp_cloud);
      seg_.segment (*plane_inliers, *planecoefficients);

      for(size_t i = 0; i < plane_inliers->indices.size (); ++i)
	temp_cloud->points[plane_inliers->indices[i]].g = 255; 
		// tint found plane green for ground

      extract_.setInputCloud (temp_cloud);
      extract_.setIndices (plane_inliers);
      extract_.filter (*temp_cloud2);


	for(int j = 0 ; j < 10 && temp_cloud2->size() > 10; j++) // look for 10 lines until cloud gets too small
	{

      		seg1_.setInputCloud (temp_cloud2);
      		seg1_.segment (*line_inliers, model);
      		linecoefficients1.push_back (model);

      		for(size_t i = 0; i < line_inliers->indices.size (); ++i)
			temp_cloud->points[ line_inliers->indices[i]].r = 255; 
				// tint found line red

      		extract1_.setInputCloud (temp_cloud2);
      		extract1_.setIndices (line_inliers);
      		extract1_.filter (*temp_cloud3);
      		temp_cloud2 = temp_cloud3;

	}

	for(size_t i = 0; i < linecoefficients1.size(); i++)
	{
		cerr << "linecoeffs: " << i  << " "
		<< linecoefficients1[i].values[0]  << " "
		<< linecoefficients1[i].values[1]  << " "
		<< linecoefficients1[i].values[2]  << " "
		<< linecoefficients1[i].values[3]  << " "
		<< endl;
	}


      return (temp_cloud);  // return colored cloud
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&PclPyramid::cloud_cb_, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);
      
      interface->start ();
      
      while (!viewer.wasStopped ())
      {
        if (cloud_)
        {
          //the call to get() sets the cloud_ to null;
          viewer.showCloud (get ());
        }
      }

      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
    pcl::VoxelGrid<PointType> grid_;
    pcl::SACSegmentation<PointType> seg_;
    pcl::SACSegmentation<PointType> seg1_;
    pcl::ExtractIndices<PointType> extract_;
    pcl::ExtractIndices<PointType> extract1_;

    std::string device_id_;
    boost::mutex mtx_;
    CloudConstPtr cloud_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int 
main (int argc, char ** argv)
{
  std::string arg;

  if(argv[1] == NULL) arg+="#1";
  else arg+=argv[1];
  
  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  double threshold = 0.05;
  //pcl::console::parse_argument (argc, argv, "-thresh", threshold);

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () == 0) 
  {
    cout << "No devices connected." << endl;
    return 1;
  }

  pcl::OpenNIGrabber grabber (arg);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PclPyramid<pcl::PointXYZRGBA> v (arg, threshold);
    v.run ();
  }
  else
  {
	cerr << "not color device" << endl;
	return 1;
  }

  return (0);
}
