#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <4143pclpyramid.h>


class PclPyramid
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    PclPyramid (const std::string& device_id = "")
      : viewer ("4143 pcl pyramid"),
        device_id_ (device_id)
    {
     voxel_grid.setFilterFieldName ("z");
     voxel_grid.setFilterLimits (0.0f, MAX_Z_DEPTH); // filter anything past 3 meters
     voxel_grid.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE); // filter to 1 cm

      plane_seg.setOptimizeCoefficients (true);
      plane_seg.setModelType (pcl::SACMODEL_PLANE);
      plane_seg.setMethodType (pcl::SAC_RANSAC);
      plane_seg.setMaxIterations (MAX_ITERATIONS);
      plane_seg.setDistanceThreshold (PLANE_THRESHOLD);

      plane_extract.setNegative (true);  // remove plane

      line_seg.setOptimizeCoefficients (true);
      line_seg.setModelType (pcl::SACMODEL_LINE);
      line_seg.setMethodType (pcl::SAC_RANSAC);
      line_seg.setMaxIterations (MAX_ITERATIONS);
      line_seg.setDistanceThreshold (LINE_THRESHOLD); // find line within ...

      line_extract.setNegative (true); // remove line

      viewer.registerKeyboardCallback(&PclPyramid::keyboard_callback, *this , 0);
      saveCloud = false;
      filesSaved = 0;
    }

    void
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void *)
    {
	cerr << "keyboard_callback" << endl;
	if (event.keyUp ())
	{
		switch (event.getKeyCode())
		{
			case 's':
			case 'S':
				saveCloud = true;
				break;
			case 'v':
			case 'V':
				// do something else
				break;
		}
	}	
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

      voxel_grid.setInputCloud (cloud_);
      voxel_grid.filter (*temp_cloud);  // filter cloud for z depth

      pcl::ModelCoefficients::Ptr planecoefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());
      std::vector<pcl::ModelCoefficients> linecoefficients1;
      pcl::ModelCoefficients model;
      model.values.resize (6);
      pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());

      plane_seg.setInputCloud (temp_cloud);
      plane_seg.segment (*plane_inliers, *planecoefficients); // find plane

      for(size_t i = 0; i < plane_inliers->indices.size (); ++i)
	temp_cloud->points[plane_inliers->indices[i]].g = 255; 
		// tint found plane green for ground

      cerr << "planecoeffs: " 
	<< planecoefficients->values[0]  << " "
	<< planecoefficients->values[1]  << " "
	<< planecoefficients->values[2]  << " "
	<< planecoefficients->values[3]  << " "
	<< endl;

      plane_extract.setInputCloud (temp_cloud);
      plane_extract.setIndices (plane_inliers);
      plane_extract.filter (*temp_cloud2);   // remove plane


	for(size_t j = 0 ; j < MAX_LINES && temp_cloud2->size() > MIN_CLOUD_POINTS; j++) 
		// look for x lines until cloud gets too small
	{

      		line_seg.setInputCloud (temp_cloud2);
      		line_seg.segment (*line_inliers, model); // find line
      		linecoefficients1.push_back (model);  // store line coeffs

      		for(size_t i = 0; i < line_inliers->indices.size (); ++i)
			temp_cloud->points[ line_inliers->indices[i]].r = 255; 
				// tint found line red

      		line_extract.setInputCloud (temp_cloud2);
      		line_extract.setIndices (line_inliers);
      		line_extract.filter (*temp_cloud3);
      		temp_cloud2 = temp_cloud3;	// remove line 

	}

	for(size_t i = 0; i < linecoefficients1.size(); i++)
	{
		cerr << "linecoeffs: " << i  << " "
		<< linecoefficients1[i].values[0]  << " "
		<< linecoefficients1[i].values[1]  << " "
		<< linecoefficients1[i].values[2]  << " "
		<< linecoefficients1[i].values[3]  << " "
		<< linecoefficients1[i].values[4]  << " "
		<< linecoefficients1[i].values[5]  << " "
		<< endl;
	}

      if (saveCloud)
	{
        std::stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        std::string filename = stream.str();
        if (pcl::io::savePCDFile(filename, *temp_cloud, true) == 0)
        {
            filesSaved++;
            cout << "Saved " << filename << "." << endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());
        
        saveCloud = false;
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
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
    pcl::SACSegmentation<pcl::PointXYZRGBA> plane_seg;
    pcl::SACSegmentation<pcl::PointXYZRGBA> line_seg;
    pcl::ExtractIndices<pcl::PointXYZRGBA> plane_extract;
    pcl::ExtractIndices<pcl::PointXYZRGBA> line_extract;

    std::string device_id_;
    boost::mutex mtx_;
    CloudConstPtr cloud_;
    bool saveCloud;
    unsigned int filesSaved;
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

  //double threshold = 0.05;
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
    PclPyramid v (arg);
    v.run ();
  }
  else
  {
	cerr << "not color device" << endl;
	return 1;
  }

  return (0);
}
