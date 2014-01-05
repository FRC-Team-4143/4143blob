#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>

#ifndef NOVIEWER
#include <pcl/visualization/cloud_viewer.h>
#endif

#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <4143blob.h>
#include <pcl/common/time.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/conversions.h>


class PclBlob
{
 public:
  typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  PclBlob (const std::string& device_id = "", const std::string& filename = "" ) :
#ifndef NOVIEWER
      viewer ("4143 pcl blob"),
#endif
      device_id_ (device_id) , filename_ (filename)
      {
        voxel_grid.setFilterFieldName ("z");
        voxel_grid.setFilterLimits (0.0f, MAX_Z_DEPTH); // filter anything past 3 meters
        voxel_grid.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE); // filter to 1 cm

        plane_seg.setOptimizeCoefficients (true);
        plane_seg.setModelType (pcl::SACMODEL_PLANE);
        plane_seg.setMethodType (pcl::SAC_RANSAC);
        plane_seg.setMaxIterations (MAX_ITERATIONS);
        plane_seg.setDistanceThreshold (PLANE_THRESHOLD);

        line_seg.setOptimizeCoefficients (true);
        line_seg.setModelType (pcl::SACMODEL_LINE);
        line_seg.setMethodType (pcl::SAC_RANSAC);
        line_seg.setMaxIterations (MAX_ITERATIONS);
        line_seg.setDistanceThreshold (LINE_THRESHOLD); // find line within ...

#ifndef NOVIEWER
        viewer.registerKeyboardCallback(&PclBlob::keyboard_callback, *this , 0);
#endif
        saveCloud = false;
        toggleView = 0;
        filesSaved = 0;
      }

  void
      keyboard_callback (const pcl::visualization::KeyboardEvent& event, void *)
      {
        if (event.keyUp ())
        {
          switch (event.getKeyCode())
          {
            case 's':
            case 'S':
              saveCloud = true; // save pcd file
              break;
            case 't':
            case 'T':
              ++toggleView  %= 2; 
              break;
          }
        }	
      }

  void 
      cloud_cb_ (const CloudConstPtr& cloud)
      {
        static double last = pcl::getTime();
        double now = pcl::getTime();
        if(now >= last + (1/FRAMES_PER_SEC)) {
          set (cloud);
          last = now;
        }
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
        CloudPtr temp_cloud4 (new Cloud);
        CloudPtr temp_cloud5 (new Cloud);
        CloudConstPtr empty_cloud;


        cout << "===============================\n"
                "======Start of frame===========\n"
                "===============================\n";
        //cerr << "cloud size orig: " << cloud_->size() << endl;
        voxel_grid.setInputCloud (cloud_);
        voxel_grid.filter (*temp_cloud);  // filter cloud for z depth

        //cerr << "cloud size postzfilter: " << temp_cloud->size() << endl;

        pcl::ModelCoefficients::Ptr planecoefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());
        std::vector<pcl::ModelCoefficients> linecoefficients1;
        pcl::ModelCoefficients model;
        model.values.resize (6);
        pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());
        std::vector<Eigen::Vector3f> corners;

        if(temp_cloud->size() > MIN_CLOUD_POINTS) {
          plane_seg.setInputCloud (temp_cloud);
          plane_seg.segment (*plane_inliers, *planecoefficients); // find plane
        }

        //cerr << "plane inliers size: " << plane_inliers->indices.size() << endl;

        cout << "planecoeffs: " 
            << planecoefficients->values[0]  << " "
            << planecoefficients->values[1]  << " "
            << planecoefficients->values[2]  << " "
            << planecoefficients->values[3]  << " "
            << endl;

        Eigen::Vector3f pn = Eigen::Vector3f(
            planecoefficients->values[0],
            planecoefficients->values[1],
            planecoefficients->values[2]);

        float planedeg = pcl::rad2deg(acos(pn.dot(Eigen::Vector3f::UnitZ())));
        cout << "angle of camera to floor normal: " << planedeg << " degrees" << endl;
        cout << "distance of camera to floor: " << planecoefficients->values[3]
            << " meters" <<  endl;

        plane_extract.setNegative (true); 
        plane_extract.setInputCloud (temp_cloud);
        plane_extract.setIndices (plane_inliers);
        plane_extract.filter (*temp_cloud2);   // remove plane
        plane_extract.setNegative (false); 
        plane_extract.filter (*temp_cloud5);   // only plane

        for(size_t i = 0; i < temp_cloud5->size (); ++i)
        {
          temp_cloud5->points[i].r = 0; 
          temp_cloud5->points[i].g = 255; 
          temp_cloud5->points[i].b = 0; 
          // tint found plane green for ground
        }

// Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud (temp_cloud2);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (temp_cloud2);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    CloudPtr cloud_cluster (new Cloud);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (temp_cloud2->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    j++;
  }

/* this is the old pclpyramid code
        for(size_t j = 0 ; j < MAX_LINES && temp_cloud2->size() > MIN_CLOUD_POINTS; j++) 
          // look for x lines until cloud gets too small
        {
//          cerr << "cloud size: " << temp_cloud2->size() << endl;

          line_seg.setInputCloud (temp_cloud2);
          line_seg.segment (*line_inliers, model); // find line

 //         cerr << "line inliears size: " << line_inliers->indices.size() << endl;

          if(line_inliers->indices.size() < MIN_CLOUD_POINTS)
            break;

          linecoefficients1.push_back (model);  // store line coeffs

          line_extract.setNegative (true); 
          line_extract.setInputCloud (temp_cloud2);
          line_extract.setIndices (line_inliers);
          line_extract.filter (*temp_cloud3);  // remove plane
          line_extract.setNegative (false); 
          line_extract.filter (*temp_cloud4);  // only plane
          for(size_t i = 0; i < temp_cloud4->size (); ++i) {
            temp_cloud4->points[i].g = 0; 
            if(j%2) {
              temp_cloud4->points[i].r = 255-j*int(255/MAX_LINES); 
              temp_cloud4->points[i].b = 0+j*int(255/MAX_LINES); 
            } else {
              temp_cloud4->points[i].b = 255-j*int(255/MAX_LINES); 
              temp_cloud4->points[i].r = 0+j*int(255/MAX_LINES); 
            }
          }
          *temp_cloud5 += *temp_cloud4;	// add line to ground

          temp_cloud2.swap ( temp_cloud3); // remove line

        }

        cout << "found " << linecoefficients1.size() << " lines." << endl;

        for(size_t i = 0; i < linecoefficients1.size(); i++)
        {
          //cerr << "linecoeffs: " << i  << " "
          //    << linecoefficients1[i].values[0]  << " "
          //    << linecoefficients1[i].values[1]  << " "
          //    << linecoefficients1[i].values[2]  << " "
          //    << linecoefficients1[i].values[3]  << " "
          //    << linecoefficients1[i].values[4]  << " "
          //    << linecoefficients1[i].values[5]  << " "
          //    << endl;

          Eigen::Vector3f lv = Eigen::Vector3f(
              linecoefficients1[i].values[3],
              linecoefficients1[i].values[4],
              linecoefficients1[i].values[5]); 

          Eigen::Vector3f lp = Eigen::Vector3f(
              linecoefficients1[i].values[0],
              linecoefficients1[i].values[1],
              linecoefficients1[i].values[2]); 

          float r = pn.dot(lv);
          float deg = pcl::rad2deg(acos(r));

          cout << "angle of line to floor normal: " << deg << " degrees" << endl;

          if(abs(deg-30) < 5 || abs(deg-150) < 5) 
          {
            cout << "found corner line" << endl;

            float t = -(lp.dot(pn) + planecoefficients->values[3])/ r;
            Eigen::Vector3f intersect = lp + lv*t;
            cout << "corner intersects floor at: " << endl << intersect << endl;
            cout << "straight line distance from camera to corner: " <<
                intersect.norm() << " meters" << endl;

            corners.push_back(intersect);

            Eigen::Vector3f floor_distance = intersect + pn;  // should be - ???

            cout << "distance along floor to corner: " <<
                floor_distance.norm() << " meters" << endl;
          }
          else if(abs(deg-90) < 5) 
          {
            cout << "found horizontal line" << endl;
          }

        }

        switch(corners.size())
        {
          case 2:
            cout << "distance between corners " << (corners[0] - corners[1]).norm() << endl;
            cout << "angle of blob to camera " <<
                pcl::rad2deg(acos(((corners[0] - corners[1]).normalized()).dot(Eigen::Vector3f::UnitX())))
                << endl;

            break;

          case 3:
            cout << "distance between corners " << (corners[0] - corners[1]).norm() << endl;
            cout << "distance between corners " << (corners[0] - corners[2]).norm() << endl;
            cout << "distance between corners " << (corners[1] - corners[2]).norm() << endl;

            cout << "angle of corner on floor (should be 90) " <<
                pcl::rad2deg(acos((corners[0] - corners[1]).dot(corners[0] - corners [2])))
                << endl;
            
            cout << "angle of blob to camera " <<
                pcl::rad2deg(acos(((corners[0] - corners[1]).normalized()).dot(Eigen::Vector3f::UnitX())))
                << endl;

            break;
        }
*/

        if (saveCloud)
        {
          std::stringstream stream, stream1;
          std::string filename;

          stream << "inputCloud" << filesSaved << ".pcd";
          filename = stream.str();
          if (pcl::io::savePCDFile(filename, *cloud_, true) == 0)
          {
            filesSaved++;
            cout << "Saved " << filename << "." << endl;
          }
          else PCL_ERROR("Problem saving %s.\n", filename.c_str());

          
          stream1 << "inputCloud" << filesSaved << ".pcd";
          filename = stream1.str();
          if (pcl::io::savePCDFile(filename, *temp_cloud5, true) == 0)
          {
            filesSaved++;
            cout << "Saved " << filename << "." << endl;
          }
          else PCL_ERROR("Problem saving %s.\n", filename.c_str());

          saveCloud = false;
        }

        empty_cloud.swap(cloud_);  // set cloud_ to null

        if(toggleView == 1) 
          return (temp_cloud);  // return orig cloud
        else
          return (temp_cloud5); // return colored cloud
      }

  void
      run ()
      {
        CloudPtr filecloud;
        pcl::Grabber* interface;
        if(filename_.empty()) {
          interface = new pcl::OpenNIGrabber (device_id_);

          boost::function<void (const CloudConstPtr&)> f = boost::bind (&PclBlob::cloud_cb_, this, _1);
          boost::signals2::connection c = interface->registerCallback (f);

          interface->start ();
        }
        else 
        {
          pcd_cloud.reset (new pcl::PCLPointCloud2);
          if(pcd.read (filename_, *pcd_cloud, origin, orientation, version) < 0)
            cout << "file read failed" << endl;
          filecloud.reset (new Cloud);
          pcl::fromPCLPointCloud2(*pcd_cloud, *filecloud);
          cloud_  = filecloud;
        }


#ifdef NOVIEWER        
        if(!filename_.empty()) 
          get();
        else 
          while(TRUE)
          {
            if (cloud_)
              get();
            boost::this_thread::sleep (boost::posix_time::microseconds (10000));
          }
#else
        while ( !viewer.wasStopped () )
        {
          if (cloud_)
          {
            //the call to get() sets the cloud_ to null;
            viewer.showCloud (get ());
          }
          boost::this_thread::sleep (boost::posix_time::microseconds (10000));
        }
#endif

        if(filename_.empty()) {
          interface->stop ();
        }
      }

#ifndef NOVIEWER
  pcl::visualization::CloudViewer viewer;
#endif
  pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
  pcl::SACSegmentation<pcl::PointXYZRGBA> plane_seg;
  pcl::SACSegmentation<pcl::PointXYZRGBA> line_seg;
  pcl::ExtractIndices<pcl::PointXYZRGBA> plane_extract;
  pcl::ExtractIndices<pcl::PointXYZRGBA> line_extract;

  pcl::PCDReader pcd;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version; 
  pcl::PCLPointCloud2::Ptr pcd_cloud;

  std::string device_id_;
  std::string filename_;
  boost::mutex mtx_;
  CloudConstPtr cloud_;
  bool saveCloud; 
  unsigned int toggleView;
  unsigned int filesSaved;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <-device device_id> <-file filename.pcd>\n\n";

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
  std::string filename;
  std::string device;

  if ( pcl::console::find_switch(argc, argv, "--help") || 
      pcl::console::find_switch(argc, argv, "-h"))
  {
    usage (argv);
    return 1;
  }

  //double threshold = 0.05;
  //pcl::console::parse_argument (argc, argv, "-thresh", threshold);

  pcl::console::parse_argument (argc, argv, "-file", filename);

  if(filename.empty() == FALSE)
    cout << "filename: " << filename << endl;

  pcl::console::parse_argument (argc, argv, "-device", device);

  if(device.empty() == TRUE)
    device += "#1";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () == 0 && filename.empty()) 
  {
    cout << "No devices connected. No filename." << endl;
    return 1;
  }

  PclBlob v (device, filename);
  v.run ();

  return (0);
}
