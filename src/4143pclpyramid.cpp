#include <4143pclpyramid.h>
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/time.h>

using namespace pcl;
using namespace std;

PclPyramid::PclPyramid (const string& device_id) :
  viewer_ ("4143 pcl pyramid"),
  device_id_ (device_id),
  save_cloud_ (false),
  toggle_view_ (0),
  files_saved_ (0)
{
  voxel_grid_.setFilterFieldName ("z");
  voxel_grid_.setFilterLimits (0.0f, MAX_Z_DEPTH); // filter anything past 3 meters
  voxel_grid_.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE); // filter to 1 cm

  plane_seg_.setOptimizeCoefficients (true);
  plane_seg_.setModelType (SACMODEL_PLANE);
  plane_seg_.setMethodType (SAC_RANSAC);
  plane_seg_.setMaxIterations (MAX_ITERATIONS);
  plane_seg_.setDistanceThreshold (PLANE_THRESHOLD);

  line_seg_.setOptimizeCoefficients (true);
  line_seg_.setModelType (SACMODEL_LINE);
  line_seg_.setMethodType (SAC_RANSAC);
  line_seg_.setMaxIterations (MAX_ITERATIONS);
  line_seg_.setDistanceThreshold (LINE_THRESHOLD); // find line within ...

  viewer_.registerKeyboardCallback (&PclPyramid::keyboard_callback, *this , 0);
}

void
PclPyramid::keyboard_callback (const visualization::KeyboardEvent& event, void *)
{
  if (event.keyUp ())
  {
    switch (event.getKeyCode ())
    {
    case 's':
    case 'S':
      save_cloud_ = true; // save pcd file
      break;
    case 't':
    case 'T':
      ++toggle_view_  %= 2; 
      break;
    }
  }
}

void
PclPyramid::cloud_cb (const CloudConstPtr& cloud)
{
  static double last = getTime ();
  double now = getTime ();
  if (now >= last + (1 / FRAMES_PER_SEC))
  {
    set (cloud);
    last = now;
  }
}

void
PclPyramid::set (const CloudConstPtr& cloud)
{
  //lock while we set our cloud;
  boost::mutex::scoped_lock lock (mtx_);
  cloud_ = cloud;
}

PclPyramid::CloudPtr
PclPyramid::get ()
{
  //lock while we swap our cloud and reset it.
  boost::mutex::scoped_lock lock (mtx_);
  CloudPtr temp_cloud (new Cloud);
  CloudPtr temp_cloud2 (new Cloud);
  CloudPtr temp_cloud3 (new Cloud);
  CloudPtr temp_cloud4 (new Cloud);
  CloudPtr temp_cloud5 (new Cloud);
  CloudConstPtr empty_cloud;

  cerr << "cloud size orig: " << cloud_->size () << endl;
  voxel_grid_.setInputCloud (cloud_);
  voxel_grid_.filter (*temp_cloud);  // filter cloud for z depth

  cerr << "cloud size postzfilter: " << temp_cloud->size () << endl;

  ModelCoefficients::Ptr planecoefficients (new ModelCoefficients ());
  PointIndices::Ptr plane_inliers (new PointIndices ());
  vector<ModelCoefficients> linecoefficients1;
  ModelCoefficients model;
  model.values.resize (6);
  PointIndices::Ptr line_inliers (new PointIndices ());
  if (temp_cloud->size () > MIN_CLOUD_POINTS)
  {
    plane_seg_.setInputCloud (temp_cloud);
    plane_seg_.segment (*plane_inliers, *planecoefficients); // find plane
  }

  cerr << "plane inliers size: " << plane_inliers->indices.size () << endl;

  cerr << "planecoeffs: " 
    << planecoefficients->values[0]  << " "
    << planecoefficients->values[1]  << " "
    << planecoefficients->values[2]  << " "
    << planecoefficients->values[3]  << " "
    << endl;

  plane_extract_.setNegative (true); 
  plane_extract_.setInputCloud (temp_cloud);
  plane_extract_.setIndices (plane_inliers);
  plane_extract_.filter (*temp_cloud2);   // remove plane
  plane_extract_.setNegative (false); 
  plane_extract_.filter (*temp_cloud5);   // only plane

  for (size_t i = 0; i < temp_cloud5->size (); ++i)
  {
    temp_cloud5->points[i].r = 0; 
    temp_cloud5->points[i].g = 255; 
    temp_cloud5->points[i].b = 0; 
    // tint found plane green for ground
  }

  for (size_t j = 0 ; j < MAX_LINES && temp_cloud2->size () > MIN_CLOUD_POINTS; j++)
  {
    // look for x lines until cloud gets too small
    cerr << "cloud size: " << temp_cloud2->size () << endl;

    line_seg_.setInputCloud (temp_cloud2);
    line_seg_.segment (*line_inliers, model); // find line

    cerr << "line inliears size: " << line_inliers->indices.size () << endl;

    if (line_inliers->indices.size () < MIN_CLOUD_POINTS)
      break;

    linecoefficients1.push_back (model);  // store line coeffs

    line_extract_.setNegative (true); 
    line_extract_.setInputCloud (temp_cloud2);
    line_extract_.setIndices (line_inliers);
    line_extract_.filter (*temp_cloud3);  // remove plane
    line_extract_.setNegative (false); 
    line_extract_.filter (*temp_cloud4);  // only plane
    for (size_t i = 0; i < temp_cloud4->size (); ++i)
    {
      temp_cloud4->points[i].g = 0; 
      if (j % 2)
      {
        temp_cloud4->points[i].r = 255 - j * int (255/MAX_LINES); 
        temp_cloud4->points[i].b = 0 + j * int (255/MAX_LINES); 
      }
      else
      {
        temp_cloud4->points[i].b = 255 - j * int (255/MAX_LINES); 
        temp_cloud4->points[i].r = 0 + j * int (255/MAX_LINES); 
      }
    }
    *temp_cloud5 += *temp_cloud4;  // add line to ground
  
    temp_cloud2.swap (temp_cloud3); // remove line

  }

  for (size_t i = 0; i < linecoefficients1.size (); i++)
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

  if (save_cloud_)
  {
    stringstream stream;
    stream << "inputCloud" << files_saved_ << ".pcd";
    string filename = stream.str ();
    if (io::savePCDFile (filename, *temp_cloud, true) == 0)
    {
      files_saved_++;
      cout << "Saved " << filename << "." << endl;
    }
    else
      PCL_ERROR ("Problem saving %s.\n", filename.c_str ());
    
    save_cloud_ = false;
  }

  empty_cloud.swap (cloud_);  // set cloud_ to null

  if (toggle_view_ == 1) 
    return (temp_cloud);  // return orig cloud
  else
    return (temp_cloud5); // return colored cloud
}

void
PclPyramid::run ()
{
  Grabber* interface = new OpenNIGrabber (device_id_);

  boost::function<void (const CloudConstPtr&)> f = boost::bind (&PclPyramid::cloud_cb, this, _1);
  boost::signals2::connection c = interface->registerCallback (f);
  
  interface->start ();
  
  while (!viewer_.wasStopped ())
  {
    if (cloud_)
    {
      //the call to get () sets the cloud_ to null;
      viewer_.showCloud (get ());
    }
    //viewer_.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (10000));
  }

  interface->stop ();
}

void
usage (char ** argv)
{
  cout << "usage: " << argv[0] << " <device_id> <options>\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0) {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
        << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
        << "         bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
        << "         <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int
main (int argc, char ** argv)
{
  string arg;

  if (argv[1] == NULL)
    arg+="#1";
  else
    arg+=argv[1];
  
  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return (1);
  }

  //double threshold = 0.05;
  //console::parse_argument (argc, argv, "-thresh", threshold);

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () == 0)
  {
    cout << "No devices connected." << endl;
    return (1);
  }

  OpenNIGrabber grabber (arg);
  if (grabber.providesCallback<OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PclPyramid v (arg);
    v.run ();
  }
  else
  {
    cerr << "not color device" << endl;
    return (1);
  }

  return (0);
}
