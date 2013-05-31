#ifndef PCLPYRAMID_HPP_INCLUDED
#define PCLPYRAMID_HPP_INCLUDED

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#define PLANE_THRESHOLD .05f
#define LINE_THRESHOLD .05f
#define MAX_LINES 12
#define MIN_CLOUD_POINTS 100
#define VOXEL_SIZE .01f
#define MAX_ITERATIONS 1000
#define MAX_Z_DEPTH 4.0f
#define FRAMES_PER_SEC 1

class PclPyramid
{
	public:
		typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
		typedef typename Cloud::Ptr CloudPtr;
		typedef typename Cloud::ConstPtr CloudConstPtr;

		PclPyramid (const std::string& device_id = "");
		virtual ~PclPyramid () { };

		void
		keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*);

		void
		cloudCallback (const CloudConstPtr& cloud);

		void
		set (const CloudConstPtr& cloud);

		CloudPtr
		get ();

		void
		run ();

		int
		openFile (std::string file);

		pcl::visualization::CloudViewer viewer_;
		pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid_;
		pcl::SACSegmentation<pcl::PointXYZRGBA> plane_seg_;
		pcl::SACSegmentation<pcl::PointXYZRGBA> line_seg_;
		pcl::ExtractIndices<pcl::PointXYZRGBA> plane_extract_;
		pcl::ExtractIndices<pcl::PointXYZRGBA> line_extract_;
		std::string device_id_;
		boost::mutex mtx_;
		CloudConstPtr cloud_;
		bool save_cloud_; 
		unsigned int toggle_view_;
		unsigned int files_saved_;
};

#endif
