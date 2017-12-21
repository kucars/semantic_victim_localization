#ifndef VICTIM_LOCALIZATION_COMMON_H
#define VICTIM_LOCALIZATION_COMMON_H

#include <ros/ros.h>
#include <pcl/point_types.h>

#include "utilities/console_utility.h"
#include "utilities/pose_conversion.h"
#include "utilities/time_profiler.h"

// include input and output archivers for serialization
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <iostream>
#include <string>
#include <vector>

static ConsoleUtility cc;
extern TimeProfiler timer;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointN> PointCloudN;

#endif // VICTIM_LOCALIZATION_COMMON_H
