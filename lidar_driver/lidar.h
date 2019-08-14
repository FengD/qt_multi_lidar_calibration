/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#ifndef _LIDAR_H_
#define _LIDAR_H_

#include <string.h>
#include <pthread.h>
#include "lidar_driver/driver.h"

class Lidar {
public:
  Lidar(std::string deviceIp, int dataPort, std::string lidarModel, int mode,
        std::string correctionFilePath, std::string deviceName,
        boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp, std::string deviceName)> lidarCallback);

	~Lidar();

	void start();

	void stop();

private:
	// lidar driver
  lidar_driver::Driver *lidarDriver;
};

#endif // _LIDAR_H_
