#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QDialog>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QSlider>
#include <QLCDNumber>
#include <QLabel>
#include <map>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "addlidardialog.h"
#include "lidar_driver/driver.h"
#include "lidar_driver/lidar.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class MainWindow;
}

struct CLidarInfo {
  CLidarConfig config;
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  int red;
  int green;
  int blue;
  PointCloudT cloud;
  CLidarInfo() {
      memset(this, 0, sizeof(CLidarInfo));
  }
};

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private:
  Ui::MainWindow *ui;

  QMenuBar *menuBar;
  QMenu *menuFirst, *menuSecond;
  QAction *actionAddLidar;

  QLabel *currentLidarName, *currentLidarIp, *currentLidarPort, *currentLidarModel, *currentLidarType;

  QSlider *xSlider, *ySlider, *zSlider, *rollSlider, *pitchSlider, *yawSlider, *redSlider, *greenSlider, *blueSlider;
  QLCDNumber *xDspl, *yDspl, *zDspl, *rollDspl, *pitchDspl, *yawDspl, *redDspl, *greenDspl, *blueDspl;

  QPushButton *fileGenerationBtn, *lidarConnectButton, *lidarDisconnectButton;

  QDialog *addLidarDialog;

  pcl::visualization::CloudViewer *viewer;

  PointCloudT cloudDisplay;

  lidar_driver::Driver *lidarDriver;

  std::map<std::string, CLidarInfo> lidars;

  std::map<std::string, Lidar>  lidarDevices;

  void init();

  void updateConfig(CLidarInfo lidarConfig);

  void cloudDisplayUpdate();

  void cloudTransform();

  PointCloudT::ConstPtr getCloud(CLidarInfo currentLidar) const;

  void colorUpdate();

  void lidarCallback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp, std::string deviceName);

  void addGroudInViewer();
private slots:
  void trigerMenu(QAction* act);
  void xSliderValueChanged(int value);
  void ySliderValueChanged(int value);
  void zSliderValueChanged(int value);
  void rollSliderValueChanged(int value);
  void pitchSliderValueChanged(int value);
  void yawSliderValueChanged(int value);
  void redSliderValueChanged(int value);
  void greenSliderValueChanged(int value);
  void blueSliderValueChanged(int value);

  void fileGenerationButtonPressed();
  void lidarConnectButtonPressed();
  void lidarDisconnectButtonPressed();
};

#endif // MAINWINDOW_H
