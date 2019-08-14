#include "mainwindow.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <QFileDialog>
#include <QMessageBox>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "ui_mainwindow.h"
//
//
// static void addGroundInViewer(pcl::visualization::PCLVisualizer& viewer) {
//
//   // viewer.setBackgroundColor(1.0f, 0.5f, 1.0f);
// 	// pcl::PointXYZ o;
// 	// o.x = 0.0;
// 	// o.y = 0;
// 	// o.z = 0;
// 	// viewer.addSphere(o, 0.25, "Sphere", 0);
// }

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow) {
  ui->setupUi(this);

  viewer = new pcl::visualization::CloudViewer("Display");

  menuBar = findChild<QMenuBar*>("actionMenuBar");
  menuFirst = findChild<QMenu*>("menuFirst");
  menuSecond = findChild<QMenu*>("menuSecond");
  actionAddLidar = findChild<QAction*>("actionAdd_Lidar");
  currentLidarName = findChild<QLabel*>("currentLidarName");
  currentLidarIp = findChild<QLabel*>("currentLidarIp");
  currentLidarPort = findChild<QLabel*>("currentLidarPort");
  currentLidarModel = findChild<QLabel*>("currentLidarModel");
  currentLidarType = findChild<QLabel*>("currentLidarType");

  xSlider = findChild<QSlider*>("xSlider");
  ySlider = findChild<QSlider*>("ySlider");
  zSlider = findChild<QSlider*>("zSlider");
  rollSlider = findChild<QSlider*>("rollSlider");
  pitchSlider = findChild<QSlider*>("pitchSlider");
  yawSlider = findChild<QSlider*>("yawSlider");
  redSlider = findChild<QSlider*>("redSlider");
  greenSlider = findChild<QSlider*>("greenSlider");
  blueSlider = findChild<QSlider*>("blueSlider");

  xDspl = findChild<QLCDNumber*>("xDisplay");
  yDspl = findChild<QLCDNumber*>("yDisplay");
  zDspl = findChild<QLCDNumber*>("zDisplay");
  rollDspl = findChild<QLCDNumber*>("rollDisplay");
  pitchDspl = findChild<QLCDNumber*>("pitchDisplay");
  yawDspl = findChild<QLCDNumber*>("yawDisplay");
  redDspl = findChild<QLCDNumber*>("redDisplay");
  greenDspl = findChild<QLCDNumber*>("greenDisplay");
  blueDspl = findChild<QLCDNumber*>("blueDisplay");

  fileGenerationBtn = findChild<QPushButton*>("fileGenerationButton");
  lidarConnectButton = findChild<QPushButton*>("connectButton");
  lidarDisconnectButton = findChild<QPushButton*>("disconnectButton");

  init();
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::init() {
  xSlider->setMaximum(2000);
  xSlider->setMinimum(-2000);
  xSlider->setTickPosition(QSlider::TickPosition(0));

  ySlider->setMaximum(2000);
  ySlider->setMinimum(-2000);
  ySlider->setTickPosition(QSlider::TickPosition(0));

  zSlider->setMaximum(2000);
  zSlider->setMinimum(-2000);
  zSlider->setTickPosition(QSlider::TickPosition(0));

  rollSlider->setMaximum(36000);
  rollSlider->setMinimum(0);

  pitchSlider->setMaximum(36000);
  pitchSlider->setMinimum(0);

  yawSlider->setMaximum(36000);
  yawSlider->setMinimum(0);

  redSlider->setMaximum(255);
  redSlider->setMinimum(0);

  greenSlider->setMaximum(255);
  greenSlider->setMinimum(0);

  blueSlider->setMaximum(255);
  blueSlider->setMinimum(0);

  connect (xSlider, SIGNAL (valueChanged (int)), this, SLOT (xSliderValueChanged (int)));
  connect (ySlider, SIGNAL (valueChanged (int)), this, SLOT (ySliderValueChanged (int)));
  connect (zSlider, SIGNAL (valueChanged (int)), this, SLOT (zSliderValueChanged (int)));
  connect (rollSlider, SIGNAL (valueChanged (int)), this, SLOT (rollSliderValueChanged (int)));
  connect (pitchSlider, SIGNAL (valueChanged (int)), this, SLOT (pitchSliderValueChanged (int)));
  connect (yawSlider, SIGNAL (valueChanged (int)), this, SLOT (yawSliderValueChanged (int)));
  connect (redSlider, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (greenSlider, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (blueSlider, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));

  connect(menuBar, SIGNAL(triggered(QAction*)), this, SLOT(trigerMenu(QAction*)));
  connect (fileGenerationBtn,  SIGNAL (clicked ()), this, SLOT (fileGenerationButtonPressed ()));
  connect (lidarConnectButton,  SIGNAL (clicked ()), this, SLOT (lidarConnectButtonPressed ()));
  connect (lidarDisconnectButton,  SIGNAL (clicked ()), this, SLOT (lidarDisconnectButtonPressed ()));
  addGroudInViewer();
}

void MainWindow::addGroudInViewer() {
  pcl::PointCloud<pcl::PointXYZRGB> ground;
  pcl::PointXYZRGB p;
  float x = -50.0;
  float y = -50.0;
  for(int i = 0; i < 101; i++) {
    for(int j = 0; j < 101; j++) {
      p.x = x + i;
      p.y = y + j;
      p.z = 0.0;

      p.r = 255;
      p.g = 255;
      p.b = 255;

      if ((j == 50 || j == 49 || j == 51) && i >= 50) {
        p.r = 255;
        p.g = 0;
        p.b = 0;
      }

      if ((i == 50 || i == 49 || i == 51) && j >= 50) {
        p.r = 0;
        p.g = 255;
        p.b = 0;
      }
      ground.points.push_back(p);
    }
  }
  viewer->showCloud(ground.makeShared(), "ground");
}

void MainWindow::trigerMenu(QAction* act) {
  if(act->text() == "Add Lidar") {
    AddLidarDialog addLidarDlg;
    int ret = addLidarDlg.exec();
    if (ret == AddLidarDialog::Rejected)
      return;

    CLidarConfig config = addLidarDlg.getLidarConfig();
    if (config.name.empty()) {
      QMessageBox msgBox;
      msgBox.setText("The name is empty!");
      msgBox.exec();
      return;
    }

    if (config.ip.empty()) {
      QMessageBox msgBox;
      msgBox.setText("The ip is empty!");
      msgBox.exec();
      return;
    }

    if(lidars.find(config.name) != lidars.end()) {
      QMessageBox msgBox;
      msgBox.setText("Lidar name existed!");
      msgBox.exec();
      return;
    }

    currentLidarName->setText(config.name.c_str());
    currentLidarIp->setText(config.ip.c_str());
    currentLidarPort->setText(std::to_string(config.port).c_str());
    currentLidarModel->setText(config.model.c_str());
    currentLidarType->setText(std::to_string(config.returnType).c_str());
    CLidarInfo lidarInfo;
    lidarInfo.config = config;
    lidarInfo.x = 0.0;
    lidarInfo.y = 0.0;
    lidarInfo.z = 0.0;
    lidarInfo.roll = 0.0;
    lidarInfo.pitch = 0.0;
    lidarInfo.yaw = 0.0;
    lidarInfo.red = 0;
    lidarInfo.green = 0;
    lidarInfo.blue = 0;
    updateConfig(lidarInfo);
    lidars.insert( std::pair<std::string, CLidarInfo>(config.name, lidarInfo) );
    QAction *lidarAction = new QAction(this);
    lidarAction->setText(config.name.c_str());
    menuSecond->addAction(lidarAction);

    Lidar lidar(config.ip, config.port, config.model, config.returnType, config.correctionFilePath, config.name, boost::bind(&MainWindow::lidarCallback, this, _1, _2, _3));
    lidarDevices.insert( std::pair<std::string, Lidar>(config.name, lidar) );
  } else if (lidars.find(act->text().toStdString()) != lidars.end()) {
    CLidarInfo currentLidar = lidars.find(act->text().toStdString())->second;
    currentLidarName->setText(act->text());
    currentLidarIp->setText(currentLidar.config.ip.c_str());
    currentLidarPort->setText(std::to_string(currentLidar.config.port).c_str());
    currentLidarModel->setText(currentLidar.config.model.c_str());
    currentLidarType->setText(std::to_string(currentLidar.config.returnType).c_str());
    updateConfig(currentLidar);
  }
}

void MainWindow::updateConfig(CLidarInfo lidarConfig) {
  xSlider->setValue(round(lidarConfig.x * 100));
  ySlider->setValue(round(lidarConfig.y * 100));
  zSlider->setValue(round(lidarConfig.z * 100));
  rollSlider->setValue(round(lidarConfig.roll * 100));
  pitchSlider->setValue(round(lidarConfig.pitch * 100));
  yawSlider->setValue(round(lidarConfig.yaw * 100));
  redSlider->setValue(lidarConfig.red);
  greenSlider->setValue(lidarConfig.green);
  blueSlider->setValue(lidarConfig.blue);

  xDspl->display(lidarConfig.x);
  yDspl->display(lidarConfig.y);
  zDspl->display(lidarConfig.z);
  rollDspl->display(lidarConfig.roll);
  pitchDspl->display(lidarConfig.pitch);
  yawDspl->display(lidarConfig.yaw);
  redDspl->display(lidarConfig.red);
  greenDspl->display(lidarConfig.green);
  blueDspl->display(lidarConfig.blue);
}

void MainWindow::xSliderValueChanged(int value) {
  xDspl->display(value * 0.01);
  lidars.find(currentLidarName->text().toStdString())->second.x = value * 0.01;
  cloudDisplayUpdate();
}

void MainWindow::ySliderValueChanged(int value) {
  yDspl->display(value * 0.01);
  lidars.find(currentLidarName->text().toStdString())->second.y = value * 0.01;
  cloudDisplayUpdate();
}

void MainWindow::zSliderValueChanged(int value) {
  zDspl->display(value * 0.01);
  lidars.find(currentLidarName->text().toStdString())->second.z = value * 0.01;
  cloudDisplayUpdate();
}

void MainWindow::rollSliderValueChanged(int value) {
  rollDspl->display(value * 0.01);
  lidars.find(currentLidarName->text().toStdString())->second.roll = value * 0.01 * M_PI / 180.0;
  cloudDisplayUpdate();
}

void MainWindow::pitchSliderValueChanged(int value) {
  pitchDspl->display(value * 0.01);
  lidars.find(currentLidarName->text().toStdString())->second.pitch = value * 0.01 * M_PI / 180.0;
  cloudDisplayUpdate();
}

void MainWindow::yawSliderValueChanged(int value) {
  yawDspl->display(value * 0.01);
  lidars.find(currentLidarName->text().toStdString())->second.yaw = value * 0.01 * M_PI / 180.0;
  cloudDisplayUpdate();
}

void MainWindow::redSliderValueChanged(int value) {
  redDspl->display(value);
  lidars.find(currentLidarName->text().toStdString())->second.red = value;
  colorUpdate();
}

void MainWindow::greenSliderValueChanged(int value) {
  greenDspl->display(value);
  lidars.find(currentLidarName->text().toStdString())->second.green = value;
  colorUpdate();
}

void MainWindow::blueSliderValueChanged(int value) {
  blueDspl->display(value);
  lidars.find(currentLidarName->text().toStdString())->second.blue = value;
  colorUpdate();
}

void MainWindow::colorUpdate() {
  for (size_t i = 0; i < lidars.find(currentLidarName->text().toStdString())->second.cloud.size (); i++) {
    lidars.find(currentLidarName->text().toStdString())->second.cloud.points[i].r = lidars.find(currentLidarName->text().toStdString())->second.red;
    lidars.find(currentLidarName->text().toStdString())->second.cloud.points[i].g = lidars.find(currentLidarName->text().toStdString())->second.green;
    lidars.find(currentLidarName->text().toStdString())->second.cloud.points[i].b = lidars.find(currentLidarName->text().toStdString())->second.blue;
  }
  cloudDisplayUpdate();
}

void MainWindow::fileGenerationButtonPressed() {
  if(!currentLidarName->text().toStdString().empty()) {
    CLidarInfo currentLidarInfo = lidars.find(currentLidarName->text().toStdString())->second;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    QString folderName;
    folderName = dialog.getExistingDirectory();
    std::ofstream calibrationfile;
    std::string filePath = folderName.toStdString() + "/" + currentLidarName->text().toStdString() + ".yaml";
    // std::cout << filePath << std::endl;
    calibrationfile.open (filePath);
    calibrationfile << "correction_x: " + std::to_string(currentLidarInfo.x) << std::endl;
    calibrationfile << "correction_y: " + std::to_string(currentLidarInfo.y) << std::endl;
    calibrationfile << "correction_z: " + std::to_string(currentLidarInfo.z) << std::endl;
    calibrationfile << "correction_roll: " + std::to_string(currentLidarInfo.roll) << std::endl;
    calibrationfile << "correction_pitch: " + std::to_string(currentLidarInfo.pitch) << std::endl;
    calibrationfile << "correction_yaw: " + std::to_string(currentLidarInfo.yaw) << std::endl;
    calibrationfile.close();
  }
}

void MainWindow::lidarConnectButtonPressed() {
  if(!currentLidarName->text().toStdString().empty()) {
    lidarDevices.find(currentLidarName->text().toStdString())->second.start();
  }
}

void MainWindow::lidarDisconnectButtonPressed() {
  if(!currentLidarName->text().toStdString().empty()) {
    lidarDevices.find(currentLidarName->text().toStdString())->second.stop();
  }
}

void MainWindow::cloudDisplayUpdate() {
  cloudDisplay.clear();
  std::map<std::string, CLidarInfo>::iterator iter;
  for(iter = lidars.begin(); iter != lidars.end(); iter++) {
    cloudDisplay += *getCloud(iter->second);
  }
  viewer->showCloud(cloudDisplay.makeShared());
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr MainWindow::getCloud(CLidarInfo currentLidar) const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  Eigen::Translation3f tl(currentLidar.x, currentLidar.y, currentLidar.z);  // tl: translation
  Eigen::AngleAxisf rot_x(currentLidar.roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y(currentLidar.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z(currentLidar.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f tf_rot_matrix = (tl * rot_x * rot_y * rot_z).matrix();
  pcl::transformPointCloud(currentLidar.cloud, *filtered, tf_rot_matrix);
  return filtered;
}

void MainWindow::lidarCallback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp, std::string deviceName) {
  (void) timestamp;
  pcl::copyPointCloud(*cloud, lidars.find(deviceName)->second.cloud);
  for (size_t i = 0; i < lidars.find(deviceName)->second.cloud.size (); i++) {
    lidars.find(deviceName)->second.cloud.points[i].r = lidars.find(deviceName)->second.red;
    lidars.find(deviceName)->second.cloud.points[i].g = lidars.find(deviceName)->second.green;
    lidars.find(deviceName)->second.cloud.points[i].b = lidars.find(deviceName)->second.blue;
  }
  cloudDisplayUpdate();
}
