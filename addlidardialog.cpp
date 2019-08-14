#include "addlidardialog.h"
#include <string.h>
#include <iostream>
#include <QFileDialog>
#include "ui_addlidardialog.h"

AddLidarDialog::AddLidarDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddLidarDialog) {
    ui->setupUi(this);

    modelGroup = new QButtonGroup();
    returnTypeGroup = new QButtonGroup();

    vlp16RadioBtn = findChild<QRadioButton*>("vlp16RadioButton");
    pandar40RadioBtn = findChild<QRadioButton*>("pandar40RadioButton");
    singleRadioBtn = findChild<QRadioButton*>("singleRadioButton");
    dualRadioBtn = findChild<QRadioButton*>("dualRadioButton");
    nameInput = findChild<QLineEdit*>("nameInput");
    ipInput = findChild<QLineEdit*>("ipInput");
    portInput = findChild<QLineEdit*>("portInput");


    correctionFileInput = findChild<QLabel*>("correctionFileInput");
    selectFileButton = findChild<QPushButton*>("selectFileButton");

    modelGroup->addButton(vlp16RadioBtn, 1);
    modelGroup->addButton(pandar40RadioBtn, 2);
    vlp16RadioBtn->setChecked(true);

    returnTypeGroup->addButton(singleRadioBtn, 1);
    returnTypeGroup->addButton(dualRadioBtn, 2);
    singleRadioBtn->setChecked(true);

    portInput->setValidator(new QIntValidator(portInput));
    ipInput->setInputMask("000.000.000.000");

    lidarConfig.model = "VLP16";
    lidarConfig.port = 2368;
    lidarConfig.returnType = 0;
    lidarConfig.correctionFilePath = " ";

    initConnection();
}

AddLidarDialog::~AddLidarDialog() {

}

void AddLidarDialog::initConnection() {
    connect (nameInput, SIGNAL (textChanged(const QString &)), this, SLOT (nameUpdate(const QString &)));
    connect (ipInput, SIGNAL (textChanged(const QString &)), this, SLOT (ipUpdate(const QString &)));
    connect (portInput, SIGNAL (textChanged(const QString &)), this, SLOT (portUpdate(const QString &)));

    connect(modelGroup, SIGNAL(buttonClicked(int)), this, SLOT(modelGroupButtonsClicked(int)));
    connect(returnTypeGroup, SIGNAL(buttonClicked(int)), this, SLOT(returnTypeGroupButtonsClicked(int)));

    connect (selectFileButton,  SIGNAL (clicked ()), this, SLOT (selectFileButtonPressed ()));
}

void AddLidarDialog::nameUpdate(const QString &input) {
    lidarConfig.name = input.toStdString();
}

void AddLidarDialog::ipUpdate(const QString &input) {
    lidarConfig.ip = input.toStdString();
}

void AddLidarDialog::portUpdate(const QString &input) {
    lidarConfig.port = input.toInt();
}

CLidarConfig AddLidarDialog::getLidarConfig() {
    return lidarConfig;
}

void AddLidarDialog::modelGroupButtonsClicked(int id) {
    lidarConfig.model = modelGroup->button(id)->text().toStdString();
}

void AddLidarDialog::returnTypeGroupButtonsClicked(int id) {
    lidarConfig.returnType = id - 1;
}

void AddLidarDialog::selectFileButtonPressed() {
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select"), "/", tr("Files (*.csv)"));
  correctionFileInput->setText(fileName);
  lidarConfig.correctionFilePath = fileName.toStdString();
}
