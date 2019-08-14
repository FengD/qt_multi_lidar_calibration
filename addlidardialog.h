#ifndef ADDLIDARDIALOG_H
#define ADDLIDARDIALOG_H

#include <QWidget>
#include <QDebug>
#include <QPushButton>
#include <QDialog>
#include <QLineEdit>
#include <QRadioButton>
#include <QButtonGroup>
#include <QLabel>
#include <string>

namespace Ui {
class AddLidarDialog;
}

struct CLidarConfig {
   std::string name;
   std::string ip;
   int port;
   std::string model;
   int returnType;
   std::string correctionFilePath;
   CLidarConfig& operator= (const CLidarConfig& config) {
       name = config.name;
       ip = config.ip;
       port = config.port;
       model = config.model;
       returnType = config.returnType;
       correctionFilePath = config.correctionFilePath;
       return *this;
     }

   CLidarConfig() {
       memset(this, 0, sizeof(CLidarConfig));
   }
};

class AddLidarDialog : public QDialog {
    Q_OBJECT
private:
    Ui::AddLidarDialog *ui;
    QButtonGroup *modelGroup;
    QButtonGroup *returnTypeGroup;
    QRadioButton *vlp16RadioBtn;
    QRadioButton *pandar40RadioBtn;
    QRadioButton *singleRadioBtn;
    QRadioButton *dualRadioBtn;
    QLineEdit *ipInput;
    QLineEdit *portInput;
    QLineEdit *nameInput;

    QLabel *correctionFileInput;
    QPushButton *selectFileButton;
    CLidarConfig lidarConfig;

    void initConnection();

public:
    AddLidarDialog(QWidget *parent = 0);

    ~AddLidarDialog();

    CLidarConfig getLidarConfig();

private slots:
    void nameUpdate(const QString &input);

    void ipUpdate(const QString &input);

    void portUpdate(const QString &input);

    void modelGroupButtonsClicked(int id);

    void returnTypeGroupButtonsClicked(int id);

    void selectFileButtonPressed();
};


#endif // ADDLIDARDIALOG_H
