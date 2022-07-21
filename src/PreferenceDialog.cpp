#include "PreferenceDialog.h"
#include "ui_PreferenceDialog.h"

#include "CameraFactory.h"
#include "Codec.h"
#include "OpenGLContext.h"

#include <QSettings>

using namespace std;

PreferenceDialog::PreferenceDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::PreferenceDialog) {
  ui->setupUi(this);

  // Query projectors
  vector<ScreenInfo> screenList = OpenGLContext::GetScreenInfo();
  for (unsigned int i = 0; i < screenList.size(); i++) {
    QString screenString = QString("Screen %1: %2x%3")
                               .arg(i)
                               .arg(screenList[i].resX)
                               .arg(screenList[i].resY);
    ui->projectorComboBox->addItem(screenString, i);
  }
  // Add virtual projector option
  ui->projectorComboBox->addItem("SLStudio Virtual Screen", -1);
// Add LC3000 option
#ifdef WITH_LC3000API
  ui->projectorComboBox->addItem("LC3000 API", -2);
#endif
// Add LC4500 option
#ifdef WITH_LC4500API
  ui->projectorComboBox->addItem("LC4500 API", -3);
#endif

  // Query cameras
  vector<vector<CameraInfo>> interfaceCameraList =
      CameraFactory::GetInterfaceCameraList();
  for (unsigned int i = 0; i < interfaceCameraList.size(); i++) {
    vector<CameraInfo> cameraList = interfaceCameraList[i];
    for (unsigned int j = 0; j < cameraList.size(); j++) {
      QString cameraString = QString("%1: %2")
                                 .arg(cameraList[j].vendor.c_str())
                                 .arg(cameraList[j].model.c_str());
      ui->cameraComboBox->addItem(cameraString, QPoint(i, j));
    }
  }
  // Add virtual camera option
  ui->cameraComboBox->addItem("Load files (virtual camera)", QPoint(-1, -1));

  // List pattern modes
  ui->patternModeComboBox->addItem("3 Pattern Phase Shift", "PhaseShift3");
  ui->patternModeComboBox->addItem("4 Pattern Phase Shift", "PhaseShift4");
  ui->patternModeComboBox->addItem("2x3 Pattern Phase Shift", "PhaseShift2x3");
  ui->patternModeComboBox->addItem("3 Pattern Phase Shift Unwrap",
                                   "PhaseShift3Unwrap");
  ui->patternModeComboBox->addItem("N Step Pattern Phase Shift",
                                   "PhaseShiftNStep");
  ui->patternModeComboBox->addItem("3 Pattern Phase Shift Fast Wrap",
                                   "PhaseShift3FastWrap");
  ui->patternModeComboBox->addItem("2+1 Pattern Phase Shift", "PhaseShift2p1");
  ui->patternModeComboBox->addItem("Descattering Phase Shift",
                                   "PhaseShiftDescatter");
  ui->patternModeComboBox->addItem("Modulated Phase Shift",
                                   "PhaseShiftModulated");
  ui->patternModeComboBox->addItem("Micro Phase Shift", "PhaseShiftMicro");
  ui->patternModeComboBox->addItem("Fast Ratio", "FastRatio");
  ui->patternModeComboBox->addItem("Gray Coding", "GrayCode");

  // Set all elements to current application settings
  QSettings settings("SLStudio");

  QString aquistion = settings.value("aquisition", "continuous").toString();
  if (aquistion == "continuous")
    ui->aquisitioncontinuousRadioButton->setChecked(true);
  else
    ui->aquisitionSingleRadioButton->setChecked(true);

  unsigned int patternModeIndex =
      ui->patternModeComboBox->findData(settings.value("pattern/mode"));
  ui->patternModeComboBox->setCurrentIndex(patternModeIndex);

  CodecDir codecDir = static_cast<CodecDir>(
      settings.value("pattern/direction", static_cast<int>(CodecDirHorizontal))
          .toInt());
  ui->patternHorizontalCheckBox->setChecked(
      static_cast<int>(codecDir) & static_cast<int>(CodecDirHorizontal));
  ui->patternVerticalCheckBox->setChecked(static_cast<int>(codecDir) &
                                          static_cast<int>(CodecDirVertical));

  int projectorIndex =
      ui->projectorComboBox->findData(settings.value("projector/screenNumber"));
  ui->projectorComboBox->setCurrentIndex(projectorIndex);
  // ui->verticalBaselineCheckbox->setChecked(settings.value("projector/verticalBaseline").toBool());

  QPoint cameraInterfaceSetting =
      QPoint(settings.value("camera/interfaceNumber").toInt(),
             settings.value("camera/cameraNumber").toInt());
  unsigned int cameraIndex =
      ui->cameraComboBox->findData(cameraInterfaceSetting);
  ui->cameraComboBox->setCurrentIndex(cameraIndex);

  float shutter = settings.value("camera/shutter", 16.666).toFloat();
  ui->shutterDoubleSpinBox->setValue(shutter);

  QString triggerMode = settings.value("trigger/mode", "hardware").toString();
  if (triggerMode == "hardware") {
    ui->triggerHardwareRadioButton->setChecked(true);
    on_triggerHardwareRadioButton_clicked();
  } else {
    ui->triggerSoftwareRadioButton->setChecked(true);
    on_triggerSoftwareRadioButton_clicked();
  }
  unsigned int shift = settings.value("trigger/shift", 0).toInt();
  ui->shiftSpinBox->setValue(shift);
  unsigned int delay = settings.value("trigger/delay", 50).toInt();
  ui->delaySpinBox->setValue(delay);

  bool frames = settings.value("writeToDisk/frames", false).toBool();
  ui->framesCheckBox->setChecked(frames);

  bool pointclouds = settings.value("writeToDisk/pointclouds", false).toBool();
  ui->pointCloudsCheckBox->setChecked(pointclouds);
}

PreferenceDialog::~PreferenceDialog() { delete ui; }

void PreferenceDialog::on_buttonBox_accepted() {

  // Save settings
  QSettings settings("SLStudio");

  // Aquisition
  if (ui->aquisitioncontinuousRadioButton->isChecked())
    settings.setValue("aquisition", "continuous");
  else
    settings.setValue("aquisition", "single");

  // Pattern mode
  QString patternMode =
      ui->patternModeComboBox->itemData(ui->patternModeComboBox->currentIndex())
          .toString();
  settings.setValue("pattern/mode", patternMode);

  // Pattern direction
  bool patternHorizontal = ui->patternHorizontalCheckBox->isChecked();
  bool patternVertical = ui->patternVerticalCheckBox->isChecked();
  CodecDir dir = CodecDirNone;
  if (patternHorizontal && patternVertical)
    dir = CodecDirBoth;
  else if (patternHorizontal)
    dir = CodecDirHorizontal;
  else if (patternVertical)
    dir = CodecDirVertical;
  settings.setValue("pattern/direction", static_cast<int>(dir));

  // Projector
  int proj =
      ui->projectorComboBox->itemData(ui->projectorComboBox->currentIndex())
          .toInt();
  settings.setValue("projector/screenNumber", proj);

  // bool verticalBaseline = ui->verticalBaselineCheckbox->isChecked();
  // settings.setValue("projector/verticalBaseline", verticalBaseline);

  // Camera
  QPoint cam = ui->cameraComboBox->itemData(ui->cameraComboBox->currentIndex())
                   .toPoint();
  settings.setValue("camera/interfaceNumber", cam.x());
  settings.setValue("camera/cameraNumber", cam.y());

  float shutter = ui->shutterDoubleSpinBox->value();
  settings.setValue("camera/shutter", shutter);

  // Trigger mode
  if (ui->triggerHardwareRadioButton->isChecked())
    settings.setValue("trigger/mode", "hardware");
  else
    settings.setValue("trigger/mode", "software");
  unsigned int shift = ui->shiftSpinBox->value();
  settings.setValue("trigger/shift", shift);
  unsigned int delay = ui->delaySpinBox->value();
  settings.setValue("trigger/delay", delay);

  // Write to disk
  bool frames = ui->framesCheckBox->isChecked();
  settings.setValue("writeToDisk/frames", frames);
  bool pointclouds = ui->pointCloudsCheckBox->isChecked();
  settings.setValue("writeToDisk/pointclouds", pointclouds);
}

void PreferenceDialog::on_triggerHardwareRadioButton_clicked() {
  //    ui->shiftLayout->setEnabled(true);
  //    ui->delayLayout->setEnabled(false);
  ui->shiftLabel->setEnabled(true);
  ui->shiftSpinBox->setEnabled(true);
  ui->delayLabel->setEnabled(false);
  ui->delaySpinBox->setEnabled(false);
  ui->delayMsLabel->setEnabled(false);
}

void PreferenceDialog::on_triggerSoftwareRadioButton_clicked() {
  //    ui->delayLayout->setEnabled(true);
  //    ui->shiftLayout->setEnabled(false);
  ui->shiftLabel->setEnabled(false);
  ui->shiftSpinBox->setEnabled(false);
  ui->delayLabel->setEnabled(true);
  ui->delaySpinBox->setEnabled(true);
  ui->delayMsLabel->setEnabled(true);
}

void PreferenceDialog::on_cameraComboBox_currentIndexChanged(
    const QString &arg1) {
  if (arg1 == "SLStudio Virtual Camera") {
    ui->shutterDoubleSpinBox->setEnabled(false);
  } else {
    ui->shutterDoubleSpinBox->setEnabled(true);
  }
}

void PreferenceDialog::on_patternHorizontalCheckBox_clicked() {
  if (!ui->patternHorizontalCheckBox->isChecked())
    ui->patternVerticalCheckBox->setChecked(true);
}

void PreferenceDialog::on_patternVerticalCheckBox_clicked() {
  if (!ui->patternVerticalCheckBox->isChecked())
    ui->patternHorizontalCheckBox->setChecked(true);
}
