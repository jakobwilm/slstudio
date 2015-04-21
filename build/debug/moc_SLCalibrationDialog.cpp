/****************************************************************************
** Meta object code from reading C++ file 'SLCalibrationDialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLCalibrationDialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLCalibrationDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLCalibrationDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      28,   21,   20,   20, 0x05,

 // slots: signature, parameters, type, tag, flags
      65,   20,   20,   20, 0x08,
      89,   20,   20,   20, 0x08,
     118,   20,   20,   20, 0x08,
     155,   20,   20,   20, 0x08,
     195,  179,   20,   20, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SLCalibrationDialog[] = {
    "SLCalibrationDialog\0\0_calib\0"
    "newCalibrationSaved(CalibrationData)\0"
    "on_snapButton_clicked()\0"
    "on_calibrateButton_clicked()\0"
    "on_listWidget_itemSelectionChanged()\0"
    "on_saveButton_clicked()\0img,idx,success\0"
    "onNewSequenceResult(cv::Mat,uint,bool)\0"
};

void SLCalibrationDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLCalibrationDialog *_t = static_cast<SLCalibrationDialog *>(_o);
        switch (_id) {
        case 0: _t->newCalibrationSaved((*reinterpret_cast< CalibrationData(*)>(_a[1]))); break;
        case 1: _t->on_snapButton_clicked(); break;
        case 2: _t->on_calibrateButton_clicked(); break;
        case 3: _t->on_listWidget_itemSelectionChanged(); break;
        case 4: _t->on_saveButton_clicked(); break;
        case 5: _t->onNewSequenceResult((*reinterpret_cast< cv::Mat(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLCalibrationDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLCalibrationDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SLCalibrationDialog,
      qt_meta_data_SLCalibrationDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLCalibrationDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLCalibrationDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLCalibrationDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLCalibrationDialog))
        return static_cast<void*>(const_cast< SLCalibrationDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int SLCalibrationDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void SLCalibrationDialog::newCalibrationSaved(CalibrationData _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
