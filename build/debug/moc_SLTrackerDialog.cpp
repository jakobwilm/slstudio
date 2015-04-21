/****************************************************************************
** Meta object code from reading C++ file 'SLTrackerDialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLTrackerDialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLTrackerDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLTrackerDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      28,   17,   16,   16, 0x05,

 // slots: signature, parameters, type, tag, flags
      62,   17,   16,   16, 0x0a,
     105,  103,   16,   16, 0x0a,
     139,   16,   16,   16, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SLTrackerDialog[] = {
    "SLTrackerDialog\0\0pointCloud\0"
    "newPointCloud(PointCloudConstPtr)\0"
    "receiveNewPointCloud(PointCloudConstPtr)\0"
    "T\0showPoseEstimate(Eigen::Affine3f)\0"
    "on_startStopPushButton_clicked()\0"
};

void SLTrackerDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLTrackerDialog *_t = static_cast<SLTrackerDialog *>(_o);
        switch (_id) {
        case 0: _t->newPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 1: _t->receiveNewPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 2: _t->showPoseEstimate((*reinterpret_cast< Eigen::Affine3f(*)>(_a[1]))); break;
        case 3: _t->on_startStopPushButton_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLTrackerDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLTrackerDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SLTrackerDialog,
      qt_meta_data_SLTrackerDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLTrackerDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLTrackerDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLTrackerDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLTrackerDialog))
        return static_cast<void*>(const_cast< SLTrackerDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int SLTrackerDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void SLTrackerDialog::newPointCloud(PointCloudConstPtr _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
