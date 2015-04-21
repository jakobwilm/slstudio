/****************************************************************************
** Meta object code from reading C++ file 'SLTrackerWorker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLTrackerWorker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLTrackerWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLTrackerWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      19,   17,   16,   16, 0x05,
      56,   52,   16,   16, 0x05,

 // slots: signature, parameters, type, tag, flags
      71,   16,   16,   16, 0x0a,
      90,   79,   16,   16, 0x0a,
     146,  126,   16,   16, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SLTrackerWorker[] = {
    "SLTrackerWorker\0\0T\0newPoseEstimate(Eigen::Affine3f)\0"
    "err\0error(QString)\0setup()\0pointCloud\0"
    "trackPointCloud(PointCloudConstPtr)\0"
    "referencePointCloud\0"
    "setReference(PointCloudConstPtr)\0"
};

void SLTrackerWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLTrackerWorker *_t = static_cast<SLTrackerWorker *>(_o);
        switch (_id) {
        case 0: _t->newPoseEstimate((*reinterpret_cast< Eigen::Affine3f(*)>(_a[1]))); break;
        case 1: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->setup(); break;
        case 3: _t->trackPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 4: _t->setReference((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLTrackerWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLTrackerWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SLTrackerWorker,
      qt_meta_data_SLTrackerWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLTrackerWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLTrackerWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLTrackerWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLTrackerWorker))
        return static_cast<void*>(const_cast< SLTrackerWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int SLTrackerWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void SLTrackerWorker::newPoseEstimate(Eigen::Affine3f _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLTrackerWorker::error(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
