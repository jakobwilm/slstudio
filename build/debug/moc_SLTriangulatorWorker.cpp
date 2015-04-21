/****************************************************************************
** Meta object code from reading C++ file 'SLTriangulatorWorker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLTriangulatorWorker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLTriangulatorWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLTriangulatorWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      41,   22,   21,   21, 0x05,
      90,   79,   21,   21, 0x05,
     128,  124,   21,   21, 0x05,

 // slots: signature, parameters, type, tag, flags
     143,   21,   21,   21, 0x0a,
     170,  151,   21,   21, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SLTriangulatorWorker[] = {
    "SLTriangulatorWorker\0\0windowName,mat,x,y\0"
    "imshow(const char*,cv::Mat,uint,uint)\0"
    "pointCloud\0newPointCloud(PointCloudConstPtr)\0"
    "err\0error(QString)\0setup()\0"
    "up,vp,mask,shading\0"
    "triangulatePointCloud(cv::Mat,cv::Mat,cv::Mat,cv::Mat)\0"
};

void SLTriangulatorWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLTriangulatorWorker *_t = static_cast<SLTriangulatorWorker *>(_o);
        switch (_id) {
        case 0: _t->imshow((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3])),(*reinterpret_cast< uint(*)>(_a[4]))); break;
        case 1: _t->newPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 2: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->setup(); break;
        case 4: _t->triangulatePointCloud((*reinterpret_cast< cv::Mat(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< cv::Mat(*)>(_a[3])),(*reinterpret_cast< cv::Mat(*)>(_a[4]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLTriangulatorWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLTriangulatorWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SLTriangulatorWorker,
      qt_meta_data_SLTriangulatorWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLTriangulatorWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLTriangulatorWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLTriangulatorWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLTriangulatorWorker))
        return static_cast<void*>(const_cast< SLTriangulatorWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int SLTriangulatorWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void SLTriangulatorWorker::imshow(const char * _t1, cv::Mat _t2, unsigned int _t3, unsigned int _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLTriangulatorWorker::newPointCloud(PointCloudConstPtr _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SLTriangulatorWorker::error(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
