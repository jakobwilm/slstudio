/****************************************************************************
** Meta object code from reading C++ file 'SLStudio.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLStudio.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLStudio.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLStudio[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      21,   10,    9,    9, 0x05,

 // slots: signature, parameters, type, tag, flags
      55,    9,    9,    9, 0x08,
      71,    9,    9,    9, 0x08,
      86,    9,    9,    9, 0x08,
     109,    9,    9,    9, 0x08,
     131,    9,    9,    9, 0x08,
     157,    9,    9,    9, 0x08,
     179,    9,    9,    9, 0x08,
     207,    9,    9,    9, 0x08,
     227,   10,    9,    9, 0x08,
     286,  268,    9,    9, 0x08,
     324,  268,    9,    9, 0x08,
     363,  360,    9,    9, 0x08,
     388,  360,    9,    9, 0x08,
     411,  360,    9,    9, 0x08,
     436,  360,    9,    9, 0x08,
     461,    9,    9,    9, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SLStudio[] = {
    "SLStudio\0\0pointCloud\0"
    "newPointCloud(PointCloudConstPtr)\0"
    "onActionStart()\0onActionStop()\0"
    "onScanWorkerFinished()\0onActionCalibration()\0"
    "onActionLoadCalibration()\0"
    "onActionPreferences()\0onActionExportCalibration()\0"
    "updateDisplayRate()\0"
    "receiveNewPointCloud(PointCloudConstPtr)\0"
    "windowName,im,x,y\0"
    "imshow(const char*,cv::Mat,uint,uint)\0"
    "hist(const char*,cv::Mat,uint,uint)\0"
    "im\0onShowHistogram(cv::Mat)\0"
    "onShowShading(cv::Mat)\0onShowDecoderUp(cv::Mat)\0"
    "onShowDecoderVp(cv::Mat)\0onActionAbout()\0"
};

void SLStudio::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLStudio *_t = static_cast<SLStudio *>(_o);
        switch (_id) {
        case 0: _t->newPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 1: _t->onActionStart(); break;
        case 2: _t->onActionStop(); break;
        case 3: _t->onScanWorkerFinished(); break;
        case 4: _t->onActionCalibration(); break;
        case 5: _t->onActionLoadCalibration(); break;
        case 6: _t->onActionPreferences(); break;
        case 7: _t->onActionExportCalibration(); break;
        case 8: _t->updateDisplayRate(); break;
        case 9: _t->receiveNewPointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 10: _t->imshow((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3])),(*reinterpret_cast< uint(*)>(_a[4]))); break;
        case 11: _t->hist((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3])),(*reinterpret_cast< uint(*)>(_a[4]))); break;
        case 12: _t->onShowHistogram((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 13: _t->onShowShading((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 14: _t->onShowDecoderUp((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 15: _t->onShowDecoderVp((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 16: _t->onActionAbout(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLStudio::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLStudio::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_SLStudio,
      qt_meta_data_SLStudio, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLStudio::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLStudio::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLStudio::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLStudio))
        return static_cast<void*>(const_cast< SLStudio*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int SLStudio::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    }
    return _id;
}

// SIGNAL 0
void SLStudio::newPointCloud(PointCloudConstPtr _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
