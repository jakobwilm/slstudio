/****************************************************************************
** Meta object code from reading C++ file 'CameraWorker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../camera/CameraWorker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CameraWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CameraWorker[] = {

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
      20,   14,   13,   13, 0x05,
      38,   13,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
      59,   49,   13,   13, 0x0a,
      76,   13,   13,   13, 0x0a,
      85,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_CameraWorker[] = {
    "CameraWorker\0\0frame\0newFrame(cv::Mat)\0"
    "finished()\0iNum,cNum\0setup(uint,uint)\0"
    "doWork()\0stopWorking()\0"
};

void CameraWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CameraWorker *_t = static_cast<CameraWorker *>(_o);
        switch (_id) {
        case 0: _t->newFrame((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 1: _t->finished(); break;
        case 2: _t->setup((*reinterpret_cast< uint(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2]))); break;
        case 3: _t->doWork(); break;
        case 4: _t->stopWorking(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData CameraWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject CameraWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_CameraWorker,
      qt_meta_data_CameraWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CameraWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CameraWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CameraWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CameraWorker))
        return static_cast<void*>(const_cast< CameraWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int CameraWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void CameraWorker::newFrame(cv::Mat _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CameraWorker::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
