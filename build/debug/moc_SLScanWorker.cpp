/****************************************************************************
** Meta object code from reading C++ file 'SLScanWorker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLScanWorker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLScanWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLScanWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      17,   14,   13,   13, 0x05,
      46,   40,   13,   13, 0x05,
      73,   64,   13,   13, 0x05,
     111,  107,   13,   13, 0x05,
     126,   13,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
     137,   13,   13,   13, 0x0a,
     145,   13,   13,   13, 0x0a,
     154,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SLScanWorker[] = {
    "SLScanWorker\0\0im\0showHistogram(cv::Mat)\0"
    "frame\0newFrame(cv::Mat)\0frameSeq\0"
    "newFrameSeq(std::vector<cv::Mat>)\0err\0"
    "error(QString)\0finished()\0setup()\0"
    "doWork()\0stopWorking()\0"
};

void SLScanWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLScanWorker *_t = static_cast<SLScanWorker *>(_o);
        switch (_id) {
        case 0: _t->showHistogram((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 1: _t->newFrame((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 2: _t->newFrameSeq((*reinterpret_cast< std::vector<cv::Mat>(*)>(_a[1]))); break;
        case 3: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->finished(); break;
        case 5: _t->setup(); break;
        case 6: _t->doWork(); break;
        case 7: _t->stopWorking(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLScanWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLScanWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SLScanWorker,
      qt_meta_data_SLScanWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLScanWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLScanWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLScanWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLScanWorker))
        return static_cast<void*>(const_cast< SLScanWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int SLScanWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void SLScanWorker::showHistogram(cv::Mat _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLScanWorker::newFrame(cv::Mat _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SLScanWorker::newFrameSeq(std::vector<cv::Mat> _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void SLScanWorker::error(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void SLScanWorker::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}
QT_END_MOC_NAMESPACE
