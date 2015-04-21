/****************************************************************************
** Meta object code from reading C++ file 'SLDecoderWorker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLDecoderWorker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLDecoderWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLDecoderWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
      36,   17,   16,   16, 0x05,
      78,   74,   16,   16, 0x05,
      99,   74,   16,   16, 0x05,
     122,   74,   16,   16, 0x05,
     164,  145,   16,   16, 0x05,
     209,  205,   16,   16, 0x05,

 // slots: signature, parameters, type, tag, flags
     224,   16,   16,   16, 0x0a,
     241,  232,   16,   16, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SLDecoderWorker[] = {
    "SLDecoderWorker\0\0windowName,mat,x,y\0"
    "imshow(const char*,cv::Mat,uint,uint)\0"
    "mat\0showShading(cv::Mat)\0"
    "showDecoderUp(cv::Mat)\0showDecoderVp(cv::Mat)\0"
    "up,vp,mask,shading\0"
    "newUpVp(cv::Mat,cv::Mat,cv::Mat,cv::Mat)\0"
    "err\0error(QString)\0setup()\0frameSeq\0"
    "decodeSequence(std::vector<cv::Mat>)\0"
};

void SLDecoderWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLDecoderWorker *_t = static_cast<SLDecoderWorker *>(_o);
        switch (_id) {
        case 0: _t->imshow((*reinterpret_cast< const char*(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3])),(*reinterpret_cast< uint(*)>(_a[4]))); break;
        case 1: _t->showShading((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 2: _t->showDecoderUp((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 3: _t->showDecoderVp((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 4: _t->newUpVp((*reinterpret_cast< cv::Mat(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< cv::Mat(*)>(_a[3])),(*reinterpret_cast< cv::Mat(*)>(_a[4]))); break;
        case 5: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->setup(); break;
        case 7: _t->decodeSequence((*reinterpret_cast< std::vector<cv::Mat>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLDecoderWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLDecoderWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SLDecoderWorker,
      qt_meta_data_SLDecoderWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLDecoderWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLDecoderWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLDecoderWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLDecoderWorker))
        return static_cast<void*>(const_cast< SLDecoderWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int SLDecoderWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void SLDecoderWorker::imshow(const char * _t1, cv::Mat _t2, unsigned int _t3, unsigned int _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SLDecoderWorker::showShading(cv::Mat _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SLDecoderWorker::showDecoderUp(cv::Mat _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void SLDecoderWorker::showDecoderVp(cv::Mat _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void SLDecoderWorker::newUpVp(cv::Mat _t1, cv::Mat _t2, cv::Mat _t3, cv::Mat _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void SLDecoderWorker::error(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
