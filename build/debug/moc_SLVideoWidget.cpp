/****************************************************************************
** Meta object code from reading C++ file 'SLVideoWidget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLVideoWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLVideoWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLVideoWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   15,   14,   14, 0x0a,
      44,   15,   14,   14, 0x0a,
      71,   65,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SLVideoWidget[] = {
    "SLVideoWidget\0\0frame\0showFrame(CameraFrame)\0"
    "showFrameCV(cv::Mat)\0event\0"
    "resizeEvent(QResizeEvent*)\0"
};

void SLVideoWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLVideoWidget *_t = static_cast<SLVideoWidget *>(_o);
        switch (_id) {
        case 0: _t->showFrame((*reinterpret_cast< CameraFrame(*)>(_a[1]))); break;
        case 1: _t->showFrameCV((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 2: _t->resizeEvent((*reinterpret_cast< QResizeEvent*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLVideoWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLVideoWidget::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_SLVideoWidget,
      qt_meta_data_SLVideoWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLVideoWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLVideoWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLVideoWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLVideoWidget))
        return static_cast<void*>(const_cast< SLVideoWidget*>(this));
    return QLabel::qt_metacast(_clname);
}

int SLVideoWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
