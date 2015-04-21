/****************************************************************************
** Meta object code from reading C++ file 'SLTraceWidget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLTraceWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLTraceWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLTraceWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      20,   15,   14,   14, 0x0a,
      45,   38,   14,   14, 0x0a,
      81,   72,   14,   14, 0x0a,
     116,  111,   14,   14, 0x0a,
     171,  152,   14,   14, 0x0a,
     200,   14,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SLTraceWidget[] = {
    "SLTraceWidget\0\0name\0addTrace(QString)\0"
    "id,val\0addMeasurement(uint,float)\0"
    "name,val\0addMeasurement(QString,float)\0"
    "vals\0addMeasurements(std::vector<float>)\0"
    "_tSpan,_yMin,_yMax\0setBounds(float,float,float)\0"
    "draw()\0"
};

void SLTraceWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLTraceWidget *_t = static_cast<SLTraceWidget *>(_o);
        switch (_id) {
        case 0: _t->addTrace((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->addMeasurement((*reinterpret_cast< uint(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 2: _t->addMeasurement((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 3: _t->addMeasurements((*reinterpret_cast< std::vector<float>(*)>(_a[1]))); break;
        case 4: _t->setBounds((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 5: _t->draw(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLTraceWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLTraceWidget::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_SLTraceWidget,
      qt_meta_data_SLTraceWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLTraceWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLTraceWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLTraceWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLTraceWidget))
        return static_cast<void*>(const_cast< SLTraceWidget*>(this));
    return QLabel::qt_metacast(_clname);
}

int SLTraceWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
