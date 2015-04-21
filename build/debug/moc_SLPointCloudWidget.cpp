/****************************************************************************
** Meta object code from reading C++ file 'SLPointCloudWidget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLPointCloudWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLPointCloudWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLPointCloudWidget[] = {

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
      20,   19,   19,   19, 0x05,

 // slots: signature, parameters, type, tag, flags
      60,   45,   19,   19, 0x0a,
      97,   19,   19,   19, 0x0a,
     114,   19,   19,   19, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SLPointCloudWidget[] = {
    "SLPointCloudWidget\0\0newPointCloudDisplayed()\0"
    "_pointCloudPCL\0updatePointCloud(PointCloudConstPtr)\0"
    "savePointCloud()\0saveScreenShot()\0"
};

void SLPointCloudWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLPointCloudWidget *_t = static_cast<SLPointCloudWidget *>(_o);
        switch (_id) {
        case 0: _t->newPointCloudDisplayed(); break;
        case 1: _t->updatePointCloud((*reinterpret_cast< PointCloudConstPtr(*)>(_a[1]))); break;
        case 2: _t->savePointCloud(); break;
        case 3: _t->saveScreenShot(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLPointCloudWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLPointCloudWidget::staticMetaObject = {
    { &QVTKWidget::staticMetaObject, qt_meta_stringdata_SLPointCloudWidget,
      qt_meta_data_SLPointCloudWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLPointCloudWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLPointCloudWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLPointCloudWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLPointCloudWidget))
        return static_cast<void*>(const_cast< SLPointCloudWidget*>(this));
    return QVTKWidget::qt_metacast(_clname);
}

int SLPointCloudWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QVTKWidget::qt_metacall(_c, _id, _a);
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
void SLPointCloudWidget::newPointCloudDisplayed()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
