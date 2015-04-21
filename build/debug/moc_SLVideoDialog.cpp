/****************************************************************************
** Meta object code from reading C++ file 'SLVideoDialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLVideoDialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLVideoDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLVideoDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   15,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SLVideoDialog[] = {
    "SLVideoDialog\0\0image\0showImageCV(cv::Mat)\0"
};

void SLVideoDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLVideoDialog *_t = static_cast<SLVideoDialog *>(_o);
        switch (_id) {
        case 0: _t->showImageCV((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLVideoDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLVideoDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SLVideoDialog,
      qt_meta_data_SLVideoDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLVideoDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLVideoDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLVideoDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLVideoDialog))
        return static_cast<void*>(const_cast< SLVideoDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int SLVideoDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
