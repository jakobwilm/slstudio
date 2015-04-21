/****************************************************************************
** Meta object code from reading C++ file 'SLPreferenceDialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SLPreferenceDialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SLPreferenceDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SLPreferenceDialog[] = {

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
      20,   19,   19,   19, 0x08,
      44,   19,   19,   19, 0x08,
      84,   19,   19,   19, 0x08,
     129,  124,   19,   19, 0x08,
     176,   19,   19,   19, 0x08,
     215,   19,   19,   19, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SLPreferenceDialog[] = {
    "SLPreferenceDialog\0\0on_buttonBox_accepted()\0"
    "on_triggerHardwareRadioButton_clicked()\0"
    "on_triggerSoftwareRadioButton_clicked()\0"
    "arg1\0on_cameraComboBox_currentIndexChanged(QString)\0"
    "on_patternHorizontalCheckBox_clicked()\0"
    "on_patternVerticalCheckBox_clicked()\0"
};

void SLPreferenceDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SLPreferenceDialog *_t = static_cast<SLPreferenceDialog *>(_o);
        switch (_id) {
        case 0: _t->on_buttonBox_accepted(); break;
        case 1: _t->on_triggerHardwareRadioButton_clicked(); break;
        case 2: _t->on_triggerSoftwareRadioButton_clicked(); break;
        case 3: _t->on_cameraComboBox_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->on_patternHorizontalCheckBox_clicked(); break;
        case 5: _t->on_patternVerticalCheckBox_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SLPreferenceDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SLPreferenceDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SLPreferenceDialog,
      qt_meta_data_SLPreferenceDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SLPreferenceDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SLPreferenceDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SLPreferenceDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SLPreferenceDialog))
        return static_cast<void*>(const_cast< SLPreferenceDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int SLPreferenceDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
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
