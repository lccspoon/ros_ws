/****************************************************************************
** Meta object code from reading C++ file 'VectorAtPositionDisplay.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/kindr_ros/kindr_rviz_plugins/include/kindr_rviz_plugins/VectorAtPositionDisplay.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'VectorAtPositionDisplay.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_kindr_rviz_plugins__VectorAtPositionDisplay_t {
    QByteArrayData data[8];
    char stringdata0[164];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_kindr_rviz_plugins__VectorAtPositionDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_kindr_rviz_plugins__VectorAtPositionDisplay_t qt_meta_stringdata_kindr_rviz_plugins__VectorAtPositionDisplay = {
    {
QT_MOC_LITERAL(0, 0, 43), // "kindr_rviz_plugins::VectorAtP..."
QT_MOC_LITERAL(1, 44, 28), // "updateVectorAtPositionSignal"
QT_MOC_LITERAL(2, 73, 0), // ""
QT_MOC_LITERAL(3, 74, 11), // "updateScale"
QT_MOC_LITERAL(4, 86, 14), // "updateShowText"
QT_MOC_LITERAL(5, 101, 19), // "updateColorAndAlpha"
QT_MOC_LITERAL(6, 121, 19), // "updateHistoryLength"
QT_MOC_LITERAL(7, 141, 22) // "updateVectorAtPosition"

    },
    "kindr_rviz_plugins::VectorAtPositionDisplay\0"
    "updateVectorAtPositionSignal\0\0updateScale\0"
    "updateShowText\0updateColorAndAlpha\0"
    "updateHistoryLength\0updateVectorAtPosition"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_kindr_rviz_plugins__VectorAtPositionDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   45,    2, 0x08 /* Private */,
       4,    0,   46,    2, 0x08 /* Private */,
       5,    0,   47,    2, 0x08 /* Private */,
       6,    0,   48,    2, 0x08 /* Private */,
       7,    0,   49,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void kindr_rviz_plugins::VectorAtPositionDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<VectorAtPositionDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateVectorAtPositionSignal(); break;
        case 1: _t->updateScale(); break;
        case 2: _t->updateShowText(); break;
        case 3: _t->updateColorAndAlpha(); break;
        case 4: _t->updateHistoryLength(); break;
        case 5: _t->updateVectorAtPosition(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (VectorAtPositionDisplay::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VectorAtPositionDisplay::updateVectorAtPositionSignal)) {
                *result = 0;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject kindr_rviz_plugins::VectorAtPositionDisplay::staticMetaObject = { {
    &rviz::MessageFilterDisplay<kindr_msgs::VectorAtPosition>::staticMetaObject,
    qt_meta_stringdata_kindr_rviz_plugins__VectorAtPositionDisplay.data,
    qt_meta_data_kindr_rviz_plugins__VectorAtPositionDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *kindr_rviz_plugins::VectorAtPositionDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *kindr_rviz_plugins::VectorAtPositionDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_kindr_rviz_plugins__VectorAtPositionDisplay.stringdata0))
        return static_cast<void*>(this);
    return rviz::MessageFilterDisplay<kindr_msgs::VectorAtPosition>::qt_metacast(_clname);
}

int kindr_rviz_plugins::VectorAtPositionDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::MessageFilterDisplay<kindr_msgs::VectorAtPosition>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void kindr_rviz_plugins::VectorAtPositionDisplay::updateVectorAtPositionSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
