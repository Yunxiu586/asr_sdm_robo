import QtQuick
import QtQuick.Controls

Rectangle {
    id: root

    property string label: ""
    property string text: label
    property bool selected: false
    property bool checked: selected
    property bool primary: false
    property var appPalette
    property int radiusValue: 10
    property int labelPixelSize: 16

    signal clicked

    radius: radiusValue
    implicitHeight: 44
    implicitWidth: 140
    opacity: root.enabled ? 1.0 : 0.55
    color: (root.selected || root.checked || root.primary) ? root.appPalette.accent : root.appPalette.controlBackground
    border.color: (root.selected || root.checked || root.primary) ? root.appPalette.accentBorder : root.appPalette.controlBorder
    border.width: 1

    Text {
        anchors.centerIn: parent
        width: Math.max(0, parent.width - 16)
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
        text: root.text.length > 0 ? root.text : root.label
        font.pixelSize: root.labelPixelSize
        font.bold: root.selected || root.checked || root.primary
        color: (root.selected || root.checked || root.primary) ? root.appPalette.accentText : root.appPalette.textPrimary
    }

    MouseArea {
        anchors.fill: parent
        enabled: root.enabled
        hoverEnabled: true
        cursorShape: root.enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
        onClicked: root.clicked()
    }
}
