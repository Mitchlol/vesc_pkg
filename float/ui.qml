/*
    Copyright 2022 Benjamin Vedder	benjamin@vedder.se

    This file is part of VESC Tool.

    VESC Tool is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VESC Tool is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.3

import Vedder.vesc.utility 1.0
import Vedder.vesc.commands 1.0
import Vedder.vesc.configparams 1.0

// This example shows how to read and write settings using the custom
// config. It is also possible to send and receive custom data using
// send_app_data and set_app_data_handler on the euc-side and Commands
// onCustomAppDataReceived and mCommands.sendCustomAppData in qml.

Item {
    id: mainItem
    anchors.fill: parent
    anchors.margins: 5

    property Commands mCommands: VescIf.commands()
    property ConfigParams mMcConf: VescIf.mcConfig()
    property ConfigParams mCustomConf: VescIf.customConfig(0)

    property var dialogParent: ApplicationWindow.overlay
    
    Component.onCompleted: {
//        params.addEditorCustom("pid_mode", 0)
//        params.addEditorCustom("kp", 0)
//        params.addEditorCustom("fault_delay_pitch", 0)
    }
    
    Timer {
        running: true
        repeat: true
        interval: 100
        
        onTriggered: {
            var buffer = new ArrayBuffer(2)
            var dv = new DataView(buffer)
            var ind = 0
            dv.setUint8(ind, 101); ind += 1
            dv.setUint8(ind, 0x1); ind += 1
            mCommands.sendCustomAppData(buffer)
        }
    }
    
    Connections {
        target: mCommands
        
        // This function will be called when VESC_IF->send_app_data is used. To
        // send data back mCommands.sendCustomAppData can be used. That data
        // will be received in the function registered with VESC_IF->set_app_data_handler
        onCustomAppDataReceived: {
            // Ints and floats can be extracted like this from the data
            var dv = new DataView(data, 0)
            var ind = 0
            var magicnr = dv.getUint8(ind); ind += 1;
            var msgtype = dv.getUint8(ind); ind += 1;

            if (magicnr != 101) {
                return;
            }
            if (msgtype == 1) {
                var pid_value = dv.getFloat32(ind); ind += 4;
                var pitch = dv.getFloat32(ind); ind += 4;
                var roll = dv.getFloat32(ind); ind += 4;
                var state = dv.getInt8(ind); ind += 1;
                var switch_state = dv.getInt8(ind); ind += 1;
                var adc1 = dv.getFloat32(ind); ind += 4;
                var adc2 = dv.getFloat32(ind); ind += 4;

                var float_setpoint = dv.getFloat32(ind); ind += 4;
                var float_atr = dv.getFloat32(ind); ind += 4;
                var float_braketilt = dv.getFloat32(ind); ind += 4;
                var float_torquetilt = dv.getFloat32(ind); ind += 4;
                var float_turntilt = dv.getFloat32(ind); ind += 4;
                var float_inputtilt = dv.getFloat32(ind); ind += 4;

                var true_pitch = dv.getFloat32(ind); ind += 4;
                var filtered_current = dv.getFloat32(ind); ind += 4;
                var float_acc_diff = dv.getFloat32(ind); ind += 4;
                var applied_booster_current = dv.getFloat32(ind); ind += 4;
                var motor_current = dv.getFloat32(ind); ind += 4;

                // var debug1 = dv.getFloat32(ind); ind += 4;
                // var debug2 = dv.getFloat32(ind); ind += 4;

                var stateString
                if(state == 0){
                    stateString = "STARTUP"
                }else if(state == 1){
                    stateString = "RUNNING"
                }else if(state == 2){
                    stateString = "RUNNING_TILTBACK"
                }else if(state == 3){
                    stateString = "RUNNING_WHEELSLIP"
                }else if(state == 4){
                    stateString = "RUNNING_UPSIDEDOWN"
                }else if(state == 6){
                    stateString = "STOP_ANGLE_PITCH"
                }else if(state == 7){
                    stateString = "STOP_ANGLE_ROLL"
                }else if(state == 8){
                    stateString = "STOP_SWITCH_HALF"
                }else if(state == 9){
                    stateString = "STOP_SWITCH_FULL"
                }else if(state == 11){
                    if ((roll > 120) || (roll < -120)) {
                        stateString = "STARTUP UPSIDEDOWN"
                    }
                    else {
                        stateString = "STARTUP"
                    }
                }else if(state == 12){
                    stateString = "STOP_REVERSE"
                }else if(state == 13){
                    stateString = "STOP_QUICKSTOP"
                }else{
                    stateString = "UNKNOWN"
                }

                var switchString
                if(switch_state == 0){
                    switchString = "Off"
                }else if(switch_state == 1){
                    switchString = "Half"
                    /*HOW TO ACCESS CONFIG FROM QML???
                    if (adc1 >= VescIf.mcConfig().fault_adc1)
                        switchString += " [On|"
                    else
                        switchString += " [Off|"
                    if (adc2 >= VescIf.mcConfig().fault_adc2)
                        switchString += "On]"
                    else
                        switchString += "Off]"*/
                }else{
                    switchString = "On"
                }

                rt_state.text =
                    "State               : " + stateString + "\n" +
                    "Switch              : " + switchString + "\n"

                rt_data.text =
                    "Current (Requested) : " + pid_value.toFixed(2) + "A\n" +
                    "Current (Motor)     : " + motor_current.toFixed(2) + "A\n" +
                    "Pitch               : " + pitch.toFixed(2) + "°\n" +
                    "Roll                : " + roll.toFixed(2) + "°\n" +
                    "ADC1 / ADC2         : " + adc1.toFixed(2) + "V / " + adc2.toFixed(2) + "V\n"

                setpoints.text =
                    "Setpoint            : " + float_setpoint.toFixed(2) + "°\n" +
                    "ATR Setpoint        : " + float_atr.toFixed(2) + "°\n" +
                    "BrakeTilt Setpoint  : " + float_braketilt.toFixed(2) + "°\n" +
                    "TorqueTilt Setpoint : " + float_torquetilt.toFixed(2) + "°\n" +
                    "TurnTilt Setpoint   : " + float_turntilt.toFixed(2) + "°\n" +
                    "InputTilt Setpoint  : " + float_inputtilt.toFixed(2) + "°\n"

                debug.text =
                    "True Pitch          : " + true_pitch.toFixed(2) + "°\n" +
                    "Torque              : " + filtered_current.toFixed(2) + "A\n" +
                    "Acc. Diff.          : " + float_acc_diff.toFixed(2) + "\n" +
                    "Booster Current     : " + applied_booster_current.toFixed(2) + "A\n"
            }
        }
    }

    ColumnLayout {
        id: root
        anchors.fill: parent
    

        TabBar {
            id: tabBar
            currentIndex: 0
            Layout.fillWidth: true
            clip: true
            enabled: true

            background: Rectangle {
                opacity: 1
                color: {color = Utility.getAppHexColor("lightBackground")}
            }
            property int buttons: 3
            property int buttonWidth: 120

            Repeater {
                model: ["RT Data", "Controls", "Profiles"]
                TabButton {
                    text: modelData
                    onClicked:{
                        stackLayout.currentIndex = index
                    }
                }
            }
        }
        
        StackLayout {
            id: stackLayout
            Layout.fillWidth: true
            Layout.fillHeight: true
            // onCurrentIndexChanged: {tabBar.currentIndex = currentIndex

            ColumnLayout { // RT Data Page
                id: rtDataColumn
                
                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    
                    ColumnLayout {
                        Text {
                            id: header0
                            color: Utility.getAppHexColor("lightText")
                            font.family: "DejaVu Sans Mono"
                            Layout.margins: 0
                            Layout.leftMargin: 0
                            Layout.fillWidth: true
                            text: "Float App State"
                            font.underline: true
                            font.weight: Font.Black
                            font.pointSize: 14
                        }
                        Text {
                            id: rt_state
                            color: Utility.getAppHexColor("lightText")
                            font.family: "DejaVu Sans Mono"
                            Layout.margins: 0
                            Layout.leftMargin: 5
                            Layout.preferredWidth: parent.width/3
                            text: "Waiting for RT Data"
                        }
                        Text {
                            id: header1
                            color: Utility.getAppHexColor("lightText")
                            font.family: "DejaVu Sans Mono"
                            Layout.margins: 0
                            Layout.leftMargin: 0
                            Layout.fillWidth: true
                            text: "Float App RT Data"
                            font.underline: true
                            font.weight: Font.Black
                            font.pointSize: 14
                        }
                        Text {
                            id: rt_data
                            color: Utility.getAppHexColor("lightText")
                            font.family: "DejaVu Sans Mono"
                            Layout.margins: 0
                            Layout.leftMargin: 5
                            Layout.preferredWidth: parent.width/3
                            text: "-\n"
                        }
                        Text {
                            id: header2
                            color: Utility.getAppHexColor("lightText")
                            font.family: "DejaVu Sans Mono"
                            Layout.margins: 0
                            Layout.leftMargin: 0
                            Layout.fillWidth: true
                            text: "Setpoints"
                            font.underline: true
                            font.weight: Font.Black
                            font.pointSize: 14
                        }
                        Text {
                            id: setpoints
                            color: Utility.getAppHexColor("lightText")
                            font.family: "DejaVu Sans Mono"
                            Layout.margins: 0
                            Layout.leftMargin: 5
                            Layout.preferredWidth: parent.width/3
                            text: "-\n"
                        }
                        Text {
                            id: header3
                            color: Utility.getAppHexColor("lightText")
                            font.family: "DejaVu Sans Mono"
                            Layout.margins: 0
                            Layout.leftMargin: 0
                            Layout.fillWidth: true
                            text: "DEBUG"
                            font.underline: true
                            font.weight: Font.Black
                        }
                        Text {
                            id: debug
                            color: Utility.getAppHexColor("lightText")
                            font.family: "DejaVu Sans Mono"
                            Layout.margins: 0
                            Layout.leftMargin: 5
                            Layout.preferredWidth: parent.width/3
                            text: "-"
                        }
                    }
                }
            }

            ColumnLayout { // Controls Page
                id: controlsColumn
                Layout.fillWidth: true

                // Movement controls
                Text {
                    id: movementControlsHeader
                    color: Utility.getAppHexColor("lightText")
                    font.family: "DejaVu Sans Mono"
                    Layout.margins: 0
                    Layout.leftMargin: 0
                    Layout.fillWidth: true
                    text: "Movement Controls"
                    font.underline: true
                    font.weight: Font.Black
                    font.pointSize: 14
                }
                RowLayout {
                    id: movementStrength
                    Layout.fillWidth: true

                    Text {
                        id: movementStrengthLabel
                        color: Utility.getAppHexColor("lightText")
                        font.family: "DejaVu Sans Mono"
                        text: "Strength:"
                    }
                    Slider {
                        id: movementStrengthSlider
                        from: 20
                        value: 40
                        to: 250
                        stepSize: 1
                    }
                }
                RowLayout {
                    id: movementControls
                    Layout.fillWidth: true
                    Button {
                        id: reverseButton
                        text: "Reverse"
                        Layout.fillWidth: true
                        onClicked: {
                            var buffer = new ArrayBuffer(6)
                            var dv = new DataView(buffer)
                            var ind = 0
                            dv.setUint8(ind, 101); ind += 1; // Float Package
                            dv.setUint8(ind, 7); ind += 1; // Command ID: RC Move
                            dv.setUint8(ind, 0); ind += 1; // Direction
                            dv.setUint8(ind, movementStrengthSlider.value); ind += 1; // Current
                            dv.setUint8(ind, 5); ind += 1; // Time
                            dv.setUint8(ind, movementStrengthSlider.value + 5); ind += 1; // Sum = time + current
                            mCommands.sendCustomAppData(buffer)
                        }
                    }
                    Button {
                        id: forwardButton
                        text: "Forward"
                        Layout.fillWidth: true
                        onClicked: {
                            var buffer = new ArrayBuffer(6)
                            var dv = new DataView(buffer)
                            var ind = 0
                            dv.setUint8(ind, 101); ind += 1; // Float Package
                            dv.setUint8(ind, 7); ind += 1; // Command ID: RC Move
                            dv.setUint8(ind, 1); ind += 1; // Direction
                            dv.setUint8(ind, movementStrengthSlider.value); ind += 1; // Current
                            dv.setUint8(ind, 5); ind += 1; // Time
                            dv.setUint8(ind, movementStrengthSlider.value + 5); ind += 1; // Sum = time + current
                            mCommands.sendCustomAppData(buffer)
                        }
                    }
                }
                
                // Tilt controls
                Text {
                    id: tiltControlsHeader
                    color: Utility.getAppHexColor("lightText")
                    font.family: "DejaVu Sans Mono"
                    Layout.margins: 0
                    Layout.leftMargin: 0
                    Layout.fillWidth: true
                    text: "Tilt Controls (In Dev)"
                    font.underline: true
                    font.weight: Font.Black
                    font.pointSize: 14
                }

                Slider {
                    id: tiltSlider
                    from: -1
                    value: 0
                    to: 1
                    Layout.fillWidth: true
                    
                    onMoved: {
                        // var chukData = chuck_data{
                        //     "js_x": 0,
                        //     "js_y": 0,
                        //     "acc_x": 0,
                        //     "acc_y": 0,
                        //     "acc_z": 0,
                        //     "bt_c": false,
                        //     "bt_z": false
                        // }
                        // mCommands.setChukData(chukData)

                        // VByteArray vb;
                        // vb.vbAppendInt8(COMM_SET_CHUCK_DATA);
                        // vb.vbAppendUint8(data.js_x);
                        // vb.vbAppendUint8(data.js_y);
                        // vb.vbAppendUint8(data.bt_c);
                        // vb.vbAppendUint8(data.bt_z);
                        // vb.vbAppendInt16(data.acc_x);
                        // vb.vbAppendInt16(data.acc_y);
                        // vb.vbAppendInt16(data.acc_z);
                        // emitData(vb);

                        //var buffer = new ArrayBuffer(11)
                        //var dv = new DataView(buffer)
                        //var ind = 0
                        //dv.setInt8(ind, 35); ind += 1; // COMM_SET_CHUCK_DATA
                        //dv.setUint8(ind, 0); ind += 1; // js_x
                        //dv.setUint8(ind, 0); ind += 1; // js_y
                        //dv.setUint8(ind, 0); ind += 1; // bt_c
                        //dv.setUint8(ind, 0); ind += 1; // bt_z
                        //dv.setInt16(ind, 0); ind += 2; // acc_x
                        //dv.setInt16(ind, 0); ind += 2; // acc_y
                        //dv.setInt16(ind, 0); ind += 2; // acc_z
                        //mCommands.dataToSend(buffer)
                    }
                }
            }

            ColumnLayout { // Profiles Page
                id: profilesColumn
                
                Button {
                    id: tuneButtonMitch
                    text: "Apply Mitch's Tune"
                    Layout.fillWidth: true
                    onClicked: {
                        // mAppConf.updateParamDouble("imu_conf.accel_confidence_decay", 0.02)
                        // mAppConf.updateParamDouble("imu_conf.mahony_kp", 2.3)
                        // mCommands.setAppConfNoStore()

                        // mCustomConf.updateParamFloat("float_version", 0)
                        mCustomConf.updateParamDouble("kp", 20)
                        mCustomConf.updateParamDouble("ki", 0)
                        mCustomConf.updateParamDouble("kd", 0)
                        mCustomConf.updateParamDouble("kp2", 1.3)
                        mCustomConf.updateParamDouble("mahony_kp", 2.3)
                        // mCustomConf.updateParamUInt16("hertz", 400)
                        // mCustomConf.updateParamDouble(float "fault_pitch", 0)
                        // mCustomConf.updateParamDouble(float "fault_roll", 0)
                        // mCustomConf.updateParamDouble(float "fault_adc1", 0)
                        // mCustomConf.updateParamDouble(float "fault_adc2", 0)
                        // mCustomConf.updateParamDouble(uint16_t "fault_delay_pitch", 0)
                        // mCustomConf.updateParamDouble(uint16_t "fault_delay_roll", 0)
                        // mCustomConf.updateParamDouble(uint16_t "fault_delay_switch_half", 0)
                        // mCustomConf.updateParamDouble(uint16_t "fault_delay_switch_full", 0)
                        // mCustomConf.updateParamDouble(uint16_t "fault_adc_half_erpm", 0)
                        // mCustomConf.updateParamDouble(bool "fault_is_dual_switch", 0)
                        // mCustomConf.updateParamDouble(bool "fault_moving_fault_disabled", 0)
                        // mCustomConf.updateParamDouble(bool "fault_darkride_enabled", 0)
                        // mCustomConf.updateParamDouble(bool "fault_reversestop_enabled", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_duty_angle", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_duty_speed", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_duty", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_hv_angle", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_hv_speed", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_hv", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_lv_angle", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_lv_speed", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_lv", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_return_speed", 0)
                        mCustomConf.updateParamDouble("tiltback_constant", 0)
                        // mCustomConf.updateParamDouble(uint16_t "tiltback_constant_erpm", 0)
                        mCustomConf.updateParamDouble("tiltback_variable", 0)
                        // mCustomConf.updateParamDouble(float "tiltback_variable_max", 0)
                        // mCustomConf.updateParamDouble(FLOAT_INPUTTILT_REMOTE_TYPE "inputtilt_remote_type", 0)
                        // mCustomConf.updateParamDouble(float "inputtilt_speed", 0)
                        // mCustomConf.updateParamDouble(float "inputtilt_angle_limit", 0)
                        // mCustomConf.updateParamDouble(bool "inputtilt_invert_throttle", 0)
                        // mCustomConf.updateParamDouble(float "inputtilt_deadband", 0)
                        // mCustomConf.updateParamDouble(float "noseangling_speed", 0)
                        // mCustomConf.updateParamDouble(float "startup_pitch_tolerance", 0)
                        // mCustomConf.updateParamDouble(float "startup_roll_tolerance", 0)
                        // mCustomConf.updateParamDouble(float "startup_speed", 0)
                        // mCustomConf.updateParamDouble(float "startup_click_current", 0)
                        // mCustomConf.updateParamDouble(bool "startup_softstart_enabled", 0)
                        // mCustomConf.updateParamDouble(bool "startup_simplestart_enabled", 0)
                        // mCustomConf.updateParamDouble(bool "startup_pushstart_enabled", 0)
                        // mCustomConf.updateParamDouble(bool "startup_dirtylandings_enabled", 0)
                        // mCustomConf.updateParamDouble(float "brake_current", 0)
                        // mCustomConf.updateParamDouble(float "ki_limit", 0)
                        // mCustomConf.updateParamDouble(float "booster_angle", 0)
                        // mCustomConf.updateParamDouble(float "booster_ramp", 0)
                        mCustomConf.updateParamDouble("booster_current", 0)
                        // mCustomConf.updateParamDouble("torquetilt_start_current", 0)
                        // mCustomConf.updateParamDouble("torquetilt_angle_limit", 0)
                        // mCustomConf.updateParamDouble("torquetilt_on_speed", 0)
                        // mCustomConf.updateParamDouble("torquetilt_off_speed", 0)
                        mCustomConf.updateParamDouble("torquetilt_strength", 0)
                        // mCustomConf.updateParamDouble("torquetilt_strength_regen", 0)
                        mCustomConf.updateParamDouble("atr_strength_up", 0)
                        mCustomConf.updateParamDouble("atr_strength_down", 0)
                        mCustomConf.updateParamDouble("atr_torque_offset", 0)
                        mCustomConf.updateParamDouble("atr_speed_boost", 0)
                        mCustomConf.updateParamDouble("atr_angle_limit", 0)
                        mCustomConf.updateParamDouble("atr_on_speed", 0)
                        mCustomConf.updateParamDouble("atr_off_speed", 0)
                        mCustomConf.updateParamDouble("atr_response_boost", 0)
                        mCustomConf.updateParamDouble("atr_transition_boost", 0)
                        mCustomConf.updateParamDouble("atr_filter", 0)
                        mCustomConf.updateParamDouble("atr_amps_accel_ratio", 0)
                        mCustomConf.updateParamDouble("atr_amps_decel_ratio", 0)
                        mCustomConf.updateParamDouble("braketilt_strength", 0)
                        // mCustomConf.updateParamDouble("braketilt_lingering", 0)
                        mCustomConf.updateParamDouble("turntilt_strength", 0)
                        // mCustomConf.updateParamDouble("turntilt_angle_limit", 0)
                        // mCustomConf.updateParamDouble("turntilt_start_angle", 0)
                        // mCustomConf.updateParamDouble(uint16_t "turntilt_start_erpm", 0)
                        // mCustomConf.updateParamDouble(float "turntilt_speed", 0)
                        // mCustomConf.updateParamDouble(uint16_t "turntilt_erpm_boost", 0)
                        // mCustomConf.updateParamDouble(uint16_t "turntilt_erpm_boost_end", 0)
                        // mCustomConf.updateParamDouble(int "turntilt_yaw_aggregate", 0)
                        // mCustomConf.updateParamDouble(bool "is_buzzer_enabled", 0)
                        mCommands.customConfigSet(0, mCustomConf)
                    }
                }
                
            }
        }
    }

    


//        RowLayout {
//            Layout.fillWidth: true
//            
//            Button {
//                text: "Read"
//                Layout.fillWidth: true
//                
//                onClicked: {
//                    mCommands.customConfigGet(0, false)
//                }
//            }
//            
//            Button {
//                text: "Read Default"
//                Layout.fillWidth: true
//                
//                onClicked: {
//                    mCommands.customConfigGet(0, true)
//                }
//            }
//            
//            Button {
//                text: "Write"
//                Layout.fillWidth: true
//                
//                onClicked: {
//                    mCommands.customConfigSet(0, mCustomConf)
//                }
//            }
//        }
}
