use safe_drive::msg::common_interfaces::sensor_msgs;
use crate::ps5_dualsense::AXES_DUALSENSE;

pub struct PlaystationInterface {
    msg: sensor_msgs::msg::Joy,
}

impl PlaystationInterface {
    pub fn new(_msg: sensor_msgs::msg::Joy) -> PlaystationInterface {
        PlaystationInterface { msg: _msg, }
    }
    pub fn set_joy_msg(&mut self, _msg: sensor_msgs::msg::Joy){
        self.msg = _msg;
    }
    pub fn pressed_dpad_left(&self) -> bool {
         self.msg.axes.as_slice()[AXES_DUALSENSE::DPAD_X] > 0.0
    }
    pub fn pressed_dpad_up(&self) -> bool {
        self.msg.axes.as_slice()[AXES_DUALSENSE::DPAD_Y] > 0.0
    }
    pub fn pressed_dpad_right(&self) -> bool {
        self.msg.axes.as_slice()[AXES_DUALSENSE::DPAD_X] < 0.0
    }
}