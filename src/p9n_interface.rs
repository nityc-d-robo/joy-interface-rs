use safe_drive::msg::common_interfaces::sensor_msgs;
use crate::ps5_dualsense::AXES_DUALSENSE;
use crate::ps5_dualsense::BUTTONS_DUALSENSE;

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
    pub fn pressed_l1(&self) -> bool {
        self.msg.buttons.as_slice()[BUTTONS_DUALSENSE::L1] == 1
    }
    pub fn pressed_r1(&self) -> bool {
        self.msg.buttons.as_slice()[BUTTONS_DUALSENSE::R1] == 1
    }
    pub fn pressed_l2(&self) -> bool {
        self.msg.buttons.as_slice()[BUTTONS_DUALSENSE::L2] == 1
    }
    pub fn pressed_r2(&self) -> bool {
        self.msg.buttons.as_slice()[BUTTONS_DUALSENSE::R2] == 1
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
    pub fn pressed_l2_analog(&self) -> f32 {
        self.msg.axes.as_slice()[AXES_DUALSENSE::L2]
    }
    pub fn pressed_r2_analog(&self) -> f32 {
        self.msg.axes.as_slice()[AXES_DUALSENSE::R2]
    }
}