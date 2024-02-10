mod p9n_interface;
mod ps5_dualsense;

use drobo_interfaces::srv::{SolenoidStateSrv_Request, SolenoidStateSrv_Response};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{sensor_msgs, std_msgs},
    pr_fatal, pr_info,
    selector::Selector,
    topic::{publisher::Publisher, subscriber::Subscriber},
    RecvResult,
};
use std::{rc::Rc, time::Duration};

pub mod DualsenseState {
    pub const SQUARE: usize = 0;
    pub const CIRCLE: usize = 1;
    pub const TRIANGLE: usize = 2;
    pub const CROSS: usize = 3;
    pub const L1: usize = 4;
    pub const L2: usize = 5;
    pub const R1: usize = 6;
    pub const R2: usize = 7;
    pub const D_PAD_UP: usize = 8;
    pub const D_PAD_DOWN: usize = 9;
    pub const D_PAD_LEFT: usize = 10;
    pub const D_PAD_RIGHT: usize = 11;
    pub const START: usize = 12;
    pub const SELECT: usize = 13;
    pub const PS: usize = 14;
}

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("controller_b", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let selector_client = ctx.create_selector()?;
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;

    
    worker(
        selector,
        subscriber
        pubrisher
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    mut selector_client: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let logger = Rc::new(Logger::new("controller_b"));
   
    
    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_r2(){
                send_speed(0x04,true,50,0,&publisher)
                send_speed(0x05,false,50,0,&publisher)
            }
            if !p9n.pressed_r2(){
                send_speed(0x04,true,0,0,&publisher)
                send_speed(0x05,false,0,0,&publisher)

            }
            
            if p9n.pressed_r1(){
                send_speed(0x06,false,50,90,&publisher)
            }
            
            
            
           
        }),
    );
    loop {
        selector.wait()?;
    }

}
fn send_speed(_address:u32, _semi_id:u32,_phase:bool,_speed:u32,_angle:i32,publisher:&Publisher<MdLibMsg>){
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _address as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = 3 as u8; //MotorLibのspeedモードに倣いました
    msg.phase = _phase as bool;
    msg.power = _speed as u16;
    msg.angle = _angle as i32;

    publisher.send(&msg).unwrap()

}

