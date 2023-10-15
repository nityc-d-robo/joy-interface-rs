mod p9n_interface;
mod ps5_dualsense;

use std::{time::Duration, rc::Rc};
use drobo_interfaces::srv::{SolenoidStateSrv_Request, SolenoidStateSrv_Response};
use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::sensor_msgs,
    pr_fatal, pr_info, selector::Selector, service::client::Client, topic::subscriber::Subscriber,
    RecvResult,
};

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("controller_2023", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let selector_client = ctx.create_selector()?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;
    let client = node.create_client::<drobo_interfaces::srv::SolenoidStateSrv>(
        "solenoid_order",
        Default::default(),
    )?;

    worker(selector, selector_client, subscriber, client)?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    mut selector_client: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    client: Client<drobo_interfaces::srv::SolenoidStateSrv>,
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let mut client = Some(client);
    let logger = Rc::new(Logger::new("controller_2023"));
    let logger2 = logger.clone();
    let mut dualsense_state: [bool; 15] = [false; 15];
    let mut solenoid_state: [bool; 3] = [false; 3];
    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_dpad_left() && !dualsense_state[ps5_dualsense::JoystickState::D_PAD_LEFT]
            {
                let c = client.take().unwrap();
                let mut request = drobo_interfaces::srv::SolenoidStateSrv_Request::new().unwrap();
                request.axle_position = 0;
                request.state = solenoid_state[0] ^ true;
                let receiver = c.send(&request).unwrap();
                match receiver.recv_timeout(Duration::from_millis(200), &mut selector_client) {
                    RecvResult::Ok((c, _response, _header)) => {
                        on_solenoid_service_received(&logger2, &request, &_response, &mut solenoid_state, &mut dualsense_state);
                        client = Some(c);
                    }
                    RecvResult::RetryLater(r) => client = Some(r.give_up()),
                    RecvResult::Err(e) => {
                        pr_fatal!(logger, "{e}");
                        panic!()
                    }
                }
            }
            if !p9n.pressed_dpad_left() && dualsense_state[ps5_dualsense::JoystickState::D_PAD_LEFT]
            {
                dualsense_state[ps5_dualsense::JoystickState::D_PAD_LEFT] = false;
            }

            if p9n.pressed_dpad_up() && !dualsense_state[ps5_dualsense::JoystickState::D_PAD_UP]
            {
                let c = client.take().unwrap();
                let mut request = drobo_interfaces::srv::SolenoidStateSrv_Request::new().unwrap();
                request.axle_position = 1;
                request.state = solenoid_state[1] ^ true;
                let receiver = c.send(&request).unwrap();
                match receiver.recv_timeout(Duration::from_millis(200), &mut selector_client) {
                    RecvResult::Ok((c, _response, _header)) => {
                        on_solenoid_service_received(&logger2, &request, &_response, &mut solenoid_state, &mut dualsense_state);
                        client = Some(c);
                    }
                    RecvResult::RetryLater(r) => client = Some(r.give_up()),
                    RecvResult::Err(e) => {
                        pr_fatal!(logger, "{e}");
                        panic!()
                    }
                }
            }
            if !p9n.pressed_dpad_up() && dualsense_state[ps5_dualsense::JoystickState::D_PAD_UP]
            {
                dualsense_state[ps5_dualsense::JoystickState::D_PAD_UP] = false;
            }

            if p9n.pressed_dpad_right() && !dualsense_state[ps5_dualsense::JoystickState::D_PAD_RIGHT]
            {
                let c = client.take().unwrap();
                let mut request = drobo_interfaces::srv::SolenoidStateSrv_Request::new().unwrap();
                request.axle_position = 2;
                request.state = solenoid_state[2] ^ true;
                let receiver = c.send(&request).unwrap();
                match receiver.recv_timeout(Duration::from_millis(200), &mut selector_client) {
                    RecvResult::Ok((c, _response, _header)) => {
                        on_solenoid_service_received(&logger2, &request, &_response, &mut solenoid_state, &mut dualsense_state);
                        client = Some(c);
                    }
                    RecvResult::RetryLater(r) => client = Some(r.give_up()),
                    RecvResult::Err(e) => {
                        pr_fatal!(logger, "{e}");
                        panic!()
                    }
                }
            }
            if !p9n.pressed_dpad_right() && dualsense_state[ps5_dualsense::JoystickState::D_PAD_RIGHT]
            {
                dualsense_state[ps5_dualsense::JoystickState::D_PAD_RIGHT] = false;
            }
        }),
    );
    loop {
        selector.wait()?;
    }
}

fn on_solenoid_service_received(logger: &Rc<Logger>, request: &SolenoidStateSrv_Request, response: &SolenoidStateSrv_Response, solenoid_state: &mut [bool; 3], dualsense_state: &mut [bool; 15]){
    pr_info!(logger, "{}番{} {}", request.axle_position, if request.state {"上昇"} else {"下降"},if response.result { "許可" } else { "却下" });
    if response.result{
        solenoid_state[request.axle_position as usize] = request.state;
    }
    dualsense_state[match request.axle_position {
        0 => ps5_dualsense::JoystickState::D_PAD_LEFT,
        1 => ps5_dualsense::JoystickState::D_PAD_UP,
        2 => ps5_dualsense::JoystickState::D_PAD_RIGHT,
        _ => panic!()
    }] = true;
    pr_info!(logger, "現在のソレノイド状態: {:?}", solenoid_state);
}