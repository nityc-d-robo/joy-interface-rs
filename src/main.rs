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
    service::client::Client,
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
    let node = ctx.create_node("controller_2023", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let selector_client = ctx.create_selector()?;
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;

    let demeter_publisher = node.create_publisher::<std_msgs::msg::Int8>("demeter_oracle", None)?;
    let support_wheel_publisher =
        node.create_publisher::<std_msgs::msg::Int32>("support_drive_topic", None)?;
    let client = node.create_client::<drobo_interfaces::srv::SolenoidStateSrv>(
        "solenoid_order",
        Default::default(),
    )?;

    worker(
        selector,
        selector_client,
        subscriber,
        demeter_publisher,
        support_wheel_publisher,
        client,
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    mut selector_client: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    demeter_publisher: Publisher<std_msgs::msg::Int8>,
    support_wheel_publisher: Publisher<std_msgs::msg::Int32>,
    client: Client<drobo_interfaces::srv::SolenoidStateSrv>,
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let mut client = Some(client);
    let logger = Rc::new(Logger::new("controller_2023"));
    let logger2 = logger.clone();
    let mut dualsense_state: [bool; 15] = [false; 15];
    let mut support_wheel_prioritize = 0; // 1: 前, -1: 後ろ
    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_l1() && !dualsense_state[DualsenseState::L1] {
                pr_info!(logger, "収穫機構: 上昇！");
                dualsense_state[DualsenseState::L1] = true;
                let mut msg = std_msgs::msg::Int8::new().unwrap();
                msg.data = 1;
                demeter_publisher.send(&msg).unwrap();
            }
            if !p9n.pressed_l1() && dualsense_state[DualsenseState::L1] {
                dualsense_state[DualsenseState::L1] = false;
                if !p9n.pressed_r1() {
                    pr_info!(logger, "収穫機構: ストップ！");
                    let mut msg = std_msgs::msg::Int8::new().unwrap();
                    msg.data = 0;
                    demeter_publisher.send(&msg).unwrap();
                }
            }
            if p9n.pressed_r1() && !dualsense_state[DualsenseState::R1] {
                pr_info!(logger, "収穫機構: 下降！");
                dualsense_state[DualsenseState::R1] = true;
                let mut msg = std_msgs::msg::Int8::new().unwrap();
                msg.data = -1;
                demeter_publisher.send(&msg).unwrap();
            }
            if !p9n.pressed_r1() && dualsense_state[DualsenseState::R1] {
                dualsense_state[DualsenseState::R1] = false;
                if !p9n.pressed_l1() {
                    pr_info!(logger, "収穫機構: ストップ！");
                    let mut msg = std_msgs::msg::Int8::new().unwrap();
                    msg.data = 0;
                    demeter_publisher.send(&msg).unwrap();
                }
            }

            if p9n.pressed_l2() {
                if (dualsense_state[DualsenseState::R2] && !dualsense_state[DualsenseState::L2]) {
                    support_wheel_prioritize = 1;
                }
                if support_wheel_prioritize >= 0 {
                    dualsense_state[DualsenseState::L2] = true;
                    pr_info!(logger, "中央輪動力: 前");
                    let mut msg = std_msgs::msg::Int32::new().unwrap();
                    msg.data = ((p9n.pressed_l2_analog() - 1.0) * 500.0) as i32;
                    support_wheel_publisher.send(&msg).unwrap();
                }
            }
            if !p9n.pressed_l2() && dualsense_state[DualsenseState::L2] {
                dualsense_state[DualsenseState::L2] = false;
                if !p9n.pressed_r2() {
                    support_wheel_prioritize = 0;
                    pr_info!(logger, "中央輪動力: ストップ");
                    let mut msg = std_msgs::msg::Int32::new().unwrap();
                    msg.data = 0;
                    support_wheel_publisher.send(&msg).unwrap();
                }
            }
            if p9n.pressed_r2() {
                if (dualsense_state[DualsenseState::L2] && !dualsense_state[DualsenseState::R2]) {
                    support_wheel_prioritize = -1;
                }
                if (support_wheel_prioritize <= 0) {
                    dualsense_state[DualsenseState::R2] = true;
                    pr_info!(logger, "中央輪動力: 後ろ");
                    let mut msg = std_msgs::msg::Int32::new().unwrap();
                    msg.data = -((p9n.pressed_r2_analog() - 1.0) * 500.0) as i32;
                    support_wheel_publisher.send(&msg).unwrap();
                }
            }
            if !p9n.pressed_r2() && dualsense_state[DualsenseState::R2] {
                dualsense_state[DualsenseState::R2] = false;
                if !p9n.pressed_l2() {
                    support_wheel_prioritize = 0;
                    pr_info!(logger, "中央輪動力: ストップ");
                    let mut msg = std_msgs::msg::Int32::new().unwrap();
                    msg.data = 0;
                    support_wheel_publisher.send(&msg).unwrap();
                }
            }

            if p9n.pressed_dpad_left() && !dualsense_state[DualsenseState::D_PAD_LEFT] {
                let c = client.take().unwrap();
                let mut request = drobo_interfaces::srv::SolenoidStateSrv_Request::new().unwrap();
                request.axle_position = 0;
                let receiver = c.send(&request).unwrap();
                match receiver.recv_timeout(Duration::from_millis(200), &mut selector_client) {
                    RecvResult::Ok((c, _response, _header)) => {
                        on_solenoid_service_received(
                            &logger2,
                            &request,
                            &_response,
                            &mut dualsense_state,
                        );
                        client = Some(c);
                    }
                    RecvResult::RetryLater(r) => client = Some(r.give_up()),
                    RecvResult::Err(e) => {
                        pr_fatal!(logger, "{e}");
                        panic!()
                    }
                }
            }
            if !p9n.pressed_dpad_left() && dualsense_state[DualsenseState::D_PAD_LEFT] {
                dualsense_state[DualsenseState::D_PAD_LEFT] = false;
            }

            if p9n.pressed_dpad_up() && !dualsense_state[DualsenseState::D_PAD_UP] {
                let c = client.take().unwrap();
                let mut request = drobo_interfaces::srv::SolenoidStateSrv_Request::new().unwrap();
                request.axle_position = 1;
                let receiver = c.send(&request).unwrap();
                match receiver.recv_timeout(Duration::from_millis(200), &mut selector_client) {
                    RecvResult::Ok((c, _response, _header)) => {
                        on_solenoid_service_received(
                            &logger2,
                            &request,
                            &_response,
                            &mut dualsense_state,
                        );
                        client = Some(c);
                    }
                    RecvResult::RetryLater(r) => client = Some(r.give_up()),
                    RecvResult::Err(e) => {
                        pr_fatal!(logger, "{e}");
                        panic!()
                    }
                }
            }
            if !p9n.pressed_dpad_up() && dualsense_state[DualsenseState::D_PAD_UP] {
                dualsense_state[DualsenseState::D_PAD_UP] = false;
            }

            if p9n.pressed_dpad_right() && !dualsense_state[DualsenseState::D_PAD_RIGHT] {
                let c = client.take().unwrap();
                let mut request = drobo_interfaces::srv::SolenoidStateSrv_Request::new().unwrap();
                request.axle_position = 2;
                let receiver = c.send(&request).unwrap();
                match receiver.recv_timeout(Duration::from_millis(200), &mut selector_client) {
                    RecvResult::Ok((c, _response, _header)) => {
                        on_solenoid_service_received(
                            &logger2,
                            &request,
                            &_response,
                            &mut dualsense_state,
                        );
                        client = Some(c);
                    }
                    RecvResult::RetryLater(r) => client = Some(r.give_up()),
                    RecvResult::Err(e) => {
                        pr_fatal!(logger, "{e}");
                        panic!()
                    }
                }
            }
            if !p9n.pressed_dpad_right() && dualsense_state[DualsenseState::D_PAD_RIGHT] {
                dualsense_state[DualsenseState::D_PAD_RIGHT] = false;
            }
        }),
    );
    loop {
        selector.wait()?;
    }
}

fn on_solenoid_service_received(
    logger: &Rc<Logger>,
    request: &SolenoidStateSrv_Request,
    response: &SolenoidStateSrv_Response,
    dualsense_state: &mut [bool; 15],
) {
    pr_info!(
        logger,
        "{}番{} {}",
        request.axle_position,
        if response.state[request.axle_position as usize] { "上昇" } else { "下降" },
        if response.result { "許可" } else { "却下" }
    );
    dualsense_state[match request.axle_position {
        0 => DualsenseState::D_PAD_LEFT,
        1 => DualsenseState::D_PAD_UP,
        2 => DualsenseState::D_PAD_RIGHT,
        _ => panic!(),
    }] = true;
    pr_info!(logger, "現在のソレノイド状態: {:?}", response.state);
}
