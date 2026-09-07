fn main() {
    let _app = bevy::prelude::App::new();
    println!(
        "Project Thessa client scaffold; protocol v{}",
        thessa_protocol::ProtocolVersion::CURRENT.0,
    );
}
