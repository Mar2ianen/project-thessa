#[tokio::main]
async fn main() {
    println!(
        "Project Thessa server scaffold; protocol v{}, epoch time {:.1} s",
        thessa_protocol::ProtocolVersion::CURRENT.0,
        thessa_sim_core::SimTime::EPOCH.seconds(),
    );
}
