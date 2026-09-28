//! Stdio and TCP framing/connection lifecycle around the shared sim driver.

use std::io::{Read, Write};
use std::sync::Arc;
use std::time::Duration;

use thessa_protocol::{FrameDecoder, kind};

use super::*;

pub(super) enum DecodedClientMessage {
    Input(ClientInput),
    Guidance(GuidanceInput),
    Autopilot(AutopilotInput),
}

/// Decode the envelope once, then deserialize only the payload selected by
/// its wire kind. Unknown or malformed client messages remain fail-closed.
pub(super) fn decode_client_message(frame: &[u8]) -> Option<DecodedClientMessage> {
    let envelope = thessa_flight_net::decode_frame(frame).ok()?;
    match envelope.kind {
        kind::CLIENT_INPUT => thessa_flight_net::decode_payload::<ClientInput>(&envelope)
            .ok()
            .map(DecodedClientMessage::Input),
        kind::GUIDANCE_COMMAND => thessa_flight_net::decode_payload::<GuidanceInput>(&envelope)
            .ok()
            .map(DecodedClientMessage::Guidance),
        kind::AUTOPILOT_COMMAND => thessa_flight_net::decode_payload::<AutopilotInput>(&envelope)
            .ok()
            .map(DecodedClientMessage::Autopilot),
        _ => None,
    }
}

fn enqueue_client_inputs(
    id: &str,
    frames: impl IntoIterator<Item = Vec<u8>>,
    upstream: &IngressSender,
) -> bool {
    for frame in frames {
        match decode_client_message(&frame) {
            Some(DecodedClientMessage::Input(input)) => {
                if !upstream.send_input(id, input) {
                    return false;
                }
            }
            Some(DecodedClientMessage::Guidance(guidance)) => {
                if !upstream.send_guidance(id, guidance) {
                    return false;
                }
            }
            Some(DecodedClientMessage::Autopilot(autopilot)) => {
                if !upstream.send_autopilot(id, autopilot) {
                    return false;
                }
            }
            None => {}
        }
    }
    true
}

pub(super) fn run_stdio(mut sim: Sim) -> Result<(), String> {
    // Canonical terrain: same site the client survey selects. Soft-fails
    // to terrain-free flight (today's embedded behavior) instead of
    // refusing to serve.
    if let Err(error) = sim.init_launch_site() {
        eprintln!("[server] launch site unavailable ({error}); terrain-free flight");
    }
    // Stdout writer thread: frames in, bytes out. Stdout is the wire.
    // Joined on shutdown after all senders drop, so the tail flushes.
    let wire_out = Arc::new(OutboundMailbox::new());
    let writer_out = wire_out.clone();
    let writer = std::thread::spawn(move || {
        let mut stdout = std::io::stdout().lock();
        while let Some(frame) = writer_out.blocking_next() {
            if stdout.write_all(frame.as_ref()).is_err() {
                break;
            }
        }
        let _ = stdout.flush();
    });

    let (upstream_tx, upstream_rx) = IngressSender::new();
    let local_id = upstream_tx
        .reserve_connection("local")
        .ok_or("could not reserve embedded connection")?;
    if !upstream_tx.mark_active_for_sender(&local_id) {
        return Err("could not activate embedded connection".into());
    }

    // Handshake on the main thread: first frame must be Hello. Frames
    // pipelined after it decode straight into the driver queue.
    let mut decoder = FrameDecoder::new();
    let stdin = std::io::stdin();
    let mut locked = stdin.lock();
    let (welcome, post_handshake_frames) = loop {
        let mut chunk = [0u8; 65536];
        let n = locked.read(&mut chunk).map_err(|e| e.to_string())?;
        if n == 0 {
            return Err("stdin closed before handshake".into());
        }
        let mut frames = decoder.push(&chunk[..n]).map_err(|e| e.to_string())?;
        if frames.is_empty() {
            continue;
        }
        let envelope =
            thessa_flight_net::decode_frame(&frames.remove(0)).map_err(|e| e.to_string())?;
        if envelope.kind != kind::HELLO {
            return Err(format!("expected HELLO, got kind {}", envelope.kind));
        }
        let hello: thessa_flight_net::Hello =
            thessa_flight_net::decode_payload(&envelope).map_err(|e| e.to_string())?;
        eprintln!("[server] hello from {:?}", hello.client_name);
        break (
            thessa_flight_net::Welcome {
                tick: sim.authority.world_tick.0,
                flight_time_s: sim.authority.flight_time_s,
            },
            frames,
        );
    };
    let welcome_frame = thessa_flight_net::encode_welcome(&welcome).map_err(|e| e.to_string())?;
    if !wire_out.push_reliable(welcome_frame) {
        return Err("stdout writer unavailable during handshake".into());
    }
    drop(locked);

    // Input pump thread: stdin bytes -> decoded inputs. EOF becomes a
    // Leave, which ends the driver loop after the queued inputs drain
    // (channel FIFO, no drain race). Own lock: the handshake lock above
    // is dropped, so no buffered byte is stranded between the two.
    let input_id = local_id.clone();
    std::thread::spawn(move || {
        let stdin = std::io::stdin();
        let mut locked = stdin.lock();
        // Keep the decoder that performed the handshake: it may contain the
        // partial prefix/body of the first pipelined input frame.
        let mut decoder = decoder;
        let mut buffer = [0u8; 65536];
        if !enqueue_client_inputs(&input_id, post_handshake_frames, &upstream_tx) {
            let _ = upstream_tx.send_leave(&input_id);
            return;
        }
        loop {
            match locked.read(&mut buffer) {
                Ok(0) => break,
                Ok(n) => match decoder.push(&buffer[..n]) {
                    Ok(frames) => {
                        if !enqueue_client_inputs(&input_id, frames, &upstream_tx) {
                            let _ = upstream_tx.send_leave(&input_id);
                            return;
                        }
                    }
                    Err(error) => {
                        eprintln!("[server] frame error: {error}");
                        break;
                    }
                },
                Err(_) => break,
            }
        }
        let _ = upstream_tx.send_leave(&input_id);
    });

    let mut driver = Driver {
        sim,
        autopilot: AutopilotHost::new()?,
        upstream: upstream_rx,
        subscribers: Vec::new(),
        pacing_target_s: 0.0,
        last_pacing: Instant::now(),
        last_snapshot: Instant::now(),
        last_status: Instant::now(),
        exit_when_empty: true,
    };
    // Local subscription goes through the driver so Welcome/ordering
    // match the TCP path exactly (handshake Welcome was already sent).
    driver.sim.register(&local_id);
    driver.sim.wall_started = Instant::now();
    driver.subscribers.push((local_id, wire_out));
    // Opening snapshot so the client never waits a full interval.
    driver.broadcast_snapshot();
    while !driver.iterate()? {}
    let sim = driver.sim;
    let _ = writer.join();
    eprintln!(
        "[server] done: {:.1} sim-s, {:.1} compute-s, x{:.1} (votes at exit: {}), {} steps ({:.1} rails-s)",
        sim.advanced_s,
        sim.compute_s,
        sim.effective_warp(),
        sim.clients.len(),
        sim.steps,
        sim.rails_s
    );
    Ok(())
}

/// One TCP client: handshake, subscribe, pump inputs, forward snapshots.
/// Ends by voting Leave; the driver prunes the subscriber on it.
async fn handle_tcp_conn(stream: tokio::net::TcpStream, peer: String, upstream: IngressSender) {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    // Small sim frames at 20 Hz: Nagle would hold them up to ~40 ms
    // waiting for ACKs (classic delayed-ACK interplay). QUIC would not
    // have this knob at all; with TCP it must be off explicitly.
    if let Err(error) = stream.set_nodelay(true) {
        eprintln!("[server] tcp nodelay failed: {error}");
        return;
    }
    let (mut reader, mut writer) = stream.into_split();
    let mut decoder = FrameDecoder::new();
    let mut buffer = vec![0u8; 65536];
    // Handshake with a deadline: first frame must be Hello.
    let hello = tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            let n = reader.read(&mut buffer).await.map_err(|e| e.to_string())?;
            if n == 0 {
                return Err("eof before hello".to_string());
            }
            let mut frames = decoder.push(&buffer[..n]).map_err(|e| e.to_string())?;
            if !frames.is_empty() {
                let frame = frames.remove(0);
                return Ok((frame, frames));
            }
        }
    })
    .await;
    let (frame, post_handshake_frames) = match hello {
        Ok(Ok(frames)) => frames,
        _ => return,
    };
    let Ok(envelope) = thessa_flight_net::decode_frame(&frame) else {
        return;
    };
    if envelope.kind != kind::HELLO {
        return;
    }
    let Ok(hello_msg) = thessa_flight_net::decode_payload::<thessa_flight_net::Hello>(&envelope)
    else {
        return;
    };
    // Client id couples the name with the peer so two same-named
    // processes never share a vote.
    let label = format!("{}@{peer}", hello_msg.client_name);
    let Some(id) = upstream.reserve_connection(&label) else {
        eprintln!("[server] client limit reached; rejecting {label}");
        return;
    };
    eprintln!("[server] tcp hello from {label} as {id}");
    let mailbox = Arc::new(OutboundMailbox::new());
    if !upstream.send_subscribe(&id, mailbox.clone()) {
        let _ = upstream.send_leave(&id);
        return;
    }
    // Writer task: subscribed frames -> socket. Ends when the driver
    // prunes us (Leave processed) or the socket breaks.
    let writer_mailbox = mailbox.clone();
    let mut write_task = tokio::spawn(async move {
        while let Some(frame) = writer_mailbox.next().await {
            if writer.write_all(frame.as_ref()).await.is_err() {
                break;
            }
        }
    });
    if !enqueue_client_inputs(&id, post_handshake_frames, &upstream) {
        let _ = upstream.send_leave(&id);
        write_task.abort();
        return;
    }
    // Reader loop: socket -> inputs. Any end votes Leave. The select also
    // aborts this reader when the outbound writer detects a dead peer, so a
    // closed mailbox cannot leave a zombie connection feeding ingress.
    let reader_id = id.clone();
    let reader_upstream = upstream.clone();
    let mut read_task = tokio::spawn(async move {
        loop {
            match reader.read(&mut buffer).await {
                Ok(0) => break,
                Ok(n) => match decoder.push(&buffer[..n]) {
                    Ok(frames) => {
                        if !enqueue_client_inputs(&reader_id, frames, &reader_upstream) {
                            break;
                        }
                    }
                    Err(error) => {
                        eprintln!("[server] tcp frame error from {reader_id}: {error}");
                        break;
                    }
                },
                Err(_) => break,
            }
        }
    });
    tokio::select! {
        _ = &mut read_task => {
            let _ = upstream.send_leave(&id);
            write_task.abort();
        }
        _ = &mut write_task => {
            let _ = upstream.send_leave(&id);
            read_task.abort();
        }
    }
}

pub(super) fn run_tcp(mut sim: Sim, addr: &str) -> Result<(), String> {
    if let Err(error) = sim.init_launch_site() {
        eprintln!("[server] launch site unavailable ({error}); terrain-free flight");
    }
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .map_err(|e| e.to_string())?;
    runtime.block_on(async move {
        let (upstream_tx, upstream_rx) = IngressSender::new();
        // Sim driver thread: blocking sleeps stay off the tokio workers.
        std::thread::spawn(move || {
            let autopilot = match AutopilotHost::new() {
                Ok(autopilot) => autopilot,
                Err(error) => {
                    eprintln!("[server] {error}");
                    return;
                }
            };
            let mut driver = Driver {
                sim,
                autopilot,
                upstream: upstream_rx,
                subscribers: Vec::new(),
                pacing_target_s: 0.0,
                last_pacing: Instant::now(),
                last_snapshot: Instant::now(),
                last_status: Instant::now(),
                exit_when_empty: false,
            };
            driver.sim.wall_started = Instant::now();
            loop {
                if let Err(error) = driver.iterate() {
                    eprintln!("[server] driver error: {error}");
                    break;
                }
            }
        });
        let listener = tokio::net::TcpListener::bind(addr)
            .await
            .map_err(|e| e.to_string())?;
        eprintln!("[server] tcp listening on {addr}");
        loop {
            let (stream, peer) = listener.accept().await.map_err(|e| e.to_string())?;
            eprintln!("[server] tcp accept {peer}");
            let upstream_tx = upstream_tx.clone();
            tokio::spawn(handle_tcp_conn(stream, peer.to_string(), upstream_tx));
        }
    })
}
