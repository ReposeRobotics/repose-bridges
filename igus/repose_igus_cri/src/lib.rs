//! Receive-only CRI status bridge for a six-axis igus ReBeL.
//!
//! This host-only crate owns a TCP worker. Copper polls a fixed-size latest-value
//! slot without blocking. The only outgoing frame is the zero-jog heartbeat;
//! the bridge never requests active control. See the crate README for wiring.

use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Time;
use cu29::units::si::time::second;
use serde::Deserialize;
use serde::Serialize;
use std::io::Read;
use std::io::Write;
use std::net::SocketAddr;
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::thread::JoinHandle;
use std::time::Duration;
use std::time::Instant;

const FRAME_CAPACITY: usize = 8192;
const HEARTBEAT: Duration = Duration::from_millis(200);
const POLL: Duration = Duration::from_millis(20);

/// Measured joint angles from `POSJOINTCURRENT`; never inferred velocities.
#[derive(
    Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuRebelState {
    /// Revolute joints A1 through A6, in controller order.
    pub positions: [Angle; 6],
}

/// Connection and freshness parameters, shared by Rust and RON configuration.
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuIgusCriParams {
    /// Numeric socket address of the controller (no DNS lookup).
    pub address: String,
    /// Maximum age of an observation published to Copper.
    pub stale_after: Time,
    /// Startup connection timeout.
    pub connect_timeout: Time,
}

impl Default for CuIgusCriParams {
    fn default() -> Self {
        Self {
            address: "192.168.3.11:3920".into(),
            stale_after: Time::new::<second>(0.5),
            connect_timeout: Time::new::<second>(2.0),
        }
    }
}

impl CuIgusCriParams {
    /// Reject invalid addresses and non-positive or unrepresentable durations.
    pub fn validate(&self) -> CuResult<()> {
        self.address
            .parse::<SocketAddr>()
            .map_err(|e| CuError::new_with_cause("CRI requires a numeric socket address", e))?;
        duration(self.stale_after)?;
        duration(self.connect_timeout)?;
        Ok(())
    }
}

fn duration(value: Time) -> CuResult<Duration> {
    let result = Duration::try_from_secs_f64(value.get::<second>())
        .map_err(|e| CuError::new_with_cause("CRI duration must be finite and positive", e))?;
    if result.is_zero() {
        return Err(CuError::from("CRI duration must be positive"));
    }
    Ok(result)
}

/// Empty transmit set: no command channel can be constructed.
pub struct IgusCriTxChannels;
/// Uninhabited identifier for the receive-only bridge.
pub type IgusCriTxId = core::convert::Infallible;
impl BridgeChannelSet for IgusCriTxChannels {
    type Id = IgusCriTxId;
    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
}
rx_channels! {
    pub struct IgusCriRxChannels: IgusCriRxId {
        joint_state => CuRebelState = "joint_state",
    }
}

#[derive(Default)]
struct Latest {
    sample: Option<(Instant, CuRebelState)>,
    failed: bool,
}

/// Receive-only native Copper bridge. Call `start` before polling `joint_state`.
///
/// A new frame is consumed once. Empty, contended, or stale slots clear the
/// output. Transport/protocol failure returns an error; restart explicitly to
/// reconnect. External I/O is recorded through Copper messages, so bridge
/// freeze/thaw uses the default no-op implementation.
#[derive(Reflect)]
pub struct CuIgusCriBridge {
    params: CuIgusCriParams,
    #[reflect(ignore)]
    latest: Arc<Mutex<Latest>>,
    #[reflect(ignore)]
    shutdown: Arc<AtomicBool>,
    #[reflect(ignore)]
    worker: Option<JoinHandle<()>>,
    #[reflect(ignore)]
    stale_after: Duration,
}

impl CuIgusCriBridge {
    /// Construct without opening a connection; `start` owns connection setup.
    pub fn from_params(params: CuIgusCriParams) -> CuResult<Self> {
        params.validate()?;
        let stale_after = duration(params.stale_after)?;
        Ok(Self {
            params,
            latest: Arc::new(Mutex::new(Latest::default())),
            shutdown: Arc::new(AtomicBool::new(false)),
            worker: None,
            stale_after,
        })
    }

    fn close(&mut self) {
        self.shutdown.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

impl Drop for CuIgusCriBridge {
    fn drop(&mut self) {
        self.close();
    }
}

impl Freezable for CuIgusCriBridge {}

impl CuBridge for CuIgusCriBridge {
    type Tx = IgusCriTxChannels;
    type Rx = IgusCriRxChannels;
    type Resources<'r> = ();

    fn new(
        config: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<IgusCriTxId>],
        _rx: &[BridgeChannelConfig<IgusCriRxId>],
        _resources: (),
    ) -> CuResult<Self> {
        Self::from_params(
            config
                .map(ComponentConfig::deserialize_into)
                .transpose()?
                .unwrap_or_default(),
        )
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.close();
        let address = self
            .params
            .address
            .parse::<SocketAddr>()
            .map_err(|e| CuError::new_with_cause("Invalid CRI address", e))?;
        let stream = TcpStream::connect_timeout(&address, duration(self.params.connect_timeout)?)
            .map_err(|e| CuError::new_with_cause("CRI connection failed", e))?;
        stream
            .set_read_timeout(Some(POLL))
            .map_err(|e| CuError::new_with_cause("CRI read timeout setup failed", e))?;
        stream
            .set_write_timeout(Some(Duration::from_millis(100)))
            .map_err(|e| CuError::new_with_cause("CRI write timeout setup failed", e))?;
        self.latest = Arc::new(Mutex::new(Latest::default()));
        self.shutdown.store(false, Ordering::Release);
        let latest = Arc::clone(&self.latest);
        let shutdown = Arc::clone(&self.shutdown);
        self.worker = Some(
            std::thread::Builder::new()
                .name("igus-cri".into())
                .spawn(move || {
                    let result = run(stream, &latest, &shutdown);
                    if result.is_err()
                        && let Ok(mut slot) = latest.lock()
                    {
                        slot.sample = None;
                        slot.failed = true;
                    }
                })
                .map_err(|e| CuError::new_with_cause("CRI worker startup failed", e))?,
        );
        Ok(())
    }

    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<IgusCriTxId, P>,
        _msg: &CuMsg<P>,
    ) -> CuResult<()> {
        Err(CuError::from("CRI bridge has no transmit channels"))
    }

    fn receive<'a, P: CuMsgPayload + 'a>(
        &mut self,
        ctx: &CuContext,
        _channel: &'static BridgeChannel<IgusCriRxId, P>,
        msg: &mut CuMsg<P>,
    ) -> CuResult<()> {
        let output = msg.downcast_mut::<CuRebelState>()?;
        output.clear_payload();
        output.tov = Tov::None;
        if self.worker.is_none() {
            return Err(CuError::from("CRI bridge is not started"));
        }
        let mut slot = match self.latest.try_lock() {
            Ok(slot) => slot,
            Err(std::sync::TryLockError::WouldBlock) => return Ok(()),
            Err(_) => return Err(CuError::from("CRI worker state poisoned")),
        };
        if slot.failed {
            return Err(CuError::from(
                "CRI transport or protocol failed; restart bridge",
            ));
        }
        if let Some((received, sample)) = slot.sample.take() {
            let age = received.elapsed();
            if age <= self.stale_after {
                // CRI has no measurement timestamp. Estimate host receipt time
                // by subtracting the worker-to-runtime delay from Copper time.
                output.tov = Tov::Time(
                    ctx.now() - CuDuration::from_nanos(age.as_nanos().min(u64::MAX as u128) as u64),
                );
                output.set_payload(sample);
            }
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.close();
        Ok(())
    }
}

fn run(mut stream: TcpStream, latest: &Mutex<Latest>, shutdown: &AtomicBool) -> CuResult<()> {
    let mut frame = [0; FRAME_CAPACITY];
    let mut used = 0;
    let mut heartbeat = None;
    let mut counter = 0u16;
    let mut bytes = [0; 1024];
    while !shutdown.load(Ordering::Acquire) {
        if heartbeat.is_none_or(|last: Instant| last.elapsed() >= HEARTBEAT) {
            // This is the entire transmit vocabulary. Never SetActive or CMD.
            writeln!(
                stream,
                "CRISTART {counter} ALIVEJOG 0 0 0 0 0 0 0 0 0 CRIEND"
            )
            .map_err(|e| CuError::new_with_cause("CRI heartbeat failed", e))?;
            counter = (counter + 1) % 10000;
            heartbeat = Some(Instant::now());
        }
        match stream.read(&mut bytes) {
            Ok(0) => return Err(CuError::from("CRI peer disconnected")),
            Ok(count) => {
                for &byte in &bytes[..count] {
                    if used == frame.len() {
                        return Err(CuError::from("CRI frame exceeds capacity"));
                    }
                    frame[used] = byte;
                    used += 1;
                    if frame[..used].ends_with(b"CRIEND") {
                        let text = core::str::from_utf8(&frame[..used])
                            .map_err(|e| CuError::new_with_cause("Invalid CRI UTF-8", e))?;
                        let mut sample = CuRebelState::default();
                        if decode_status(text, &mut sample)? {
                            let mut slot = latest
                                .lock()
                                .map_err(|_| CuError::from("CRI worker state poisoned"))?;
                            slot.sample = Some((Instant::now(), sample));
                        }
                        used = 0;
                    }
                }
            }
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock
                        | std::io::ErrorKind::TimedOut
                        | std::io::ErrorKind::Interrupted
                ) => {}
            Err(e) => return Err(CuError::new_with_cause("CRI read failed", e)),
        }
    }
    Ok(())
}

fn decode_status(frame: &str, output: &mut CuRebelState) -> CuResult<bool> {
    let mut tokens = frame.split_ascii_whitespace();
    if tokens.next() != Some("CRISTART") {
        return Err(CuError::from("Invalid CRI frame start"));
    }
    tokens
        .next()
        .and_then(|v| v.parse::<u16>().ok())
        .ok_or_else(|| CuError::from("Invalid CRI counter"))?;
    if tokens.next() != Some("STATUS") {
        return Ok(false);
    }
    for token in tokens.by_ref() {
        if token == "POSJOINTCURRENT" {
            let mut positions = [Angle::default(); 6];
            for position in &mut positions {
                let value = tokens
                    .next()
                    .and_then(|v| v.parse::<f64>().ok())
                    .filter(|v| v.is_finite())
                    .ok_or_else(|| CuError::from("Invalid CRI joint position"))?;
                *position = Angle::new::<degree>(value);
            }
            output.positions = positions;
            return Ok(true);
        }
    }
    Err(CuError::from("CRI STATUS missing POSJOINTCURRENT"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    const STATUS: &str = "CRISTART 42 STATUS MODE joint POSJOINTCURRENT 0 90 -90 180 45 -45 0 0 0 0 0 0 0 0 0 0 POSCARTROBOT 0 0 0 0 0 0 CRIEND";

    #[test]
    fn test_status_units_and_validation() {
        let mut output = CuRebelState::default();
        assert!(decode_status(STATUS, &mut output).unwrap());
        assert!((output.positions[1].get::<degree>() - 90.0).abs() < 1e-10);
        assert!(
            decode_status(
                "CRISTART 1 STATUS POSJOINTCURRENT 0 NaN 0 0 0 0 CRIEND",
                &mut output
            )
            .is_err()
        );
        assert!(
            decode_status("CRISTART 1 STATUS POSJOINTCURRENT 0 1 CRIEND", &mut output).is_err()
        );
        assert!(decode_status("CRISTART 1 STATUS MODE joint CRIEND", &mut output).is_err());
        assert!(!decode_status("CRISTART 1 RUNSTATE ignored CRIEND", &mut output).unwrap());
        assert!(
            CuIgusCriParams {
                stale_after: Time::new::<second>(f64::NAN),
                ..Default::default()
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn test_fragmentation_coalescing_and_heartbeat() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let stream = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        stream.set_read_timeout(Some(POLL)).unwrap();
        stream.set_write_timeout(Some(POLL)).unwrap();
        let (mut peer, _) = listener.accept().unwrap();
        peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let latest = Arc::new(Mutex::new(Latest::default()));
        let shutdown = Arc::new(AtomicBool::new(false));
        let slot = Arc::clone(&latest);
        let stop = Arc::clone(&shutdown);
        let worker = std::thread::spawn(move || run(stream, &slot, &stop));
        let mut heartbeat = Vec::new();
        loop {
            let mut byte = [0];
            peer.read_exact(&mut byte).unwrap();
            heartbeat.push(byte[0]);
            if byte[0] == b'\n' {
                break;
            }
        }
        assert_eq!(
            core::str::from_utf8(&heartbeat).unwrap(),
            "CRISTART 0 ALIVEJOG 0 0 0 0 0 0 0 0 0 CRIEND\n"
        );
        peer.write_all(b"CRISTART 0 RUNSTATE ignored CRIEND\nCRISTA")
            .unwrap();
        peer.write_all(&STATUS.as_bytes()[6..]).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while latest.lock().unwrap().sample.is_none() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(
            (latest.lock().unwrap().sample.as_ref().unwrap().1.positions[2].get::<degree>() + 90.0)
                .abs()
                < 1e-10
        );
        shutdown.store(true, Ordering::Release);
        worker.join().unwrap().unwrap();
    }

    #[test]
    fn test_transport_failure_and_frame_capacity() {
        for oversized in [false, true] {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let mut bridge = CuIgusCriBridge::from_params(CuIgusCriParams {
                address: listener.local_addr().unwrap().to_string(),
                ..Default::default()
            })
            .unwrap();
            let (ctx, _) = CuContext::new_mock_clock();
            bridge.start(&ctx).unwrap();
            let (mut peer, _) = listener.accept().unwrap();
            if oversized {
                peer.write_all(&[b'x'; FRAME_CAPACITY + 1]).unwrap();
            } else {
                drop(peer);
            }
            let deadline = Instant::now() + Duration::from_secs(2);
            while !bridge.latest.lock().unwrap().failed {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(2));
            }
            let mut msg = CuMsg::<CuRebelState>::default();
            assert!(
                bridge
                    .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
                    .is_err()
            );
            bridge.stop(&ctx).unwrap();
        }
    }

    #[test]
    fn test_receive_consumes_and_rejects_stale_samples() {
        let mut bridge = CuIgusCriBridge::from_params(Default::default()).unwrap();
        // A parked worker models the live lifecycle without a hardware socket.
        bridge.worker = Some(std::thread::spawn(|| {}));
        let (ctx, _) = CuContext::new_mock_clock();
        let mut msg = CuMsg::<CuRebelState>::default();
        bridge.latest.lock().unwrap().sample = Some((Instant::now(), CuRebelState::default()));
        bridge
            .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
            .unwrap();
        assert!(msg.payload().is_some());
        bridge
            .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
            .unwrap();
        assert!(msg.payload().is_none());
        bridge.latest.lock().unwrap().sample = Some((
            Instant::now() - Duration::from_secs(1),
            CuRebelState::default(),
        ));
        bridge
            .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
            .unwrap();
        assert!(msg.payload().is_none());
        bridge.latest.lock().unwrap().failed = true;
        assert!(
            bridge
                .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
                .is_err()
        );
    }
}
