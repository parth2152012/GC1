//! Lightweight, dependency-free metrics/event logger.
//!
//! Appends one JSON object per line (JSONL) to a log file so the required Stage-1
//! "standardized log format" and official performance metrics (mission, communication,
//! autonomy, robustness, safety) can be reconstructed after a run, e.g. with a small
//! offline script that aggregates this file.
//!
//! This module intentionally introduces no new crates: it only uses std, serde and
//! serde_json, which are already dependencies, so no `cargo update`/network access is
//! required to build it.

use serde::Serialize;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::OnceLock;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::sync::Mutex;

static LOG_FILE: OnceLock<Mutex<File>> = OnceLock::new();

/// Timestamp (ms since Unix epoch) at which the most recent fault was injected via
/// `mark_fault_injected`. Zero means "no fault currently pending". This is a simple,
/// process-wide clock used to compute the "Recovery time" autonomy metric: the CLI
/// (or an automated test harness) marks a fault, and the orchestrator loop calls
/// `take_recovery_seconds` once the swarm has visibly reconfigured (e.g. the reserve
/// UAV is promoted to LeadSurveyor).
static FAULT_INJECTED_AT_MS: AtomicU64 = AtomicU64::new(0);

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Opens (or creates) the metrics log file. Safe to call multiple times; only the
/// first call takes effect. Call this once near the top of `main()`.
pub fn init(path: &str) {
    if LOG_FILE.get().is_some() {
        return;
    }
    match OpenOptions::new().create(true).append(true).open(path) {
        Ok(file) => {
            let _ = LOG_FILE.set(Mutex::new(file));
            println!("📊 Metrics log initialized at {path}");
        }
        Err(error) => eprintln!("⚠️  Could not open metrics log {path}: {error}"),
    }
}

/// Records that a fault (battery/link failure) was just injected, for later recovery-time
/// measurement.
pub fn mark_fault_injected() {
    FAULT_INJECTED_AT_MS.store(now_ms(), Ordering::Release);
}

/// Consumes the pending fault-injection timestamp, if any, returning the elapsed seconds
/// since it was injected. Returns `None` if no fault is currently pending (already
/// consumed, or never injected).
pub fn take_recovery_seconds() -> Option<f64> {
    let started = FAULT_INJECTED_AT_MS.swap(0, Ordering::AcqRel);
    if started == 0 {
        return None;
    }
    let elapsed_ms = now_ms().saturating_sub(started);
    Some(elapsed_ms as f64 / 1000.0)
}

/// One structured event per required metric category (Mission / Communication /
/// Autonomy / Robustness / Safety) from the Stage-1 rubric, plus a couple of
/// supporting events (photo capture, mesh health) useful for debugging.
#[derive(Serialize)]
#[serde(tag = "event")]
pub enum MetricEvent {
    MissionStart {
        mission_id: u32,
        poi_count: usize,
    },
    MissionProgress {
        mission_id: u32,
        total: usize,
        completed: usize,
        failed: usize,
        elapsed_s: f64,
        /// Sum of priority weights (High=3, Medium=2, Low=1) of completed PoIs, as a
        /// percentage of the sum of priority weights of all PoIs seen so far. Fills the
        /// "Priority-weighted mission score" row that the Report Template leaves blank.
        priority_weighted_score_pct: f32,
        /// Smallest inter-UAV separation observed so far this mission, in meters.
        /// `None` until the reserve has flown at least once alongside the active UAV.
        /// Fills the "Minimum inter-UAV separation" row with a measured value instead
        /// of just the 20 m configured threshold.
        min_separation_m: Option<f64>,
    },
    PoiAssigned {
        mission_id: u32,
        poi_id: u32,
        drone_id: u8,
    },
    PoiCompleted {
        mission_id: u32,
        poi_id: u32,
        drone_id: u8,
    },
    /// Fault-Recovery/Robustness metric input: logged whenever a PoI stuck on a UAV
    /// that has become unavailable is released back into the pool for reassignment.
    PoiReleased {
        mission_id: u32,
        poi_id: u32,
        drone_id: u8,
    },
    /// Autonomous Relay & Role Management metric.
    RelayReallocation {
        drone_id: u8,
        from_role: String,
        to_role: String,
    },
    /// Autonomy metric: seconds between a fault being injected and the swarm visibly
    /// reconfiguring (reserve promoted).
    RecoveryTime {
        drone_id: u8,
        seconds: f64,
    },
    /// Safety metric input: logged whenever the 20 m minimum-separation threshold is
    /// breached.
    SeparationBreach {
        distance_m: f64,
    },
    /// Communication-resilience metric input: logged whenever the modelled link
    /// approaches the 100 m hard range.
    LinkWarning {
        distance_m: f64,
    },
    CollisionAvoidanceManeuver {
        drone_id: u8,
    },
    PhotoCaptured {
        frame_id: u32,
        bytes: usize,
        path: String,
    },
    /// Robustness/Fault-Recovery metric input: logged whenever a fault is deliberately
    /// injected (via the `fail battery`/`fail link` CLI commands or an automated harness).
    FaultInjected {
        kind: String,
        detail: String,
    },
    /// One origin packet successfully handed to the UDP socket.
    PacketSent {
        sequence: u64,
        sender_id: u8,
        target_id: u8,
        timestamp_ms: u64,
    },
    /// One packet received by the mesh. `latency_ms` is measured from the packet's
    /// origin timestamp to this process's receive timestamp.
    PacketReceived {
        sequence: u64,
        sender_id: u8,
        target_id: u8,
        latency_ms: u64,
    },
    /// Communication metric input: periodic mesh health snapshot. Counters are
    /// cumulative; the aggregator uses the final snapshot, not a sum of snapshots.
    MeshStatus {
        received: u64,
        forwarded: u64,
        dropped_outage: u64,
        link_down: bool,
        known_peers: usize,
    },
}

#[derive(Serialize)]
struct LogLine<'a> {
    ts_ms: u64,
    #[serde(flatten)]
    event: &'a MetricEvent,
}

/// Appends `event` to the metrics log as one JSON line, tagged with a wall-clock
/// timestamp. Silently does nothing if `init` was never called or failed to open a
/// file, so this is always safe to call.
pub async fn log_event(event: MetricEvent) {
    let Some(mutex) = LOG_FILE.get() else {
        return;
    };
    let line = LogLine {
        ts_ms: now_ms(),
        event: &event,
    };
    let Ok(json) = serde_json::to_string(&line) else {
        return;
    };
    let mut file = mutex.lock().await;
    let _ = writeln!(file, "{json}");
}
