#!/usr/bin/env python3
"""
Two Towers System View
======================

A read-only operator view of the whole system: every tower's state on one
page, served over HTTP so it can be opened from a laptop or a phone.

This exists because the interesting behaviour of a two-tower system is not
visible from either tower. Watching one turret tells you it is tracking; it
does not tell you the OTHER one deliberately stood down so this one could.
That decision only shows up when both towers' status is on one screen.

Architecture:
    /tower_a/status ─┐
                     ├─> this node (subscriber) ─> Flask ─> browser
    /tower_b/status ─┘

Deliberately a separate node, not part of a tracker:

  - a crash here cannot take a turret down with it
  - it is read-only: it subscribes and never publishes, so nothing it does
    can move a servo
  - it runs on whichever machine is convenient. ROS 2 has no master, so this
    is a gateway, not a coordinator, and the towers do not know it exists

ROS2 Interface:
    Subscribers:
        - /<tower_id>/status (TowerStatus) for each id in the tower_ids param

    Parameters:
        - tower_ids   (string[]): namespaces to watch, e.g. [tower_a, tower_b]
        - tower_labels(string[]): display names, e.g. [Orthanc, Barad-dur].
                                  Positional; falls back to the id.
        - stream_urls (string): comma-separated MJPEG URL per tower, positional
                               against tower_ids. An empty entry disables video
                               for that tower; "" disables all of them.

                               A comma-separated STRING rather than a string
                               array on purpose. Launch concatenates a list of
                               substitutions into one string, so a list-typed
                               parameter built from launch arguments arrives
                               as a single joined value and fails to coerce --
                               `Cannot convert value '' to a list of str`.
                               Taking the join as the interface removes the
                               ambiguity instead of fighting it.
        - port        (int): HTTP port for this page

Author: Tate Lloyd
License: MIT
"""

import threading
import time

import rclpy
from flask import Flask, Response, jsonify
from rclpy.node import Node

from two_towers.msg import TowerStatus

# A tower whose last status is older than this is shown as OFFLINE rather than
# frozen at its last reading. Status publishes at 5 Hz, so this is ~15 missed
# messages: long enough not to flicker on a congested hotspot, short enough
# that pulling a tower's power is visible on screen while you are still
# pointing at it.
STALE_AFTER_SEC = 3.0

app = Flask(__name__)

# Written by the ROS executor thread, read by Flask request threads. Python
# dict writes are atomic under the GIL and every value here is replaced
# wholesale rather than mutated, so a reader either sees the previous snapshot
# or the next one -- never a half-written one. A lock would also be correct;
# it is not needed for this access pattern.
_towers = {}
_config = {"order": [], "labels": {}, "streams": {}}


# =============================================================================
# ROS 2 NODE
# =============================================================================

class SystemViewNode(Node):
    """Subscribes to every tower's status and keeps the latest snapshot."""

    def __init__(self):
        super().__init__('system_view')

        self.declare_parameter('tower_ids', ['tower_a', 'tower_b'])
        self.declare_parameter('tower_labels', ['Orthanc', 'Barad-dur'])
        self.declare_parameter('stream_urls', '')
        self.declare_parameter('port', 8080)

        tower_ids = list(self.get_parameter('tower_ids').value)
        labels = list(self.get_parameter('tower_labels').value)
        streams_raw = self.get_parameter('stream_urls').value or ''
        streams = [u.strip() for u in streams_raw.split(',')]
        self.port = self.get_parameter('port').value

        _config['order'] = tower_ids
        for i, tid in enumerate(tower_ids):
            # Positional, with the id as the fallback. A short labels list is
            # a config typo, not a reason to refuse to start.
            _config['labels'][tid] = labels[i] if i < len(labels) else tid
            _config['streams'][tid] = streams[i] if i < len(streams) else ''

        self.subs = []
        for tid in tower_ids:
            topic = f'/{tid}/status'
            # Default-argument binding, not closure capture: a closure over the
            # loop variable would leave every callback writing to the last id.
            self.subs.append(
                self.create_subscription(
                    TowerStatus, topic,
                    lambda msg, tid=tid: self._on_status(tid, msg),
                    10,
                )
            )
            self.get_logger().info(f'Watching {topic} as "{_config["labels"][tid]}"')

        self._start_server()
        self.get_logger().info(f'System view on http://0.0.0.0:{self.port}')

    def _on_status(self, tower_id: str, msg: TowerStatus):
        """Flatten a TowerStatus into something JSON-serializable."""
        _towers[tower_id] = {
            'tower_id': msg.tower_id,
            'state': msg.state,
            'has_target': msg.has_target,
            'pan_angle': round(msg.pan_angle, 1),
            'tilt_angle': round(msg.tilt_angle, 1),
            'target_x': round(msg.target_position.x, 3),
            'target_y': round(msg.target_position.y, 3),
            'confidence': round(msg.target_confidence, 2),
            'time_to_edge': round(msg.time_to_edge, 1),
            'frames_since_detection': msg.frames_since_detection,
            # Wall clock, so staleness survives the publisher going away
            # entirely -- a message-count heuristic cannot detect silence.
            'received_at': time.time(),
        }

    def _start_server(self):
        self.flask_thread = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=self.port,
                                   threaded=True, debug=False, use_reloader=False),
            daemon=True,
        )
        self.flask_thread.start()


# =============================================================================
# HTTP
# =============================================================================

@app.route('/api/status')
def api_status():
    """Current snapshot of every configured tower, staleness resolved here."""
    now = time.time()
    out = []
    for tid in _config['order']:
        row = {
            'id': tid,
            'label': _config['labels'].get(tid, tid),
            'stream': _config['streams'].get(tid, ''),
            'online': False,
            'state': 'offline',
        }
        snap = _towers.get(tid)
        if snap is not None and (now - snap['received_at']) < STALE_AFTER_SEC:
            row.update(snap)
            row['online'] = True
            row['age'] = round(now - snap['received_at'], 1)
        out.append(row)
    return jsonify({'towers': out, 'stale_after': STALE_AFTER_SEC})


@app.route('/')
def index():
    return Response(PAGE, mimetype='text/html')


PAGE = """<!doctype html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Two Towers</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; padding:16px; background:#0d1117; color:#e6edf3;
         font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
  h1 { font-size:18px; font-weight:600; margin:0 0 4px; letter-spacing:.02em; }
  .sub { color:#7d8590; font-size:12px; margin-bottom:16px; }
  .grid { display:grid; gap:14px; grid-template-columns:repeat(auto-fit,minmax(300px,1fr)); }
  .card { background:#161b22; border:1px solid #30363d; border-radius:10px; padding:14px; }
  .top { display:flex; align-items:center; justify-content:space-between; margin-bottom:12px; }
  .name { font-size:16px; font-weight:600; }
  .badge { font-size:11px; font-weight:700; letter-spacing:.08em; padding:4px 9px;
           border-radius:20px; text-transform:uppercase; }
  .locked   { background:#1a7f37; color:#fff; }
  .tracking { background:#9e6a03; color:#fff; }
  .holding  { background:#1f6feb; color:#fff; }
  .scanning { background:#30363d; color:#8b949e; }
  .offline  { background:#6e2c2c; color:#ffdcdc; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  td { padding:3px 0; }
  td.k { color:#7d8590; width:52%; }
  td.v { text-align:right; font-variant-numeric:tabular-nums; }
  img.cam { width:100%; border-radius:6px; margin-top:12px; border:1px solid #30363d;
            display:block; background:#000; }
  .foot { color:#7d8590; font-size:11px; margin-top:14px; }
</style></head><body>
<h1>Two Towers</h1>
<div class="sub" id="sub">connecting...</div>
<div class="grid" id="grid"></div>
<div class="foot" id="foot"></div>
<script>
// Poll rather than stream. Status publishes at 5 Hz and this is a human-facing
// view, so 2 Hz is plenty; it also means a dropped request self-heals on the
// next tick instead of needing reconnect logic for a websocket.
function badge(t) {
  if (!t.online) return 'offline';
  if (t.state === 'locked') return 'locked';
  if (t.state === 'tracking') return 'tracking';
  return 'scanning';
}
function label(t, anyPeerTracking) {
  if (!t.online) return 'offline';
  if (t.state === 'locked') return 'locked';
  if (t.state === 'tracking') return 'tracking';
  // A scanning tower while another tower has the target is exactly the
  // coordination behaviour; call it out rather than making the viewer infer it.
  return anyPeerTracking ? 'standing by' : 'scanning';
}
async function tick() {
  let d;
  try { d = await (await fetch('/api/status')).json(); }
  catch (e) { document.getElementById('sub').textContent = 'lost connection to view'; return; }

  const anyTracking = d.towers.some(t => t.online && t.has_target);
  document.getElementById('grid').innerHTML = d.towers.map(t => {
    const cls = t.has_target ? badge(t) : (t.online && anyTracking ? 'holding' : badge(t));
    const rows = t.online ? `
      <tr><td class="k">pan / tilt</td><td class="v">${t.pan_angle}&deg; / ${t.tilt_angle}&deg;</td></tr>
      <tr><td class="k">target x , y</td><td class="v">${t.has_target ? t.target_x+' , '+t.target_y : '&mdash;'}</td></tr>
      <tr><td class="k">confidence</td><td class="v">${t.has_target ? t.confidence : '&mdash;'}</td></tr>
      <tr><td class="k">time to frame edge</td><td class="v">${t.time_to_edge >= 0 ? t.time_to_edge+' s' : '&mdash;'}</td></tr>
      <tr><td class="k">frames since detection</td><td class="v">${t.frames_since_detection}</td></tr>`
      : `<tr><td class="k">no status received</td><td class="v">&mdash;</td></tr>`;
    const cam = t.stream ? `<img class="cam" src="${t.stream}" alt="">` : '';
    return `<div class="card">
      <div class="top"><span class="name">${t.label}</span>
      <span class="badge ${cls}">${label(t, anyTracking && !t.has_target)}</span></div>
      <table>${rows}</table>${cam}</div>`;
  }).join('');

  const online = d.towers.filter(t => t.online).length;
  document.getElementById('sub').textContent =
    `${online} of ${d.towers.length} towers online`;
  document.getElementById('foot').textContent =
    `a tower silent for ${d.stale_after}s is shown offline`;
}
tick(); setInterval(tick, 500);
</script></body></html>
"""


# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = SystemViewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # The SIGINT handler may already have shut the context down; calling
        # shutdown() twice raises. Same guard as the detector.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
