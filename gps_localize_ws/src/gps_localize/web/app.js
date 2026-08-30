/* GPS_Localize web UI.
 *
 * Talks to web_server.py only:
 *   GET  /api/stream      Server-Sent Events, one "telemetry" frame per tick
 *   POST /api/heartbeat   proof that this browser is alive (the robot stops without it)
 *   POST /api/manual|nav|estop|config|waypoints
 *
 * No frameworks, no fonts, no CDN: the page works with the robot's Wi-Fi
 * hotspot and no internet at all.
 *
 * MEMORY: the trail array, the log list and the DOM nodes are all capped. The
 * page is meant to stay open for hours on a phone.
 */
(function () {
  'use strict';

  var $ = function (id) { return document.getElementById(id); };
  var TRAIL_MAX = 300;
  var LOG_MAX = 200;

  var state = {
    snap: null,
    datum: null,           // client side map origin {lat, lon}
    trail: [],
    waypoints: [],
    fitRequested: true,
    picked: null,          // point chosen on the map {x, y, lat, lon, distance, bearing, turn}
    poseTool: { armed: false, drag: null, applied: null },   // RViz-style start pose
    connected: false,
    lastFrame: 0,          // when the newest telemetry frame arrived
    undoAction: null,      // pending undo, offered in the banner
    wpDirty: false,        // waypoint edits not saved to the robot yet
    mode: 'robot',         // 'robot' = real stack, 'viewer' = desktop viewer
    modeApplied: null,     // what the DOM currently reflects
    simulation: false,     // viewer with the demo robot: buttons drive the simulation
    viewerAnnounced: false,
    jog: { forward: 0, turn: 0, timer: null, speedMps: null, turnDps: null },
    cfg: {
      group: 'nav',
      data: null,
      groups: ['nav', 'pid', 'motor', 'estimator', 'robot', 'safety', 'web']
    }
  };

  /* ------------------------------------------------------------------ api */
  function api(path, body) {
    var options = { method: body === undefined ? 'GET' : 'POST', headers: {} };
    if (body !== undefined) {
      options.headers['Content-Type'] = 'application/json';
      options.body = JSON.stringify(body);
    }
    return fetch(path, options).then(function (response) {
      return response.json().catch(function () { return { ok: false, error: 'bad response' }; });
    });
  }

  /* Banner is the single place an error is reported. Errors never auto-dismiss
   * and always offer a way out (Retry); notices are just informational. */
  function toast(message, isError, retry) {
    var box = $('banner');
    $('bannerText').textContent = message || '';
    $('bannerRetry').className = retry ? 'btn' : 'btn hidden';
    box.className = message ? ('banner' + (isError ? '' : ' warn')) : 'banner hidden';
  }

  /* ------------------------------------------------- connection state ---
   * Five states, so the page never lies about how fresh the numbers are:
   *   connecting -> live -> stale (data shown but old) -> offline (nothing arriving)
   * The CSS dims .live-data and marks the tiles as soon as we leave "live". */
  function setConnection(state, detail) {
    if (state !== state_conn) {
      state_conn = state;
      document.body.dataset.conn = state;
    }
    var text = {
      connecting: 'กำลังเชื่อมต่อ… / connecting…',
      live: 'ข้อมูลสด / live',
      stale: 'ข้อมูลค้าง / stale',
      offline: 'ไม่ได้เชื่อมต่อ / offline'
    }[state] || state;
    $('connLabel').textContent = text + (detail ? ' · ' + detail : '');
  }
  var state_conn = null;

  /* --------------------------------------------------------- viewer mode */
  /* The desktop viewer (webapp/viewer_server.py) serves the very same page but
   * has no ROS and no robot. It reports server.mode = 'viewer'; everything that
   * would move the robot is disabled here, and the page says so loudly. */
  function applyServerMode(server) {
    if (!server) return;
    var mode = server.mode || 'robot';
    var simulation = !!server.simulation;
    // this runs on every telemetry frame, so do the DOM work only on a change
    if (mode === state.modeApplied && simulation === state.simulation) return;
    state.modeApplied = mode;
    state.mode = mode;
    state.simulation = simulation;
    if (mode !== 'viewer') { $('viewerBar').className = 'viewerbar hidden'; return; }

    $('viewerBar').className = 'viewerbar';
    $('viewerNote').textContent = (state.simulation
      ? 'ปุ่มทุกปุ่มใช้ได้ แต่สั่งได้เฉพาะ "หุ่นจำลอง" ในหน้าจอนี้ / buttons drive a SIMULATED robot · '
      : 'ควบคุมไม่ได้ / control disabled · ') +
      'แก้ไฟล์ตั้งค่าได้ที่ ' + (server.config_dir || '');

    // With the simulator running the buttons stay usable - that is the point of
    // the demo. Without it (--no-demo) nothing can respond, so they are locked.
    var controls = [$('btnEstop'), $('btnReset'), $('wpAddHere'),
                    $('btnMagStart'), $('btnMagFinish')];
    Array.prototype.push.apply(controls, document.querySelectorAll('[data-nav]'));
    Array.prototype.push.apply(controls, document.querySelectorAll('.jog'));
    controls.forEach(function (element) {
      if (!element) return;
      element.disabled = !state.simulation;
      element.title = state.simulation
        ? 'สั่งหุ่นจำลองเท่านั้น / drives the simulated robot only'
        : 'ปิดใช้งานในโหมดดูอย่างเดียว / disabled in viewer mode';
    });

    if (server.robot_url) $('robotIp').value = server.robot_url;
    if (!state.viewerAnnounced) {
      state.viewerAnnounced = true;
      window.setTimeout(function () {
        window.alert('โหมดสาธิต / DEMO — VIEW ONLY\n\n' +
          'หน้าต่างนี้ไม่ได้ต่อกับหุ่นยนต์จริง\n' +
          'This window is not connected to a real robot.\n\n' +
          (state.simulation
            ? 'ปุ่มทุกปุ่มกดได้ แต่สั่งได้แค่หุ่นจำลองบนแผนที่ ใช้ฝึกใช้งานและสอนงานได้เต็มที่\n' +
              'All buttons work, but they only move the simulated robot on the map.\n\n'
            : '') +
          'ควบคุมของจริง: เปิด http://<ไอพีหุ่นยนต์>:8080 จากมือถือหรือคอมที่อยู่ใน Wi-Fi เดียวกัน\n' +
          'To drive the real robot: open http://<robot-ip>:8080 on the robot Wi-Fi.');
      }, 400);
    }
  }

  function blockedInViewer() {
    if (state.mode !== 'viewer' || state.simulation) return false;
    toast('โหมดดูอย่างเดียว ควบคุมหุ่นยนต์ไม่ได้ / viewer mode: robot control is disabled', false);
    return true;
  }

  $('btnOpenRobot').addEventListener('click', function () {
    var value = ($('robotIp').value || '').trim();
    if (!value) { window.alert('ใส่ไอพีของหุ่นยนต์ก่อน เช่น 192.168.1.50'); return; }
    if (!/^https?:\/\//.test(value)) value = 'http://' + value;
    if (!/:\d+$/.test(value)) value += ':8080';
    window.open(value, '_blank');
  });

  /* --------------------------------------------------------------- geodesy */
  var R = 6378137.0;
  function enu(lat, lon, datum) {
    var east = (lon - datum.lon) * Math.PI / 180 * R * Math.cos(datum.lat * Math.PI / 180);
    var north = (lat - datum.lat) * Math.PI / 180 * R;
    return { x: east, y: north };
  }
  /** inverse of enu(): local metres back to WGS-84 degrees */
  function latLon(x, y, datum) {
    return {
      lat: datum.lat + (y / R) * 180 / Math.PI,
      lon: datum.lon + (x / (R * Math.cos(datum.lat * Math.PI / 180))) * 180 / Math.PI
    };
  }
  function haversine(lat1, lon1, lat2, lon2) {
    var p1 = lat1 * Math.PI / 180, p2 = lat2 * Math.PI / 180;
    var dp = p2 - p1, dl = (lon2 - lon1) * Math.PI / 180;
    var a = Math.sin(dp / 2) * Math.sin(dp / 2) +
            Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) * Math.sin(dl / 2);
    return 2 * R * Math.asin(Math.min(1, Math.sqrt(a)));
  }

  /** Compass bearing from point 1 to point 2: 0 = north, clockwise. */
  function bearing(lat1, lon1, lat2, lon2) {
    var p1 = lat1 * Math.PI / 180, p2 = lat2 * Math.PI / 180;
    var dl = (lon2 - lon1) * Math.PI / 180;
    var y = Math.sin(dl) * Math.cos(p2);
    var x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
  }

  /** Shortest signed turn from the robot's heading to a bearing, -180..180. */
  function turnTo(bearingDeg) {
    var heading = (state.snap && state.snap.telemetry && state.snap.telemetry.heading_deg);
    if (heading === undefined || heading === null) return null;
    return (bearingDeg - heading + 540) % 360 - 180;
  }

  /* ------------------------------------------------------------------ tabs */
  Array.prototype.forEach.call(document.querySelectorAll('.tab'), function (tab) {
    tab.addEventListener('click', function () {
      Array.prototype.forEach.call(document.querySelectorAll('.tab'), function (t) {
        t.classList.remove('active');
      });
      Array.prototype.forEach.call(document.querySelectorAll('.tabpage'), function (p) {
        p.classList.remove('active');
      });
      tab.classList.add('active');
      $('tab-' + tab.dataset.tab).classList.add('active');
      if (tab.dataset.tab === 'map') { state.fitRequested = true; drawMap(); }
      if (tab.dataset.tab === 'cfg' && !state.cfg.data) loadConfig(state.cfg.group);
      if (tab.dataset.tab === 'wp') loadWaypoints();
    });
  });

  /* ------------------------------------------------------------------- SSE */
  var source = null;

  function connect() {
    if (source) { try { source.close(); } catch (err) { /* already gone */ } }
    setConnection('connecting');
    source = new EventSource('/api/stream');

    source.addEventListener('telemetry', function (event) {
      try { onSnapshot(JSON.parse(event.data)); } catch (err) { /* ignore one bad frame */ }
    });

    source.addEventListener('log', function (event) {
      try { addLog(JSON.parse(event.data)); } catch (err) { /* ignore */ }
    });

    source.onopen = function () {
      state.connected = true;
      state.lastFrame = Date.now();
      setConnection('live');
      toast('');
    };

    source.onerror = function () {
      state.connected = false;
      // EventSource retries on its own (retry: 2000 from the server); the
      // banner stays until data actually flows again, with a manual way out.
      toast('การเชื่อมต่อหลุด — หุ่นยนต์จะหยุดเองภายใน 1 วินาที / connection lost, the robot stops by itself',
            true, true);
    };
  }

  $('bannerRetry').addEventListener('click', function () {
    if (this.dataset.undo === '1' && state.undoAction) {
      var restore = state.undoAction;
      clearUndo();
      toast('');
      restore();
      return;
    }
    toast('');
    connect();
    api('/api/state').then(function (response) {
      if (response && response.ok && response.state) onSnapshot(response.state);
    });
  });

  /* Watches how old the newest frame is and degrades the whole page honestly. */
  function watchFreshness() {
    if (!state.lastFrame) { setConnection('connecting'); return; }
    var age = (Date.now() - state.lastFrame) / 1000;
    if (age < 2) setConnection('live');
    else if (age < 10) setConnection('stale', 'ล่าสุด ' + age.toFixed(0) + ' วินาทีที่แล้ว');
    else setConnection('offline', 'ล่าสุด ' + Math.round(age) + ' วินาทีที่แล้ว');
  }

  /* ------------------------------------------------------------- heartbeat */
  var beatFailures = 0;
  function heartbeat() {
    api('/api/heartbeat', {}).then(function (response) {
      if (response && response.ok) {
        beatFailures = 0;
        if (state.connected) toast('');
      }
    }).catch(function () {
      beatFailures += 1;
      if (beatFailures > 2) {
        toast('ส่งสัญญาณหัวใจไม่ได้ — ตรวจสอบเครือข่าย / heartbeat failed, check the network', true);
      }
    });
  }


  /* Undo instead of a confirm dialog: losing a waypoint by mistake is easy,
   * and a confirm on every delete just trains people to click through. */
  var undoTimer = null;
  function offerUndo(message, restore) {
    toast(message + ' — กด “เลิกทำ” ถ้าไม่ได้ตั้งใจ', false);
    var button = $('bannerRetry');
    button.className = 'btn';
    button.textContent = 'เลิกทำ / Undo';
    button.dataset.undo = '1';
    if (undoTimer) window.clearTimeout(undoTimer);
    state.undoAction = restore;
    undoTimer = window.setTimeout(function () {
      if (button.dataset.undo === '1') { clearUndo(); toast(''); }
    }, 8000);
  }

  function clearUndo() {
    var button = $('bannerRetry');
    button.dataset.undo = '';
    button.textContent = 'ลองใหม่ / Retry';
    state.undoAction = null;
  }

  /* ------------------------------------------------ simple / advanced view */
  function setView(mode) {
    document.body.dataset.view = mode;
    $('btnSimple').className = 'chipbtn' + (mode === 'simple' ? ' active' : '');
    $('btnAdvanced').className = 'chipbtn' + (mode === 'advanced' ? ' active' : '');
    try { window.localStorage.setItem('gpsLocalizeView', mode); } catch (err) { /* private mode */ }
  }
  $('btnSimple').addEventListener('click', function () { setView('simple'); });
  $('btnAdvanced').addEventListener('click', function () { setView('advanced'); });

  $('btnHelp').addEventListener('click', function () {
    var dialog = $('helpDialog');
    if (dialog.showModal) dialog.showModal(); else dialog.setAttribute('open', 'open');
  });
  $('helpClose').addEventListener('click', function () {
    var dialog = $('helpDialog');
    if (dialog.close) dialog.close(); else dialog.removeAttribute('open');
  });

  /* --------------------------------------------- the one-sentence headline */
  /* Says what the robot is doing in plain words and offers the single action
   * that makes sense next, so nobody has to work out whether they want START,
   * RESUME or NEXT. */

  /* ------------------------------------------------- motion guard ---------
   * Why a command did nothing, in words, before it is sent.
   *
   * Without this a press of START while the mushroom is down is simply
   * ignored: the firmware refuses it, correctly, and the person is left
   * pressing a button that does nothing with no idea why. Worse, the two
   * blockers need OPPOSITE actions - a hardware emergency has to be released
   * by hand at the robot and no amount of clicking will clear it, while a
   * software e-stop is cleared from this page. Saying "stopped" for both would
   * send someone to the wrong place.
   *
   * Returns null when the robot may move, otherwise {title, detail, clearable}.
   */
  function motionBlocker(snap) {
    snap = snap || state.snap || {};
    var t = snap.telemetry || {};
    var safety = snap.safety || {};
    var reasons = t.stop_reasons || safety.firmware_stop_reasons || [];
    var has = function (name) { return reasons.indexOf(name) >= 0; };

    if (has('hardware_emergency') || has('esp_button')) {
      return {
        clearable: false,
        title: 'ปุ่มฉุกเฉินถูกกด / the emergency button is pressed',
        detail: 'ต้องไปบิดปุ่มฉุกเฉินที่ตัวหุ่นยนต์ให้เด้งขึ้นก่อน — กดปุ่มในหน้านี้ไม่ช่วย\n\n'
              + 'Twist the emergency button on the robot until it pops back up. '
              + 'This page cannot clear a hardware emergency, by design - the '
              + 'motor rails are physically open while it is down.'
      };
    }
    if (has('software_estop') || safety.estop) {
      return {
        clearable: true,
        title: 'หยุดฉุกเฉินจากหน้าเว็บ / software e-stop is latched',
        detail: 'กดปุ่ม "ปลดหยุดฉุกเฉิน / Clear e-stop" ในหน้านี้ก่อน แล้วจึงสั่งงานได้\n\n'
              + 'Clear it from this page, then start again.'
      };
    }
    if (has('battery_low')) {
      return {
        clearable: false,
        title: 'แบตเตอรี่ต่ำ / the battery cutoff is latched',
        detail: 'ต้องปิดหุ่นยนต์แล้วเปิดใหม่เท่านั้น หลังจากชาร์จหรือเปลี่ยนแบตแล้ว\n\n'
              + 'Charge or swap the pack, then switch the robot off and on again. '
              + 'Nothing else clears this - a LiPo at the cutoff recovers on its '
              + 'own when the load comes off, so an automatic release would let '
              + 'the robot drive it flat.'
      };
    }
    if (has('agent_lost')) {
      return {
        clearable: false,
        title: 'ไม่ได้เชื่อมต่อหุ่นยนต์ / the robot is not connected',
        detail: 'The micro-ROS agent cannot reach the board. Check that it is '
              + 'powered and on the same network.'
      };
    }
    if (has('startup_lockout')) {
      return {
        clearable: false,
        title: 'กำลังเริ่มระบบ / still starting up',
        detail: 'The boot lockout has not expired yet. Try again in a moment.'
      };
    }
    return null;
  }

  /* Show the reason and refuse to send. Called by every control path, so a
   * blocked robot explains itself once rather than swallowing the command. */
  function blockedByEmergency() {
    var blocker = motionBlocker();
    if (!blocker) return false;
    window.alert(blocker.title + '\n\n' + blocker.detail);
    toast(blocker.title, true);
    return true;
  }

  function renderHero(snap) {
    var t = snap.telemetry || {};
    var nav = snap.nav || {};
    var safety = snap.safety || {};
    var estop = !!safety.estop || t.state_name === 'ESTOP';
    var reasons = t.stop_reasons || [];
    var navState = nav.state || 'IDLE';
    var primary = { label: 'เริ่มภารกิจ / START', action: 'start', enabled: true };
    var secondary = null;
    var icon = '🤖', title = '', sub = '', tone = 'ok';

    if (state.mode === 'viewer' && !state.simulation) {
      icon = '👁'; tone = 'warn';
      title = 'โหมดดูอย่างเดียว';
      sub = 'หน้าต่างนี้ควบคุมหุ่นยนต์ไม่ได้ / view only';
      primary.enabled = false;
    } else if (state_conn === 'offline' || state_conn === 'connecting') {
      icon = '⏳'; tone = 'warn';
      title = state_conn === 'offline' ? 'ไม่ได้เชื่อมต่อกับหุ่นยนต์' : 'กำลังเชื่อมต่อ…';
      sub = 'ตัวเลขที่เห็นอาจไม่ใช่ค่าปัจจุบัน / data may be old';
      primary.enabled = false;
    } else if (estop) {
      icon = '🛑'; tone = 'bad';
      title = 'หยุดฉุกเฉินอยู่';
      sub = reasons.length ? ('สาเหตุ: ' + reasons.join(', ')) : 'กด “ปลดล็อก” มุมขวาบนเมื่อปลอดภัยแล้ว';
      primary = { label: 'ปลดล็อก / RESET', action: 'reset', enabled: true };
    } else if (navState === 'RUNNING' || navState === 'DISCOVER') {
      icon = '🚗'; tone = 'busy';
      var target = nav.target || {};
      title = navState === 'DISCOVER'
        ? 'กำลังหาทิศทาง กรุณารอสักครู่'
        : 'กำลังไปจุดที่ ' + ((nav.index || 0) + 1) + '/' + (nav.total || 1) +
          (target.name ? ' · ' + target.name : '');
      sub = 'เหลือระยะ ' + fmt(nav.distance_m, 1) + ' m' +
            (nav.eta_s ? ' · ประมาณ ' + fmt(nav.eta_s, 0) + ' วินาที' : '');
      primary = { label: 'หยุดหุ่น / STOP', action: 'stop', enabled: true };
      secondary = { label: 'พักไว้ก่อน / PAUSE', action: 'pause' };
    } else if (navState === 'PAUSED') {
      icon = '⏸'; tone = 'warn';
      title = 'พักภารกิจไว้';
      sub = 'กด “ไปต่อ” เพื่อวิ่งต่อจากจุดที่ค้างไว้';
      primary = { label: 'ไปต่อ / RESUME', action: 'resume', enabled: true };
      secondary = { label: 'จบภารกิจ / STOP', action: 'stop' };
    } else if (navState === 'BLOCKED') {
      icon = '⚠'; tone = 'warn';
      title = 'วิ่งต่อไม่ได้ตอนนี้';
      sub = nav.block_reason || 'สัญญาณ GPS ไม่ดีพอ';
      primary = { label: 'จบภารกิจ / STOP', action: 'stop', enabled: true };
    } else if (navState === 'FINISHED') {
      icon = '🏁'; tone = 'ok';
      title = 'ถึงจุดหมายครบแล้ว';
      sub = 'กดเริ่มใหม่เพื่อวิ่งอีกรอบ';
      primary = { label: 'เริ่มใหม่ / START', action: 'start', enabled: true };
    } else {
      icon = '🤖'; tone = 'ok';
      title = 'จอดอยู่ พร้อมรับคำสั่ง';
      var count = (nav.total !== undefined ? nav.total : state.waypoints.length) || 0;
      sub = count
        ? ('มีจุดหมาย ' + count + ' จุด · กดเริ่มเพื่อให้วิ่งเอง หรือขับเองด้านล่าง')
        : 'ยังไม่มีจุดหมาย — ไปหน้า “แผนที่” คลิกจุดที่ต้องการ แล้วเลือก “เพิ่มเข้ารายการ”';
      primary.enabled = count > 0;
    }

    $('heroCard').className = 'hero card ' + tone;
    $('heroIcon').textContent = icon;
    text('heroTitle', title);
    text('heroSub', sub);

    var button = $('heroPrimary');
    button.textContent = primary.label;
    button.disabled = !primary.enabled;
    button.dataset.action = primary.action;
    button.title = primary.enabled ? '' :
      (state.waypoints.length ? '' : 'เพิ่มจุดหมายก่อนจึงจะเริ่มได้ / add a waypoint first');

    var second = $('heroSecondary');
    if (secondary) {
      second.className = 'btn';
      second.textContent = secondary.label;
      second.dataset.action = secondary.action;
    } else {
      second.className = 'btn hidden';
    }
  }

  function heroAction(action) {
    if (blockedInViewer()) return;
    if (action === 'reset') { api('/api/estop', { stop: false }); return; }
    api('/api/nav', { action: action });
  }
  $('heroPrimary').addEventListener('click', function () { heroAction(this.dataset.action); });
  $('heroSecondary').addEventListener('click', function () { heroAction(this.dataset.action); });

  /* speed presets */
  Array.prototype.forEach.call(document.querySelectorAll('[data-preset]'), function (button) {
    button.addEventListener('click', function () {
      changeLimits({ speed_mps: Number(button.dataset.preset) });
    });
  });

  /* -------------------------------------------------------------- snapshot */
  function onSnapshot(snap) {
    state.snap = snap;
    state.lastFrame = Date.now();
    watchFreshness();
    applyServerMode(snap.server);
    var t = snap.telemetry || {};
    var fix = snap.fix || {};
    var nav = snap.nav || {};
    var safety = snap.safety || {};

    // ---- header ----
    var stateName = t.state_name || 'UNKNOWN';
    $('stateLabel').textContent = stateName + (t.stop_reasons && t.stop_reasons.length
      ? ' · ' + t.stop_reasons.join(', ') : '');

    setPill('pillLink', safety.link_ok === false ? 'bad' : (state.connected ? 'ok' : 'warn'),
      safety.link_ok === false ? 'ขาด' : 'ปกติ');
    setPill('pillEsp', snap.telemetry_age_s !== null && snap.telemetry_age_s < 1.5 ? 'ok' : 'bad',
      snap.telemetry_age_s === null ? 'ไม่พบ' : snap.telemetry_age_s.toFixed(1) + ' s');
    var sats = Math.round(t.gps_satellites || 0);
    setPill('pillGps', t.gps_fix_quality > 0 ? (sats >= 6 ? 'ok' : 'warn') : 'bad',
      sats + ' ดาว · HDOP ' + fmt(t.gps_hdop, 1));
    var volts = t.battery_v || 0;
    setPill('pillBatt', volts > 10.8 ? 'ok' : (volts > 9.9 ? 'warn' : 'bad'), fmt(volts, 2) + ' V');

    // ---- dashboard tiles ----
    text('tSpeed', fmt(t.speed_mps, 2));
    text('tHeading', fmt(t.heading_deg, 0));
    text('tTargetHeading', fmt(t.target_heading_deg, 0));
    text('tHeadingErr', fmt(t.heading_error_deg, 1));
    text('tLat', fix.lat !== undefined ? fix.lat.toFixed(7) : '—');
    text('tLon', fix.lon !== undefined ? fix.lon.toFixed(7) : '—');
    text('tSats', sats);
    text('tHdop', fmt(t.gps_hdop, 1));
    text('tDist', nav.distance_m !== undefined ? fmt(nav.distance_m, 1) : '—');

    text('tRpmL', fmt(t.wheel_rpm_left, 1));
    text('tRpmR', fmt(t.wheel_rpm_right, 1));
    text('tMpsL', fmt(t.wheel_mps_left, 2));
    text('tMpsR', fmt(t.wheel_mps_right, 2));
    text('tPwmL', fmt(t.pwm_left, 0));
    text('tPwmR', fmt(t.pwm_right, 0));
    text('tYawRate', fmt(t.yaw_rate_dps, 1));
    text('tOdomX', fmt(t.odom_x, 2));
    text('tOdomY', fmt(t.odom_y, 2));

    renderSensors(t);
    renderHero(snap);

    text('tBattV', fmt(t.battery_v, 2));
    text('tBattA', fmt(t.battery_a, 2));
    text('tMaV', fmt(t.motor_a_v, 2));
    text('tMaA', fmt(t.motor_a_a, 2));
    text('tMbV', fmt(t.motor_b_v, 2));
    text('tMbA', fmt(t.motor_b_a, 2));
    text('tFanV', fmt(t.fan_v, 2));
    text('tFanA', fmt(t.fan_a, 2));
    text('tHeap', fmt(t.free_heap_kb, 0));
    text('tLoop', fmt(t.loop_us, 0));

    // ---- mission ----
    var badge = $('navState');
    badge.textContent = nav.state || 'IDLE';
    badge.className = 'badge ' + (nav.state === 'RUNNING' ? 'run'
      : (nav.state === 'BLOCKED' || nav.state === 'PAUSED' ? 'hold' : ''));
    var target = nav.target;
    $('navTarget').textContent = target
      ? ('จุดที่ ' + ((nav.index || 0) + 1) + '/' + nav.total + ' · ' + target.name +
         ' (' + target.lat.toFixed(6) + ', ' + target.lon.toFixed(6) + ')')
      : 'ยังไม่ได้เลือกจุดหมาย / no waypoint selected';
    text('navDist', nav.distance_m !== undefined ? fmt(nav.distance_m, 1) : '—');
    text('navBearing', nav.bearing_deg !== undefined ? fmt(nav.bearing_deg, 0) : '—');
    text('navEta', nav.eta_s ? fmt(nav.eta_s, 0) : '—');
    $('navBlock').textContent = nav.block_reason || '';
    var progress = nav.total ? Math.min(100, ((nav.index || 0) / nav.total) * 100) : 0;
    $('navProgress').style.width = progress.toFixed(0) + '%';

    // ---- e-stop button state ----
    var estop = !!safety.estop || stateName === 'ESTOP';
    $('btnReset').disabled = !estop;

    // ---- map data ----
    if (fix.lat !== undefined && fix.lon !== undefined && (fix.lat !== 0 || fix.lon !== 0)) {
      if (!state.datum) state.datum = { lat: fix.lat, lon: fix.lon };
      var p = enu(fix.lat, fix.lon, state.datum);
      var lastPoint = state.trail[state.trail.length - 1];
      if (!lastPoint || Math.abs(lastPoint.x - p.x) > 0.15 || Math.abs(lastPoint.y - p.y) > 0.15) {
        state.trail.push(p);
        if (state.trail.length > TRAIL_MAX) state.trail.shift();
      }
    }
    // Never let a telemetry frame overwrite edits in progress: the list arrives
    // 5 times a second, and the operator may be halfway through typing a
    // coordinate or reordering the route.
    if (nav.waypoints && nav.waypoints.length && !state.wpDirty
        && !$('tab-wp').classList.contains('active')) {
      state.waypoints = nav.waypoints;
    }
    if ($('tab-map').classList.contains('active')) drawMap();
    if ($('tab-wp').classList.contains('active')) updateWaypointDistances();

    if (snap.server && state.jog.speedMps === null) {
      state.jog.speedMps = snap.server.manual_speed_mps;
      state.jog.turnDps = snap.server.manual_turn_dps;
      text('manualSpeed', Number(snap.server.manual_speed_mps || 0).toFixed(2));
      text('manualTurn', Number(snap.server.manual_turn_dps || 0).toFixed(0));
      renderSpeed();
    }

    // the address other devices should open - shown so it can be read aloud
    var lan = (snap.server && snap.server.lan_url) || '';
    var lanEl = $('lanUrl');
    if (lan) {
      lanEl.textContent = '📱 ' + lan.replace('http://', '');
      lanEl.href = lan;
      lanEl.title = 'เปิดที่อยู่นี้จากมือถือใน Wi-Fi เดียวกัน / open this on a phone';
      lanEl.className = 'lanurl';
    } else {
      lanEl.className = 'lanurl hidden';
    }

    $('footInfo').textContent =
      'ผู้ใช้งานที่เชื่อมต่อ ' + ((snap.server && snap.server.clients) || 0) +
      ' · ทำงานมาแล้ว ' + Math.floor(((snap.server && snap.server.uptime_s) || 0) / 60) + ' นาที' +
      ' · ESP RAM ว่าง ' + fmt(t.free_heap_kb, 0) + ' kB';
  }

  /* ------------------------------------------------- sensor health chips */
  /* One chip per sensor. The robot keeps driving when one of them dies - the
   * chips show which source the heading and the speed fell back to. */
  function renderSensors(t) {
    var missing = t.missing_sensors || [];
    var has = function (name) { return missing.indexOf(name) < 0; };
    var chips = [
      ['IMU', has('imu_missing')],
      ['GPS', has('gps_missing')],
      ['เข็มทิศ Compass', t.mag_status === 1, t.mag_status === 2],
      ['IMU mag', has('imu_magnetometer_missing')]
    ];
    var box = $('sensorChips');
    box.textContent = '';
    chips.forEach(function (chip) {
      var span = document.createElement('span');
      span.className = chip[2] ? 'warn' : (chip[1] ? 'ok' : 'bad');
      span.textContent = chip[0] + (chip[2] ? ' ⚠ รบกวน' : (chip[1] ? ' ✓' : ' ✕'));
      box.appendChild(span);
    });
    if (missing.indexOf('heading_relative_only') >= 0) {
      var warnChip = document.createElement('span');
      warnChip.className = 'warn';
      warnChip.textContent = 'ทิศยังไม่อ้างอิงจริง / heading relative only';
      box.appendChild(warnChip);
    }

    // motor faults seen through the INA226 (stall = current but no back-EMF)
    (t.motor_fault_names || []).forEach(function (fault) {
      var chip = document.createElement('span');
      chip.className = 'bad';
      chip.textContent = {
        left_stalled: 'ล้อซ้ายติดขัด / left stalled',
        right_stalled: 'ล้อขวาติดขัด / right stalled',
        left_open_circuit: 'มอเตอร์ซ้ายไม่กินไฟ / left open',
        right_open_circuit: 'มอเตอร์ขวาไม่กินไฟ / right open'
      }[fault] || fault;
      box.appendChild(chip);
    });

    text('tHeadingRef', t.heading_ref_name || '—');
    text('tMagHeading', fmt(t.mag_heading_deg, 0));
    text('tMagStatus', t.mag_status_name || '—');
    text('tSpeedSource', t.speed_source_name || '—');
    text('tMotorRpmL', fmt(t.motor_rpm_left, 0));
    text('tMotorRpmR', fmt(t.motor_rpm_right, 0));

    var calibrating = t.mag_calib_state === 1;
    $('btnMagStart').disabled = calibrating || state.mode === 'viewer';
    $('btnMagFinish').disabled = !calibrating || state.mode === 'viewer';
    if (calibrating) {
      $('magHint').textContent = 'กำลังเก็บค่า… หมุนหุ่นช้า ๆ ให้ครบ 1 รอบ แล้วกดบันทึกผล';
    } else if (t.mag_calib_state === 2) {
      $('magHint').textContent = 'บันทึกลงบอร์ดแล้ว ใช้ได้เลยแม้ปิดเครื่อง';
    } else if (t.mag_calib_state === 3) {
      $('magHint').textContent = 'หมุนไม่ครบรอบ ลองใหม่อีกครั้ง';
    }
  }

  $('btnMagStart').addEventListener('click', function () {
    if (blockedInViewer()) return;
    api('/api/mag_calib', { start: true });
    window.alert('เริ่มปรับเข็มทิศ / compass calibration started\n\n' +
      'หมุนหุ่นยนต์ช้า ๆ ให้ครบอย่างน้อย 1 รอบ (ใช้ปุ่มเลี้ยวได้)\n' +
      'Turn the robot slowly through at least one full circle, then press Finish.');
  });

  $('btnMagFinish').addEventListener('click', function () {
    if (blockedInViewer()) return;
    api('/api/mag_calib', { start: false });
  });

  function setPill(id, cls, value) {
    var pill = $(id);
    pill.className = 'pill ' + cls;
    pill.lastElementChild.textContent = value;
  }
  function text(id, value) { var el = $(id); if (el) el.textContent = value; }
  function fmt(value, digits) {
    if (value === undefined || value === null || isNaN(value)) return '—';
    return Number(value).toFixed(digits);
  }

  /* -------------------------------------------------------------- e-stop */
  $('btnEstop').addEventListener('click', function () {
    api('/api/estop', { stop: true });
    toast('สั่งหยุดฉุกเฉินแล้ว — กด “ปลดล็อก” เมื่อปลอดภัย / emergency stop latched', true);
  });
  $('btnReset').addEventListener('click', function () {
    api('/api/estop', { stop: false }).then(function () { toast(''); });
  });

  /* ----------------------------------------------------------- navigation */
  Array.prototype.forEach.call(document.querySelectorAll('[data-nav]'), function (button) {
    button.addEventListener('click', function () {
      if (blockedByEmergency()) return;
      api('/api/nav', { action: button.dataset.nav });
    });
  });

  /* ------------------------------------------------------------ manual jog */
  function sendJog() {
    // full scale: the speed the operator set is the speed the robot drives.
    // There used to be a second "power" multiplier here, which meant the
    // 0.18 m/s preset actually produced 0.11 m/s.
    if (blockedByEmergency()) return;
    api('/api/manual', { forward: state.jog.forward, turn: state.jog.turn });
  }

  function startJog(forward, turn) {
    if (blockedInViewer()) return;
    state.jog.forward = forward;
    state.jog.turn = turn;
    if (state.jog.timer) clearInterval(state.jog.timer);
    sendJog();
    if (forward !== 0 || turn !== 0) {
      // the firmware stops after 500 ms without a command, so keep it fed
      state.jog.timer = setInterval(sendJog, 150);
    }
  }

  function stopJog() {
    if (state.jog.timer) { clearInterval(state.jog.timer); state.jog.timer = null; }
    state.jog.forward = 0;
    state.jog.turn = 0;
    api('/api/manual', { action: 'stop' });
  }

  Array.prototype.forEach.call(document.querySelectorAll('.jog'), function (button) {
    var forward = Number(button.dataset.fwd);
    var turn = Number(button.dataset.turn);
    button.addEventListener('pointerdown', function (event) {
      event.preventDefault();
      button.classList.add('held');
      startJog(forward, turn);
    });
    ['pointerup', 'pointerleave', 'pointercancel'].forEach(function (name) {
      button.addEventListener(name, function () {
        button.classList.remove('held');
        stopJog();
      });
    });
  });

  /* ------------------------------------------- manual speed steppers */
  function renderSpeed() {
    var speed = state.jog.speedMps;
    if (typeof speed !== 'number') return;
    text('speedNow', speed.toFixed(2));
    Array.prototype.forEach.call(document.querySelectorAll('[data-preset]'), function (button) {
      var match = Math.abs(Number(button.dataset.preset) - speed) < 0.005;
      button.className = 'btn' + (match ? ' on' : '');
    });
  }

  function applyLimits(response) {
    if (!response || !response.ok) return;
    if (typeof response.speed_mps === 'number') {
      state.jog.speedMps = response.speed_mps;
      text('manualSpeed', response.speed_mps.toFixed(2));
    }
    if (typeof response.turn_dps === 'number') {
      state.jog.turnDps = response.turn_dps;
      text('manualTurn', response.turn_dps.toFixed(0));
    }
    if (response.saved) $('speedMsg').textContent = 'บันทึกเป็นค่าเริ่มต้นแล้ว / saved';
    renderSpeed();
  }

  function changeLimits(payload) {
    if (blockedInViewer()) return;
    api('/api/manual_limits', payload).then(function (response) {
      if (response && response.ok) {
        applyLimits(response);
        if (state.jog.timer) sendJog();      // take effect while a button is held
      } else if (response) {
        $('speedMsg').textContent = response.error || '';
      }
    });
  }

  Array.prototype.forEach.call(document.querySelectorAll('[data-speed]'), function (button) {
    button.addEventListener('click', function () {
      changeLimits({ speed_delta: Number(button.dataset.speed) });
    });
  });
  Array.prototype.forEach.call(document.querySelectorAll('[data-turnrate]'), function (button) {
    button.addEventListener('click', function () {
      changeLimits({ turn_delta: Number(button.dataset.turnrate) });
    });
  });
  $('btnSaveSpeeds').addEventListener('click', function () {
    changeLimits({ save: true });
  });


  /* --------------------------------------------------- turn commands */
  function sendTurn(payload) {
    if (blockedInViewer()) return;
    api('/api/turn', payload).then(function (response) {
      if (!response.ok) toast('หมุนไม่ได้: ' + response.error, true);
    });
  }

  Array.prototype.forEach.call(document.querySelectorAll('[data-turn-deg]'), function (button) {
    button.addEventListener('click', function () {
      sendTurn({ deg: Number(button.dataset.turnDeg) });
    });
  });
  Array.prototype.forEach.call(document.querySelectorAll('[data-compass]'), function (button) {
    button.addEventListener('click', function () {
      sendTurn({ compass: button.dataset.compass });
    });
  });
  $('btnTurnCustom').addEventListener('click', function () {
    sendTurn({ deg: Number($('turnDeg').value || 0) });
  });

  var KEYS = { w: [1, 0], s: [-1, 0], a: [0, -1], d: [0, 1],
               W: [1, 0], S: [-1, 0], A: [0, -1], D: [0, 1],
               ArrowUp: [1, 0], ArrowDown: [-1, 0], ArrowLeft: [0, -1], ArrowRight: [0, 1] };
  var keyHeld = null;
  window.addEventListener('keydown', function (event) {
    if (event.target.tagName === 'INPUT' || event.target.tagName === 'SELECT') return;
    if (event.key === ' ' || event.key === 'Spacebar') {   // space = stop everything now
      keyHeld = null;
      stopJog();
      api('/api/nav', { action: 'pause', reason: 'spacebar' });
      toast('หยุดแล้ว (Space) / stopped', false);
      event.preventDefault();
      return;
    }
    var key = KEYS[event.key];
    if (!key || keyHeld === event.key) return;
    keyHeld = event.key;
    startJog(key[0], key[1]);
    event.preventDefault();
  });
  window.addEventListener('keyup', function (event) {
    if (keyHeld === event.key) { keyHeld = null; stopJog(); }
  });
  window.addEventListener('blur', function () { if (keyHeld) { keyHeld = null; stopJog(); } });

  /* ------------------------------------------------------------------ map */
  var canvas = $('map');
  var ctx = canvas.getContext('2d');

  $('mapFit').addEventListener('click', function () { state.fitRequested = true; drawMap(); });
  $('mapClear').addEventListener('click', function () { state.trail = []; drawMap(); });

  var view = { scale: 4, cx: 0, cy: 0 };

  function fitView(points) {
    if (!points.length) { view.scale = 4; view.cx = 0; view.cy = 0; return; }
    var minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    points.forEach(function (p) {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
    });
    var pad = 6;
    var width = Math.max(maxX - minX, 8) + pad * 2;
    var height = Math.max(maxY - minY, 8) + pad * 2;
    view.scale = Math.min(canvas.width / width, canvas.height / height);
    view.cx = (minX + maxX) / 2;
    view.cy = (minY + maxY) / 2;
  }

  function toScreen(p) {
    return {
      x: canvas.width / 2 + (p.x - view.cx) * view.scale,
      y: canvas.height / 2 - (p.y - view.cy) * view.scale
    };
  }

  /* ------------------------------------------- pointer -> real position */
  /* The canvas is drawn at 900x560 but displayed at whatever width the phone
   * has, so the pointer has to be scaled by the element/backing-store ratio
   * before it can be turned into metres and then into degrees. */
  function eventToCanvas(event) {
    var rect = canvas.getBoundingClientRect();
    return {
      x: (event.clientX - rect.left) * (canvas.width / rect.width),
      y: (event.clientY - rect.top) * (canvas.height / rect.height)
    };
  }

  function screenToEnu(point) {
    return {
      x: view.cx + (point.x - canvas.width / 2) / view.scale,
      y: view.cy - (point.y - canvas.height / 2) / view.scale
    };
  }

  /** Pointer position as a real world coordinate, or null without a datum. */
  function pointerToWorld(event) {
    if (!state.datum) return null;
    var p = screenToEnu(eventToCanvas(event));
    var deg = latLon(p.x, p.y, state.datum);
    var fix = (state.snap && state.snap.fix) || {};
    var world = { x: p.x, y: p.y, lat: deg.lat, lon: deg.lon,
                  distance: null, bearing: null, turn: null };
    if (fix.lat !== undefined) {
      world.distance = haversine(fix.lat, fix.lon, deg.lat, deg.lon);
      world.bearing = bearing(fix.lat, fix.lon, deg.lat, deg.lon);
      world.turn = turnTo(world.bearing);
    }
    return world;
  }

  /** "12.4 m · ทิศ 137° · หมุนขวา 25°" - shared by the hover readout and the menu */
  function describePoint(world, separator) {
    if (world.distance === null) return '';
    var text = 'ระยะ ' + world.distance.toFixed(1) + ' m' +
               ' · ทิศ ' + world.bearing.toFixed(0) + '°';
    if (world.turn !== null) {
      var side = world.turn >= 0 ? 'หมุนขวา' : 'หมุนซ้าย';
      text += ' · ' + side + ' ' + Math.abs(world.turn).toFixed(0) + '°';
    }
    return (separator || ' ') + text;
  }

  /* Canvas cannot use CSS classes, so it reads the same tokens the stylesheet
   * uses. Change the theme once in :root and the map follows. */
  function token(name, fallback) {
    var value = getComputedStyle(document.body).getPropertyValue(name).trim();
    return value || fallback;
  }


  /* Compass rose, top-right of the map.
   *
   * The robot marker already points where the robot is heading, but on a blank
   * field that tells you nothing about WHICH WAY IS NORTH, and nothing about
   * where the heading is coming from. This draws both:
   *
   *   outer ring + N/E/S/W   which way north is
   *   solid needle           the fused heading, what the robot steers by
   *   hollow needle          the raw QMC5883 compass, only when it is fitted
   *                          and healthy, so you can see it disagree
   *   caption                the numeric heading and its source
   *
   * When the two needles separate, the magnetometer is being pulled by
   * something - which is exactly the moment you want to see it, not a number
   * buried on another tab.
   */

  /* ------------------------------ topics -------------------------------
   * The whole ROS graph, in the browser.
   *
   * There is no ros2 CLI on Windows, and inside Docker Desktop the graph is
   * not visible from the host either, so without this the only way to see
   * what the robot is publishing is to shell into the container. Clicking a
   * topic subscribes to it on the robot and shows the live value; the
   * subscription is dropped again once the page stops asking.
   */
  function setupTopics() {
    var body = $('topicsBody');
    if (!body) return;
    var expanded = {};        // topic -> true while its value is open
    var timer = null;

    function render(list) {
      var filter = ($('topicsFilter').value || '').toLowerCase();
      var shown = list.filter(function (t) {
        return !filter || t.name.toLowerCase().indexOf(filter) >= 0 ||
               (t.types || []).join(' ').toLowerCase().indexOf(filter) >= 0;
      });
      $('topicsCount').textContent = shown.length + ' / ' + list.length;
      $('topicsEmpty').classList.toggle('hidden', list.length > 0);

      body.textContent = '';
      shown.forEach(function (t) {
        var tr = document.createElement('tr');
        tr.className = 'topicrow';
        tr.appendChild(cell(t.name, 'mono'));
        tr.appendChild(cell((t.types || []).join(', ').replace(/\/msg\//g, '/'), 'mut'));
        tr.appendChild(cell(t.publishers < 0 ? '?' : String(t.publishers)));
        tr.appendChild(cell(t.subscribers < 0 ? '?' : String(t.subscribers)));
        tr.appendChild(cell(t.hz === null || t.hz === undefined ? '—' : t.hz + ''));

        var td = document.createElement('td');
        var btn = document.createElement('button');
        btn.className = 'btn small';
        btn.textContent = expanded[t.name] ? 'ซ่อน / hide' : 'ดูค่า / watch';
        btn.addEventListener('click', function () {
          if (expanded[t.name]) { delete expanded[t.name]; } else { expanded[t.name] = true; }
          refresh();
        });
        td.appendChild(btn);
        tr.appendChild(td);
        body.appendChild(tr);

        if (expanded[t.name]) {
          var vr = document.createElement('tr');
          var vtd = document.createElement('td');
          vtd.colSpan = 6;
          var pre = document.createElement('pre');
          pre.className = 'topicvalue';
          pre.textContent = 'กำลังรอค่า / waiting…';
          vtd.appendChild(pre);
          vr.appendChild(vtd);
          body.appendChild(vr);
          watch(t.name, pre);
        }
      });
    }

    function cell(text, cls) {
      var td = document.createElement('td');
      if (cls) td.className = cls;
      td.textContent = text;
      return td;
    }

    function watch(name, pre) {
      api('/api/topic?name=' + encodeURIComponent(name)).then(function (d) {
        if (!d || !d.ok) {
          pre.textContent = (d && d.error) || 'ไม่มีข้อมูล / no data';
          return;
        }
        if (d.simulated) {
          pre.textContent = d.note || 'simulated';
          return;
        }
        if (d.value === null || d.value === undefined) {
          pre.textContent = d.note || 'กำลังรอค่า / waiting…';
          return;
        }
        var head = d.type + (d.hz ? '   ' + d.hz + ' Hz' : '') +
                   (d.age_s !== null && d.age_s !== undefined ? '   ' + d.age_s + ' s ago' : '');
        pre.textContent = head + '\n' + JSON.stringify(d.value, null, 2);
      }).catch(function () { /* a dropped poll is not worth shouting about */ });
    }

    function refresh() {
      api('/api/topics').then(function (d) {
        if (d && d.ok) render(d.topics || []);
      }).catch(function () { /* offline: leave the last list on screen */ });
    }

    $('topicsRefresh').addEventListener('click', refresh);
    $('topicsFilter').addEventListener('input', refresh);
    $('topicsAuto').addEventListener('change', schedule);

    function schedule() {
      if (timer) { clearInterval(timer); timer = null; }
      if ($('topicsAuto').checked) timer = setInterval(function () {
        if ($('tab-topics').classList.contains('active')) refresh();
      }, 2000);
    }

    state.refreshTopics = refresh;
    schedule();
    refresh();
  }

  function drawCompass(ctx, canvas, snap) {
    var t = (snap && snap.telemetry) || {};
    var heading = t.heading_deg;
    if (heading === undefined || heading === null) return;

    var r = 40;
    var cx = canvas.width - r - 22;
    var cy = r + 22;

    ctx.save();
    ctx.globalAlpha = 0.92;

    // dial
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fillStyle = cssVar('--card', '#14181f');
    ctx.fill();
    ctx.strokeStyle = cssVar('--line', '#2a323d');
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // cardinal letters
    ctx.font = '10px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    var marks = [['N', 0], ['E', 90], ['S', 180], ['W', 270]];
    for (var i = 0; i < marks.length; i++) {
      var a = marks[i][1] * Math.PI / 180;
      var lx = cx + Math.sin(a) * (r - 9);
      var ly = cy - Math.cos(a) * (r - 9);
      ctx.fillStyle = marks[i][0] === 'N' ? cssVar('--err', '#e0524a')
                                         : cssVar('--mut', '#8b95a3');
      ctx.fillText(marks[i][0], lx, ly);
    }

    // raw magnetometer, only when a healthy compass is actually fitted
    if (t.mag_status === 1 && t.mag_heading_deg !== undefined &&
        t.mag_heading_deg !== null) {
      needle(ctx, cx, cy, r - 16, t.mag_heading_deg,
             cssVar('--warn', '#d79b3c'), false);
    }
    // fused heading: what the robot actually steers by
    needle(ctx, cx, cy, r - 12, heading, cssVar('--acc', '#4c8dff'), true);

    ctx.beginPath();
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
    ctx.fillStyle = cssVar('--txt', '#e6e9ef');
    ctx.fill();

    // numeric heading and where it comes from
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillStyle = cssVar('--txt', '#e6e9ef');
    ctx.fillText(Math.round(((heading % 360) + 360) % 360) + '\u00B0', cx, cy + r + 12);
    // Position uncertainty, if the EKF has anything to say. A map dot that
    // looks equally confident while coasting blind is a lie to the driver.
    if (t.pos_sigma_m !== undefined && t.pos_sigma_m !== null && t.pos_anchored) {
      ctx.font = '9px system-ui, sans-serif';
      ctx.fillStyle = t.pos_sigma_m > 3 ? cssVar('--warn', '#d79b3c')
                                        : cssVar('--mut', '#8b95a3');
      ctx.fillText('\u00B1' + t.pos_sigma_m.toFixed(1) + ' m', cx, cy + r + 48);
    }
    if (t.heading_ref_name) {
      ctx.font = '9px system-ui, sans-serif';
      ctx.fillStyle = cssVar('--mut', '#8b95a3');
      ctx.fillText(t.heading_ref_name, cx, cy + r + 24);
    }
    if (t.mag_status === 2) {
      ctx.font = '9px system-ui, sans-serif';
      ctx.fillStyle = cssVar('--warn', '#d79b3c');
      ctx.fillText('\u0E41\u0E21\u0E48\u0E40\u0E2B\u0E25\u0E47\u0E01\u0E23\u0E1A\u0E01\u0E27\u0E19', cx, cy + r + 36);
    }
    ctx.restore();
  }

  /** One needle at a compass bearing. Filled = the heading in use. */
  function needle(ctx, cx, cy, len, deg, colour, filled) {
    var a = (((deg % 360) + 360) % 360) * Math.PI / 180;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(a);
    ctx.beginPath();
    ctx.moveTo(0, -len);
    ctx.lineTo(5, 6);
    ctx.lineTo(0, 2);
    ctx.lineTo(-5, 6);
    ctx.closePath();
    if (filled) {
      ctx.fillStyle = colour;
      ctx.fill();
    } else {
      ctx.strokeStyle = colour;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
    ctx.restore();
  }

  /** Read a CSS custom property so the map follows the page theme. */
  function cssVar(name, fallback) {
    try {
      var v = getComputedStyle(document.documentElement).getPropertyValue(name);
      return (v && v.trim()) || fallback;
    } catch (e) {
      return fallback;
    }
  }

  function drawMap() {
    var line = token('--line', '#2b3240');
    var muted = token('--mut', '#9aa6b8');
    var accent = token('--acc', '#4da3ff');
    var okColor = token('--ok', '#35c46a');
    var warnColor = token('--warn', '#ffb020');
    var errColor = token('--err', '#ff4d4d');

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    var snap = state.snap || {};
    var fix = snap.fix || {};
    var nav = snap.nav || {};
    var points = [];
    var wpPoints = [];

    if (state.datum) {
      state.waypoints.forEach(function (wp) {
        var p = enu(wp.lat, wp.lon, state.datum);
        p.name = wp.name;
        wpPoints.push(p);
        points.push(p);
      });
      state.trail.forEach(function (p) { points.push(p); });
    }

    if (state.fitRequested) { fitView(points); state.fitRequested = false; }

    // empty state: no datum yet means the map cannot mean anything
    $('mapEmpty').className = state.datum ? 'empty hidden' : 'empty';

    // grid, 5 m spacing (or coarser when zoomed out)
    var step = 5;
    while (step * view.scale < 40) step *= 2;
    ctx.strokeStyle = line;
    ctx.lineWidth = 1;
    ctx.beginPath();
    var half = Math.ceil(Math.max(canvas.width, canvas.height) / view.scale / 2) + step;
    for (var g = -half; g <= half; g += step) {
      var a = toScreen({ x: view.cx + g, y: view.cy - half });
      var b = toScreen({ x: view.cx + g, y: view.cy + half });
      ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
      var c = toScreen({ x: view.cx - half, y: view.cy + g });
      var d = toScreen({ x: view.cx + half, y: view.cy + g });
      ctx.moveTo(c.x, c.y); ctx.lineTo(d.x, d.y);
    }
    ctx.stroke();

    // trail
    if (state.trail.length > 1) {
      ctx.strokeStyle = muted;
      ctx.lineWidth = 2;
      ctx.beginPath();
      state.trail.forEach(function (p, i) {
        var s = toScreen(p);
        if (i === 0) ctx.moveTo(s.x, s.y); else ctx.lineTo(s.x, s.y);
      });
      ctx.stroke();
    }

    // waypoints
    wpPoints.forEach(function (p, i) {
      var s = toScreen(p);
      var isTarget = nav.index === i && nav.state === 'RUNNING';
      ctx.fillStyle = isTarget ? '#ffb020' : accent;
      ctx.beginPath();
      ctx.arc(s.x, s.y, isTarget ? 9 : 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = muted;
      ctx.font = '12px system-ui, sans-serif';
      ctx.fillText((i + 1) + '. ' + (p.name || ''), s.x + 10, s.y - 8);
    });

    // robot: GPS position when there is one, otherwise the dead-reckoned
    // estimate from the hand-set start pose (drawn hollow, it is not a fix)
    var pose = snap.pose || {};
    var robotLat = fix.lat, robotLon = fix.lon, reckoned = false;
    if (robotLat === undefined && pose.lat !== null && pose.lat !== undefined) {
      robotLat = pose.lat;
      robotLon = pose.lon;
      reckoned = pose.source === 'dead_reckoning';
    }

    if (state.datum && robotLat !== undefined) {
      var fixForDraw = { lat: robotLat, lon: robotLon };
      var robot = toScreen(enu(fixForDraw.lat, fixForDraw.lon, state.datum));
      var heading = ((snap.telemetry && snap.telemetry.heading_deg) || 0) * Math.PI / 180;
      ctx.save();
      ctx.translate(robot.x, robot.y);
      ctx.rotate(heading);                 // compass degrees: 0 = up = north
      ctx.beginPath();
      ctx.moveTo(0, -14); ctx.lineTo(9, 10); ctx.lineTo(0, 5); ctx.lineTo(-9, 10);
      ctx.closePath();
      if (reckoned) {                      // hollow orange = estimated, not measured
        ctx.strokeStyle = warnColor;
        ctx.lineWidth = 2;
        ctx.stroke();
      } else {
        ctx.fillStyle = okColor;
        ctx.fill();
      }
      ctx.restore();
      if (reckoned) {
        ctx.fillStyle = warnColor;
        ctx.font = '11px system-ui, sans-serif';
        ctx.fillText('ประมาณจากล้อ/IMU (ไม่มี GPS)', robot.x + 14, robot.y + 4);
      }

      if (nav.state === 'RUNNING' && wpPoints[nav.index]) {
        var target = toScreen(wpPoints[nav.index]);
        ctx.strokeStyle = warnColor;
        ctx.setLineDash([6, 6]);
        ctx.beginPath();
        ctx.moveTo(robot.x, robot.y);
        ctx.lineTo(target.x, target.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }

    // pose arrow being dragged (RViz style: base where it is, arrow where it looks)
    if (state.poseTool.drag && state.datum) {
      var from = toScreen(state.poseTool.drag.from);
      var to = toScreen(state.poseTool.drag.to);
      ctx.strokeStyle = okColor;
      ctx.fillStyle = okColor;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();
      var ang = Math.atan2(to.y - from.y, to.x - from.x);
      ctx.beginPath();                              // arrow head
      ctx.moveTo(to.x, to.y);
      ctx.lineTo(to.x - 14 * Math.cos(ang - 0.4), to.y - 14 * Math.sin(ang - 0.4));
      ctx.lineTo(to.x - 14 * Math.cos(ang + 0.4), to.y - 14 * Math.sin(ang + 0.4));
      ctx.closePath();
      ctx.fill();
      ctx.beginPath();                              // base circle
      ctx.arc(from.x, from.y, 7, 0, Math.PI * 2);
      ctx.stroke();
    }

    // point picked with the mouse / finger
    if (state.picked && state.datum) {
      var pick = toScreen(state.picked);
      ctx.strokeStyle = errColor;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(pick.x - 11, pick.y); ctx.lineTo(pick.x + 11, pick.y);
      ctx.moveTo(pick.x, pick.y - 11); ctx.lineTo(pick.x, pick.y + 11);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(pick.x, pick.y, 6, 0, Math.PI * 2);
      ctx.stroke();
    }

    // scale bar
    ctx.fillStyle = muted;
    ctx.font = '12px system-ui, sans-serif';
    ctx.fillText(step + ' m', 14, canvas.height - 14);
    ctx.strokeStyle = muted;
    ctx.beginPath();
    ctx.moveTo(60, canvas.height - 18);
    ctx.lineTo(60 + step * view.scale, canvas.height - 18);
    ctx.stroke();
  
    // drawn last so the rose sits on top of the map content
    drawCompass(ctx, canvas, snap);
  }

  /* ------------------------------- start pose tool (RViz 2D Pose Estimate) */
  /* Only usable after the operator arms it, because it overrides what the robot
   * believes about itself. Press on the map where the robot really is, drag in
   * the direction it faces, release. Meant for indoor work where there is no
   * GPS and no usable compass. */
  $('mapPoseTool').addEventListener('click', function () {
    if (blockedInViewer()) return;
    state.poseTool.armed = !state.poseTool.armed;
    state.poseTool.drag = null;
    $('mapPoseTool').classList.toggle('armed', state.poseTool.armed);
    canvas.style.cursor = state.poseTool.armed ? 'cell' : 'crosshair';
    $('poseHint').innerHTML = state.poseTool.armed
      ? '<b>พร้อมแล้ว:</b> กดค้างบนแผนที่ตรงที่หุ่นอยู่ ลากไปทางที่หุ่นหันหน้า แล้วปล่อย · กดปุ่มอีกครั้งเพื่อยกเลิก'
      : 'ใช้เมื่อ <b>ไม่มีสัญญาณ GPS</b> (ทดสอบในร่ม): กดปุ่มด้านบนก่อน แล้วกดค้างบนแผนที่ตรงที่หุ่นอยู่ ' +
        'ลากไปทางที่หุ่นหันหน้า แล้วปล่อย — เหมือน “2D Pose Estimate” ใน RViz ของ Nav2';
    closeMapMenu();
  });

  canvas.addEventListener('pointerdown', function (event) {
    if (!state.poseTool.armed) return;
    var world = pointerToWorld(event);
    if (!world) { toast('ยังไม่มีพิกัดอ้างอิงบนแผนที่ / no map reference yet', true); return; }
    state.poseTool.drag = { from: world, to: world };
    canvas.setPointerCapture(event.pointerId);
    event.preventDefault();
  });

  canvas.addEventListener('pointermove', function (event) {
    if (!state.poseTool.drag) return;
    var world = pointerToWorld(event);
    if (!world) return;
    state.poseTool.drag.to = world;
    var arrow = poseArrow();
    $('mapHud').textContent = 'ตั้งตำแหน่งเริ่มต้น / set start pose\n' +
      arrow.lat.toFixed(7) + ', ' + arrow.lon.toFixed(7) + '\nหันหน้า ' + arrow.heading.toFixed(0) + '°';
    drawMap();
  });

  function poseArrow() {
    var drag = state.poseTool.drag;
    var dx = drag.to.x - drag.from.x;
    var dy = drag.to.y - drag.from.y;
    // ENU dx/dy -> compass bearing (0 = north, clockwise)
    var heading = (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
    return { lat: drag.from.lat, lon: drag.from.lon, heading: heading,
             length: Math.sqrt(dx * dx + dy * dy) };
  }

  function finishPose(event) {
    if (!state.poseTool.drag) return;
    var arrow = poseArrow();
    state.poseTool.drag = null;
    if (event) { try { canvas.releasePointerCapture(event.pointerId); } catch (err) { /* ignore */ } }

    if (arrow.length < 0.3) {                 // a tap, not a drag: no direction
      toast('ลากให้ยาวขึ้นเพื่อบอกทิศที่หุ่นหันหน้า / drag further to set the heading', true);
      drawMap();
      return;
    }

    api('/api/set_pose', { lat: arrow.lat, lon: arrow.lon, heading_deg: arrow.heading })
      .then(function (response) {
        if (response && response.ok) {
          state.poseTool.applied = { lat: arrow.lat, lon: arrow.lon, heading: arrow.heading };
          toast('ตั้งตำแหน่งเริ่มต้นแล้ว: ' + arrow.heading.toFixed(0) + '°' +
                (response.simulated ? ' (จำลอง / simulated)' : ''), false);
        } else {
          toast('ตั้งตำแหน่งไม่ได้: ' + ((response && response.error) || ''), true);
        }
      });

    state.poseTool.armed = false;             // one shot, must be re-armed
    $('mapPoseTool').classList.remove('armed');
    canvas.style.cursor = 'crosshair';
    drawMap();
  }

  canvas.addEventListener('pointerup', finishPose);
  canvas.addEventListener('pointercancel', function () { state.poseTool.drag = null; drawMap(); });

  /* --------------------------------------------- map hover and picking */
  /* Hover (or drag a finger) anywhere on the map to read the real latitude and
   * longitude of that spot. Click/tap to drop a marker and choose what to do
   * with it: drive there now, or store it as a waypoint. */
  canvas.addEventListener('pointermove', function (event) {
    if (state.poseTool.drag) return;          // the pose tool draws its own readout
    var world = pointerToWorld(event);
    if (!world) {
      $('mapHud').textContent = 'ยังไม่มีพิกัดอ้างอิง — รอสัญญาณ GPS\nwaiting for the first GPS fix';
      return;
    }
    $('mapHud').textContent =
      world.lat.toFixed(7) + ', ' + world.lon.toFixed(7) + describePoint(world, '\n');
  });

  canvas.addEventListener('pointerleave', function () {
    var fix = (state.snap && state.snap.fix) || {};
    $('mapHud').textContent = fix.lat !== undefined
      ? 'หุ่นอยู่ที่ ' + fix.lat.toFixed(7) + ', ' + fix.lon.toFixed(7)
      : '—';
  });

  canvas.addEventListener('click', function (event) {
    if (state.poseTool.armed || state.poseTool.drag) return;   // pose tool owns the gesture
    var world = pointerToWorld(event);
    if (!world) {
      toast('ยังไม่มีพิกัดอ้างอิง รอ GPS จับสัญญาณก่อน / no GPS reference yet', false);
      return;
    }
    state.picked = world;
    drawMap();
    openMapMenu(event, world);
  });

  function openMapMenu(event, world) {
    var menu = $('mapMenu');
    var wrap = canvas.parentElement.getBoundingClientRect();
    var x = event.clientX - wrap.left;
    var y = event.clientY - wrap.top;

    menu.className = 'mapmenu';                     // show first, then measure
    var width = menu.offsetWidth || 210;
    var height = menu.offsetHeight || 180;
    if (x + width > wrap.width) x = wrap.width - width - 6;
    if (y + height > wrap.height) y = wrap.height - height - 6;
    menu.style.left = Math.max(6, x) + 'px';
    menu.style.top = Math.max(6, y) + 'px';

    $('mapMenuPos').textContent =
      world.lat.toFixed(7) + ', ' + world.lon.toFixed(7) + describePoint(world, '\n');
  }

  function closeMapMenu() {
    $('mapMenu').className = 'mapmenu hidden';
  }

  /* The Clipboard API only exists in a secure context. The robot is served over
   * plain http on a LAN address, so on a phone it is usually missing entirely -
   * and where it does exist it rejects when the page is not focused. Always
   * have the manual fallback ready instead of throwing. */
  function copyCoordinates(text) {
    // last resort: put the numbers on screen, selected, so they can be copied
    // by hand. No window.prompt - a modal blocks the whole page.
    var fallback = function () {
      var field = document.createElement('input');
      field.value = text;
      field.setAttribute('readonly', 'readonly');
      field.style.position = 'fixed';
      field.style.left = '-1000px';
      document.body.appendChild(field);
      field.select();
      var copied = false;
      try { copied = document.execCommand('copy'); } catch (err) { copied = false; }
      document.body.removeChild(field);
      toast(copied ? ('คัดลอกแล้ว: ' + text)
                   : ('พิกัด (จดไว้ได้เลย) / coordinates: ' + text), false);
    };

    try {
      if (window.isSecureContext && navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(text).then(function () {
          toast('คัดลอกแล้ว: ' + text, false);
        }, fallback);
      } else {
        fallback();
      }
    } catch (err) {
      fallback();
    }
  }

  Array.prototype.forEach.call(document.querySelectorAll('[data-map]'), function (button) {
    button.addEventListener('click', function () {
      var world = state.picked;
      if (!world) { closeMapMenu(); return; }

      if (button.dataset.map === 'goto') {
        if (blockedInViewer()) return;
        if (blockedByEmergency()) return;
        api('/api/nav', { action: 'goto_point', lat: world.lat, lon: world.lon })
          .then(function (response) {
            toast(response && response.ok
              ? ''
              : 'สั่งไปจุดนี้ไม่ได้: ' + ((response && response.error) || ''), !(response && response.ok));
          });
      } else if (button.dataset.map === 'add') {
        var points = state.waypoints.slice();
        points.push({
          name: 'WP' + (points.length + 1),
          lat: Number(world.lat.toFixed(7)),
          lon: Number(world.lon.toFixed(7))
        });
        api('/api/waypoints', { waypoints: points }).then(function (response) {
          if (response && response.ok) {
            state.waypoints = response.waypoints || state.waypoints;
            markWaypointsSaved('');
            renderWaypoints();
            drawMap();
            toast('เพิ่มจุดหมายแล้ว ' + state.waypoints.length + ' จุด / waypoint added', false);
          } else {
            toast('เพิ่มไม่ได้: ' + ((response && response.error) || ''), true);
          }
        });
      } else if (button.dataset.map === 'copy') {
        copyCoordinates(world.lat.toFixed(7) + ', ' + world.lon.toFixed(7));
      }
      closeMapMenu();
    });
  });

  // clicking anywhere else, or Esc, closes the menu
  document.addEventListener('click', function (event) {
    if (event.target === canvas) return;
    if ($('mapMenu').contains(event.target)) return;
    closeMapMenu();
  });
  window.addEventListener('keydown', function (event) {
    if (event.key === 'Escape') { closeMapMenu(); state.picked = null; drawMap(); }
  });

  /* ------------------------------------------------------------ waypoints */
  function loadWaypoints() {
    api('/api/waypoints').then(function (response) {
      if (!response.ok) return;
      state.waypoints = response.waypoints || [];
      markWaypointsSaved('');
      renderWaypoints();
    });
  }

  /* "Saved" vs "unsaved" has to be visible, or people lose work silently. */
  function markWaypointsDirty() {
    state.wpDirty = true;
    $('wpMsg').textContent = 'ยังไม่บันทึก — กดปุ่ม “บันทึก” / unsaved changes';
    $('wpSave').classList.add('attention');
  }

  function markWaypointsSaved(message) {
    state.wpDirty = false;
    $('wpMsg').textContent = message || 'บันทึกแล้ว / saved';
    $('wpSave').classList.remove('attention');
  }

  function renderWaypoints() {
    if (!Array.isArray(state.waypoints)) state.waypoints = [];
    var body = $('wpTable').querySelector('tbody');
    body.textContent = '';
    // empty state: say what belongs here and how to make the first one
    var isEmpty = state.waypoints.length === 0;
    $('wpEmpty').className = isEmpty ? 'empty' : 'empty hidden';
    $('wpTable').className = isEmpty ? 'wptable hidden' : 'wptable';
    state.waypoints.forEach(function (wp, index) {
      var row = document.createElement('tr');
      var n = index + 1;
      row.innerHTML =
        '<td>' + n + '</td>' +
        '<td><input data-k="name" aria-label="ชื่อจุดที่ ' + n + ' / name of waypoint ' + n +
          '" value="' + escapeHtml(wp.name || '') + '"></td>' +
        '<td><input data-k="lat" type="number" step="0.0000001" aria-label="latitude ' + n +
          '" value="' + wp.lat + '"></td>' +
        '<td><input data-k="lon" type="number" step="0.0000001" aria-label="longitude ' + n +
          '" value="' + wp.lon + '"></td>' +
        '<td class="wpdist">—</td>' +
        '<td>' +
          '<button class="iconbtn" data-act="go">ไป</button>' +
          '<button class="iconbtn" data-act="up">↑</button>' +
          '<button class="iconbtn" data-act="down">↓</button>' +
          '<button class="iconbtn" data-act="del">✕</button>' +
        '</td>';
      row.querySelectorAll('input').forEach(function (input) {
        input.addEventListener('change', function () {
          var key = input.dataset.k;
          state.waypoints[index][key] = key === 'name' ? input.value : Number(input.value);
          markWaypointsDirty();
        });
      });
      row.querySelectorAll('[data-act]').forEach(function (button) {
        button.addEventListener('click', function () {
          var act = button.dataset.act;
          if (act === 'del') {
            var removed = state.waypoints[index];
            state.waypoints.splice(index, 1);
            markWaypointsDirty();
            offerUndo('ลบ "' + (removed.name || '') + '" แล้ว / deleted', function () {
              state.waypoints.splice(index, 0, removed);
              markWaypointsDirty();
              renderWaypoints();
              drawMap();
            });
          }
          else if (act === 'up' && index > 0) { swap(state.waypoints, index, index - 1); markWaypointsDirty(); }
          else if (act === 'down' && index < state.waypoints.length - 1) { swap(state.waypoints, index, index + 1); markWaypointsDirty(); }
          else if (act === 'go') {
            if (blockedByEmergency()) return;
            api('/api/nav', { action: 'goto', index: index }); return;
          }
          renderWaypoints();
        });
      });
      body.appendChild(row);
    });
    updateWaypointDistances();
  }

  function updateWaypointDistances() {
    var fix = (state.snap && state.snap.fix) || {};
    if (fix.lat === undefined) return;
    var rows = $('wpTable').querySelectorAll('tbody tr');
    var navIndex = (state.snap && state.snap.nav && state.snap.nav.index);
    Array.prototype.forEach.call(rows, function (row, index) {
      var wp = state.waypoints[index];
      if (!wp) return;
      var cell = row.querySelector('.wpdist');
      if (cell) cell.textContent = haversine(fix.lat, fix.lon, wp.lat, wp.lon).toFixed(1) + ' m';
      row.classList.toggle('active', index === navIndex);
    });
  }

  function swap(list, a, b) { var tmp = list[a]; list[a] = list[b]; list[b] = tmp; }
  function escapeHtml(text) {
    return String(text).replace(/[&<>"']/g, function (ch) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch];
    });
  }

  $('wpAddRow').addEventListener('click', function () {
    var fix = (state.snap && state.snap.fix) || {};
    state.waypoints.push({
      name: 'WP' + (state.waypoints.length + 1),
      lat: fix.lat !== undefined ? Number(fix.lat.toFixed(7)) : 0,
      lon: fix.lon !== undefined ? Number(fix.lon.toFixed(7)) : 0
    });
    markWaypointsDirty();
    renderWaypoints();
  });

  $('wpAddHere').addEventListener('click', function () {
    api('/api/waypoints/add_here', {}).then(function (response) {
      if (!response.ok) { $('wpMsg').textContent = 'เพิ่มไม่ได้: ' + response.error; return; }
      state.waypoints = response.waypoints || state.waypoints;
      markWaypointsSaved('เพิ่มจุดจากตำแหน่งปัจจุบันแล้ว / added');
      renderWaypoints();
    });
  });

  $('wpSave').addEventListener('click', function () {
    api('/api/waypoints', { waypoints: state.waypoints }).then(function (response) {
      if (response.ok) {
        state.waypoints = response.waypoints || state.waypoints;
        markWaypointsSaved();
        renderWaypoints();
      } else {
        $('wpMsg').textContent = 'ผิดพลาด: ' + response.error;   // keep the edits
      }
    });
  });

  /* ------------------------------------------------------------- settings */
  var LABELS = {
    'nav.cruise_speed_mps': ['ความเร็วเดินทาง', 'Cruise speed', 'ความเร็วปกติระหว่างวิ่งไปจุดหมาย (m/s)'],
    'nav.approach_speed_mps': ['ความเร็วตอนใกล้ถึง', 'Approach speed', 'ช้าลงเมื่อใกล้จุดหมาย (m/s)'],
    'nav.approach_distance_m': ['ระยะเริ่มชะลอ', 'Approach distance', 'เริ่มชะลอเมื่อเหลือระยะเท่านี้ (m)'],
    'nav.arrive_radius_m': ['รัศมีถือว่าถึง', 'Arrive radius', 'เข้าใกล้เท่านี้ = ถึงจุดหมายแล้ว (m)'],
    'nav.heading_tolerance_deg': ['ความคลาดเคลื่อนทิศ', 'Heading tolerance', 'ถ้าเบี่ยงเกินนี้จะหมุนตัวก่อนวิ่ง (°)'],
    'nav.command_rate_hz': ['ความถี่ส่งคำสั่ง', 'Command rate', 'ครั้งต่อวินาที'],
    'nav.loop_waypoints': ['วนซ้ำ', 'Loop waypoints', 'ถึงจุดสุดท้ายแล้วเริ่มใหม่'],
    'nav.waypoint_timeout_s': ['หมดเวลาต่อจุด', 'Waypoint timeout', 'ไปไม่ถึงภายในเวลานี้จะหยุด (s)'],
    'nav.require_fix': ['ต้องมีสัญญาณ GPS', 'Require fix', 'ปิดได้เฉพาะตอนทดสอบในร่ม'],
    'nav.min_satellites': ['ดาวเทียมขั้นต่ำ', 'Min satellites', 'น้อยกว่านี้จะไม่ยอมวิ่ง'],
    'nav.max_hdop': ['HDOP สูงสุด', 'Max HDOP', 'ค่ายิ่งน้อยยิ่งแม่น (แนะนำ 3)'],
    'nav.fix_timeout_s': ['GPS ขาดได้นานสุด', 'Fix timeout', 'วินาที'],
    'nav.reverse_allowed': ['ถอยหลังได้', 'Reverse allowed', 'ปกติปิดไว้'],

    'motor.accel_ramp_s': ['เวลาเร่งจนสุด', 'Accel ramp', 'วินาที จากหยุดถึงแรงเต็ม ยิ่งมากยิ่งออกตัวนุ่ม'],
    'motor.decel_ramp_s': ['เวลาชะลอจนหยุด', 'Decel ramp', 'วินาที'],
    'motor.ke_v_per_rpm': ['ค่า Ke', 'Ke', 'โวลต์ต่อรอบ/นาที ของเพลาออก (12V/100rpm = 0.12)'],
    'motor.resistance_ohm': ['ความต้านทานรวม', 'Total resistance', 'โอห์ม มอเตอร์+ไดรเวอร์+สายไฟ'],
    'motor.stall_current_a': ['กระแสที่ถือว่าล้อติด', 'Stall current', 'แอมป์'],
    'motor.diode_a_vf_v': ['ไดโอด A แรงดันตก', 'Diode A Vf', 'โวลต์ · Schottky ~0.45, ธรรมดา ~0.8'],
    'motor.diode_a_r_ohm': ['ไดโอด A ความชัน', 'Diode A slope', 'โอห์ม ใส่ 0 ถ้าไม่ทราบ'],
    'motor.diode_b_vf_v': ['ไดโอด B แรงดันตก', 'Diode B Vf', 'โวลต์ · ใส่ค่าของไดโอดตัวที่ใช้จริง'],
    'motor.diode_b_r_ohm': ['ไดโอด B ความชัน', 'Diode B slope', 'โอห์ม'],

    'pid.heading.kp': ['ทิศ Kp', 'Heading Kp', 'แรงดึงกลับเข้าทิศ ถ้าส่ายให้ลดลง'],
    'pid.heading.ki': ['ทิศ Ki', 'Heading Ki', 'แก้การเบี่ยงค้าง'],
    'pid.heading.kd': ['ทิศ Kd', 'Heading Kd', 'หน่วงการส่าย'],
    'pid.heading.kf': ['ทิศ Kf', 'Heading Kf', 'ปกติ 0'],
    'pid.heading.tol': ['ทิศ deadband', 'Heading deadband', 'องศาที่ถือว่าตรงแล้ว'],
    'pid.speed.kp': ['ความเร็ว Kp', 'Speed Kp', ''],
    'pid.speed.ki': ['ความเร็ว Ki', 'Speed Ki', ''],
    'pid.speed.kd': ['ความเร็ว Kd', 'Speed Kd', ''],
    'pid.speed.tol': ['ความเร็ว deadband', 'Speed deadband', 'm/s'],
    'estimator.heading_source': ['แหล่งทิศทาง', 'Heading source', '0=เข็มทิศ IMU, 1=IMU+GPS (แนะนำ), 2=GPS'],
    'estimator.align_min_mps': ['ความเร็วขั้นต่ำเทียบ GPS', 'Align min speed', 'm/s'],
    'estimator.align_tau_s': ['เวลาปรับเข้าหา GPS', 'Align time constant', 'วินาที ยิ่งมากยิ่งนิ่ง'],
    'estimator.gps_tau_s': ['เวลาปรับความเร็ว GPS', 'GPS speed tau', 'วินาที'],
    'estimator.model_tau_s': ['เวลาปรับตอนไม่มี GPS', 'Model tau', 'วินาที'],
    'estimator.track_m': ['ระยะห่างล้อ', 'Wheel track', 'เมตร วัดจากกึ่งกลางล้อซ้ายถึงขวา'],

    'safety.heartbeat_rate_hz': ['ความถี่สัญญาณหัวใจ', 'Heartbeat rate', 'ครั้งต่อวินาทีที่ส่งให้ ESP32'],
    'safety.require_web_heartbeat': ['บังคับให้เว็บต่ออยู่', 'Require web heartbeat', 'ปิด = หุ่นวิ่งต่อแม้ปิดเว็บ (ไม่แนะนำ)'],
    'safety.web_heartbeat_timeout_s': ['เว็บขาดได้นานสุด', 'Web timeout', 'วินาที เกินนี้หุ่นหยุด'],
    'safety.web_heartbeat_grace_s': ['ผ่อนผันหลังต่อใหม่', 'Reconnect grace', 'วินาที'],
    'safety.esp_telemetry_timeout_s': ['ESP เงียบได้นานสุด', 'ESP timeout', 'วินาที'],
    'safety.auto_clear_estop': ['ปลดล็อกอัตโนมัติ', 'Auto clear e-stop', 'ควรปิดไว้เสมอ'],
    'safety.publish_stop_command': ['ส่งคำสั่งหยุดซ้ำ', 'Publish stop command', ''],

    'robot.wheel_diameter_m': ['เส้นผ่านศูนย์กลางล้อ', 'Wheel diameter', 'เมตร (81 mm = 0.081)'],
    'robot.wheel_track_m': ['ระยะห่างล้อ', 'Wheel track', 'เมตร'],
    'robot.max_speed_mps': ['ความเร็วสูงสุด', 'Max speed', 'm/s'],
    'robot.gps_antenna_offset_m': ['ระยะเสา GPS', 'GPS antenna offset', 'เมตร หน้าเพลาล้อ'],

    'web.port': ['พอร์ตเว็บ', 'Web port', 'ต้องรีสตาร์ทโปรแกรมหลังแก้'],
    'web.telemetry_rate_hz': ['ความถี่อัปเดตหน้าจอ', 'Telemetry rate', 'ครั้งต่อวินาที'],
    'web.manual_speed_mps': ['ความเร็วขับเอง', 'Manual speed', 'm/s'],
    'web.manual_turn_dps': ['ความเร็วเลี้ยวขับเอง', 'Manual turn rate', '°/s'],
    'web.max_clients': ['จำนวนเครื่องที่ดูพร้อมกัน', 'Max clients', ''],
    'web.log_lines': ['จำนวนบรรทัดบันทึก', 'Log lines', ''],
    'web.trail_points': ['จำนวนจุดเส้นทาง', 'Trail points', ''],
    'web.allow_config_edit': ['อนุญาตให้แก้ค่า', 'Allow config edit', 'ปิด = ล็อกหน้าตั้งค่า']
  };

  function buildGroupTabs() {
    var box = $('cfgGroups');
    box.textContent = '';
    var names = {
      nav: 'การเดินทาง',        // navigation
      pid: 'PID ควบคุม',        // controller gains
      motor: 'มอเตอร์',          // ramp + electrical model
      estimator: 'เซ็นเซอร์/ทิศทาง',  // which sensor provides heading and speed
      robot: 'ตัวหุ่นยนต์',       // physical robot
      safety: 'ความปลอดภัย',    // watchdogs
      web: 'เว็บ'               // the UI itself
    };
    state.cfg.groups.forEach(function (group) {
      var button = document.createElement('button');
      button.textContent = names[group] || group;
      button.className = group === state.cfg.group ? 'active' : '';
      button.addEventListener('click', function () { loadConfig(group); });
      box.appendChild(button);
    });
  }

  function loadConfig(group) {
    state.cfg.group = group;
    buildGroupTabs();
    api('/api/config?group=' + encodeURIComponent(group)).then(function (response) {
      if (!response.ok) { $('cfgMsg').textContent = response.error; return; }
      state.cfg.data = response.data;
      renderConfig(response.data);
    });
  }

  function renderConfig(data) {
    var form = $('cfgForm');
    form.textContent = '';
    walk(data, '', form);
    var locked = state.snap && state.snap.server && state.snap.server.allow_config_edit === false;
    $('cfgSave').disabled = !!locked;
    $('cfgReset').disabled = !!locked;
  }

  function walk(node, prefix, form) {
    Object.keys(node).forEach(function (key) {
      var value = node[key];
      var path = prefix ? prefix + '.' + key : key;
      if (value !== null && typeof value === 'object' && !Array.isArray(value)) {
        var head = document.createElement('div');
        head.className = 'cfg-section';
        head.textContent = key;
        form.appendChild(head);
        walk(value, path, form);
        return;
      }
      form.appendChild(configField(path, value));
    });
  }

  function configField(path, value) {
    var short = path.split('.').slice(-2).join('.');
    var meta = LABELS[path] || LABELS[short] || [path.split('.').pop(), '', ''];
    var item = document.createElement('div');
    item.className = 'cfg-item';

    var label = document.createElement('label');
    label.textContent = meta[0];
    if (meta[1]) {
      var en = document.createElement('span');
      en.className = 'en';
      en.textContent = ' · ' + meta[1];
      label.appendChild(en);
    }
    item.appendChild(label);

    var input = document.createElement('input');
    input.dataset.path = path;
    if (typeof value === 'boolean') {
      input.type = 'checkbox';
      input.checked = value;
      input.dataset.type = 'bool';
    } else if (typeof value === 'number') {
      input.type = 'number';
      input.step = 'any';
      input.value = value;
      input.dataset.type = 'number';
    } else {
      input.type = 'text';
      input.value = value === null ? '' : String(value);
      input.dataset.type = 'text';
    }
    item.appendChild(input);

    if (meta[2]) {
      var desc = document.createElement('div');
      desc.className = 'desc';
      desc.textContent = meta[2];
      item.appendChild(desc);
    }
    return item;
  }

  $('cfgSave').addEventListener('click', function () {
    var data = JSON.parse(JSON.stringify(state.cfg.data || {}));
    var inputs = $('cfgForm').querySelectorAll('input[data-path]');
    Array.prototype.forEach.call(inputs, function (input) {
      var parts = input.dataset.path.split('.');
      var node = data;
      for (var i = 0; i < parts.length - 1; i++) node = node[parts[i]];
      var key = parts[parts.length - 1];
      if (input.dataset.type === 'bool') node[key] = input.checked;
      else if (input.dataset.type === 'number') node[key] = Number(input.value);
      else node[key] = input.value;
    });
    api('/api/config', { group: state.cfg.group, data: data }).then(function (response) {
      $('cfgMsg').textContent = response.ok
        ? 'บันทึกแล้ว มีผลทันที / saved and applied'
        : ('ผิดพลาด: ' + response.error);
      if (response.ok) { state.cfg.data = response.data; renderConfig(response.data); }
    });
  });

  $('cfgReload').addEventListener('click', function () { loadConfig(state.cfg.group); });

  $('cfgReset').addEventListener('click', function () {
    if (!window.confirm('คืนค่าโรงงานของกลุ่ม "' + state.cfg.group + '" ใช่ไหม?')) return;
    api('/api/config/reset', { group: state.cfg.group }).then(function (response) {
      $('cfgMsg').textContent = response.ok ? 'คืนค่าโรงงานแล้ว' : ('ผิดพลาด: ' + response.error);
      if (response.ok) { state.cfg.data = response.data; renderConfig(response.data); }
    });
  });

  /* ------------------------------------------------------------------ log */
  function addLog(entry) {
    var box = $('logBox');
    var line = document.createElement('div');
    line.innerHTML = '<span class="t">' + escapeHtml(entry.t || '') + '</span>' +
                     '<span class="src">' + escapeHtml(entry.source || '') + '</span>' +
                     escapeHtml(entry.message || '');
    box.appendChild(line);
    while (box.childElementCount > LOG_MAX) box.removeChild(box.firstElementChild);
    box.scrollTop = box.scrollHeight;
  }

  /* ----------------------------------------------------------------- boot */
  api('/api/state').then(function (response) {
    if (response.ok && response.state) applyServerMode(response.state.server);
  });
  api('/api/logs').then(function (response) {
    if (response.ok) response.logs.forEach(addLog);
  });
  var savedView = 'simple';
  try { savedView = window.localStorage.getItem('gpsLocalizeView') || 'simple'; } catch (err) { /* ignore */ }
  setView(savedView);
  loadWaypoints();
  buildGroupTabs();
  setupTopics();
  setConnection('connecting');
  connect();
  setInterval(heartbeat, 250);
  setInterval(watchFreshness, 1000);   // degrade to stale/offline on its own
  heartbeat();
  window.addEventListener('beforeunload', function () { stopJog(); });
})();
