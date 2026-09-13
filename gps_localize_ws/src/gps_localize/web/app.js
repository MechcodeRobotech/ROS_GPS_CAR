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
  // Defaults only. The robot sends the configured values in every snapshot and
  // applyServerConfig() below adopts them - these three used to be hardcoded
  // here while web.yaml and the Settings page pretended to control them.
  var TRAIL_MAX = 300;
  var jogRepeatMs = 150;
  var heartbeatMs = 250;
  var heartbeatTimer = null;
  var LOG_MAX = 200;

  var state = {
    snap: null,
    datum: null,           // client side map origin {lat, lon}
    trail: [],
    waypoints: [],
    fitRequested: true,
    basemap: 'street',
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
    // Read from the board's parameters, so it follows a wheel change.
    wheelDiameterM: null,
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
                    $('btnMagStart'), $('btnMagFinish'), $('btnMagClear'),
                    $('btnImuStart'), $('btnImuSave'), $('btnImuClear'), $('btnImuRetry'),
                    $('btnMagCancel'), $('btnImuCancel'),
                    $('btnCalibResetOdom')];
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

  
  var lastAlertTone = null;
  function playAlertSound() {
    if (!window.AudioContext && !window.webkitAudioContext) return;
    try {
      var ctx = new (window.AudioContext || window.webkitAudioContext)();
      var osc = ctx.createOscillator();
      var gain = ctx.createGain();
      osc.connect(gain);
      gain.connect(ctx.destination);
      osc.type = 'square';
      osc.frequency.setValueAtTime(880, ctx.currentTime);
      osc.frequency.setValueAtTime(1100, ctx.currentTime + 0.2);
      gain.gain.setValueAtTime(0.1, ctx.currentTime);
      gain.gain.linearRampToValueAtTime(0, ctx.currentTime + 0.5);
      osc.start(ctx.currentTime);
      osc.stop(ctx.currentTime + 0.5);
    } catch (e) { /* ignored, probably blocked by browser policy */ }
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
  /** Switch tabs. Named rather than inline so a button on one page can send you
   *  to another - "edit these →" on the Calibration tab opens the parameter
   *  list already filtered, instead of telling you to go and find the field. */
  function showTab(name) {
    var button = document.querySelector('.tab[data-tab="' + name + '"]');
    var page = $('tab-' + name);
    if (!button || !page) return;
    Array.prototype.forEach.call(document.querySelectorAll('.tab'), function (t) {
      t.classList.remove('active');
    });
    Array.prototype.forEach.call(document.querySelectorAll('.tabpage'), function (p) {
      p.classList.remove('active');
    });
    button.classList.add('active');
    page.classList.add('active');
    if (name === 'map') { state.fitRequested = true; drawMap(); }
    if (name === 'board') loadParams();
    if (name === 'help') { loadHelp(true); refreshFindRobot(); }
    if (name === 'cfg' && !state.cfg.data) loadConfig(state.cfg.group);
    if (name === 'wp') loadWaypoints();
    // The wheel diameter and the track width shown on the Calibration tab are
    // parameters, and the board is the only place that knows them.
    if (name === 'calib' && !(paramsState && paramsState.entries.length)) loadParams();
    if (name === 'geom') {
      if (paramsState && paramsState.entries.length) renderGeometry(); else loadParams();
    }
  }

  Array.prototype.forEach.call(document.querySelectorAll('.tab'), function (tab) {
    tab.addEventListener('click', function () { showTab(tab.dataset.tab); });
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
  /* One view, everything visible. The Simple/Advanced switch hid 18 elements
   * and DEFAULTED to Simple - so a control that plainly exists was simply
   * absent, with nothing on screen to explain why. The .adv classes stay in the
   * markup as a note about what is detail rather than essential; nothing acts
   * on them now. */
  function setView() {
    document.body.dataset.view = 'advanced';
    try { window.localStorage.removeItem('gpsLocalizeView'); } catch (err) { /* private mode */ }
  }

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

    // FIRST, because it is the only fault where pressing the emergency does not
    // remove power. Every other reason here describes a robot that is safely
    // stopped; this one describes one that is not.
    if (has('contactor_stuck')) {
      return {
        clearable: false,
        title: 'อันตราย - รีเลย์ค้าง / DANGER - contactor is stuck closed',
        detail: 'The contactor was commanded open but the motor rails are still '
              + 'live. The emergency stop CANNOT remove power in this state. '
              + 'Disconnect the battery by hand before going near the wheels.'
      };
    }
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
    if (has('battery_soft')) {
      return {
        clearable: false,
        title: 'แบตเตอรี่อ่อน (ตัดกำลังมอเตอร์ชั่วคราว) / Battery Soft Stop',
        detail: 'หุ่นยนต์ตัดไฟมอเตอร์อัตโนมัติเนื่องจากแบตเตอรี่เริ่มอ่อน\n\n'
              + 'The battery dropped below the soft-stop threshold. The robot cannot drive '
              + 'until the voltage recovers or the battery is swapped.'
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
      // No STOP here. Ending a mission belongs with the mission, next to the
      // waypoint list it ends - the same reason START moved there. This card
      // keeps the actions that resume or recover, which are the ones you want
      // under your hand without scrolling. The EMERGENCY STOP is a different
      // thing entirely and is in the toolbar on every page.
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

    
    if (tone === 'bad' && lastAlertTone !== 'bad') {
      playAlertSound();
    }
    lastAlertTone = tone;
    
    $('heroCard').className = 'hero card ' + tone;

    $('heroIcon').textContent = icon;
    text('heroTitle', title);
    text('heroSub', sub);

    /* STARTING a mission is the Mission card's job, not this one's.
     *
     * There were two buttons called START on one screen and the reader had to
     * work out whether they differed. They did not. The one that stayed is the
     * one beside the waypoint list it acts on, which is where someone is
     * looking when they set a mission up.
     *
     * This button keeps every OTHER action it had - resume after a pause, and
     * unlatching an emergency stop - because those have nowhere else to live
     * and the second of them is the one you want under your hand. It keeps its
     * space when it has nothing to offer rather than vanishing, so the card
     * does not change height and move the controls below it.
     */
    var button = $('heroPrimary');
    var startsMission = primary.action === 'start';
    button.className = startsMission ? 'btn go hero-primary invisible'
                                     : 'btn go hero-primary';
    button.textContent = primary.label;
    button.disabled = startsMission || !primary.enabled;
    button.dataset.action = primary.action;
    button.title = primary.enabled ? '' :
      (state.waypoints.length ? '' : 'เพิ่มจุดหมายก่อนจึงจะเริ่มได้ / add a waypoint first');

    var second = $('heroSecondary');
    if (secondary) {
      second.className = 'btn';
      second.disabled = false;
      second.textContent = secondary.label;
      second.dataset.action = secondary.action;
    } else {
      // KEEPS ITS SPACE. `hidden` is display:none, so this button vanishing when
      // the state changed shrank the hero card and pulled the whole page - drive
      // pad included - upwards. The state changes the instant you press move,
      // which is the worst possible moment for the controls to jump.
      // visibility:hidden hides it just as well and holds the layout still.
      second.className = 'btn invisible';
      second.disabled = true;
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

  /** (Re)start the heartbeat timer at the configured period. */
  function armHeartbeat(ms) {
    if (heartbeatTimer) clearInterval(heartbeatTimer);
    heartbeatMs = ms;
    heartbeatTimer = setInterval(heartbeat, heartbeatMs);
  }

  /** Adopt the three browser-side settings the robot is configured with.
   *
   * The jog repeat is picked up by the next press, because that timer is
   * created per press. The heartbeat timer is long-lived, so it is re-armed
   * only when the period actually changes - re-arming it on every frame would
   * mean it never fired.
   */
  function applyServerConfig(server) {
    if (!server) return;
    if (server.trail_points > 0) TRAIL_MAX = server.trail_points;
    if (server.manual_repeat_ms > 0) jogRepeatMs = server.manual_repeat_ms;
    var hz = server.heartbeat_rate_hz;
    if (hz > 0) {
      var ms = Math.round(1000 / hz);
      if (ms !== heartbeatMs) armHeartbeat(ms);
    }
  }

  /* -------------------------------------------------------------- snapshot */
  function onSnapshot(snap) {
    state.snap = snap;
    applyServerConfig(snap.server);
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
    // The ESP32 pill answers "is the board there", so it says so in words. An
    // age in seconds is the evidence for that answer, not the answer itself -
    // and "325.3 s" needed reading and interpreting before it meant anything,
    // which is the opposite of what a glance-at-it row is for. The number stays
    // in the tooltip for when it is genuinely wanted.
    /* The ESP32 pill shows the AGE of the last frame, because that is the
     * number that says whether the robot is still there - and while it is
     * healthy the number is the reassurance: 0.1 s, ticking, means the link is
     * alive right now. Replacing it with the word "ok" threw that away.
     *
     * When the link drops, the same number becomes how long it has been gone,
     * and that is worth spelling out in a unit anyone can act on - "ขาด 2 นาที"
     * rather than "137.4 s". */
    var espAge = snap.telemetry_age_s;
    var espLive = espAge !== null && espAge < 1.5;
    var espText;
    if (espAge === null) {
      espText = 'ไม่พบ / none';
    } else if (espLive) {
      espText = espAge.toFixed(1) + ' s';
    } else if (espAge < 90) {
      espText = 'ขาด ' + Math.round(espAge) + ' s';
    } else {
      espText = 'ขาด ' + Math.round(espAge / 60) + ' นาที';
    }
    setPill('pillEsp', espLive ? 'ok' : 'bad', espText,
      espAge === null ? 'no telemetry has ever arrived'
                      : 'last frame ' + espAge.toFixed(1) + ' s ago');
    // HDOP belongs in the pill, not only in a tooltip: a satellite count with
    // no dilution figure says how many were seen, not how good the fix is, and
    // those come apart exactly when it matters.
    var sats = Math.round(t.gps_satellites || 0);
    setPill('pillGps', t.gps_fix_quality > 0 ? (sats >= 6 ? 'ok' : 'warn') : 'bad',
      sats + ' ดาว · HDOP ' + fmt(t.gps_hdop, 1),
      sats + ' satellites, HDOP ' + fmt(t.gps_hdop, 1));
    // The control loop, on the top bar where it was. It is the first thing that
    // shows the firmware is in trouble - it went from 4 ms to 178 ms today and
    // nothing else on this bar moved - and it is worth a glance while driving.
    var loopMs = (t.loop_us || 0) / 1000;
    setPill('pillLoop', loopMs > 9 ? 'bad' : (loopMs > 6 ? 'warn' : 'ok'),
      loopMs >= 10 ? loopMs.toFixed(0) + ' ms' : loopMs.toFixed(1) + ' ms',
      'control loop, one tick. The period is 10 ms, so anything near that is '
      + 'the firmware running out of time.');
    var volts = t.battery_v || 0;
    var _cells = 3, _cutoff = 11.2, _warn = 11.7;
    if (typeof paramsState !== 'undefined' && paramsState.entries && paramsState.entries.length) {
      paramsState.entries.forEach(function (p) {
        if (p.key === 'battery.cutoff_v') _cutoff = p.value;
        if (p.key === 'battery.warn_v') _warn = p.value;
      });
    }
    setPill('pillBatt', volts > _warn ? 'ok' : (volts > _cutoff ? 'warn' : 'bad'), fmt(volts, 2) + ' V');

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
    text('tRollRate', fmt(t.roll_rate_dps, 1));
    text('tPitchRate', fmt(t.pitch_rate_dps, 1));
    text('tYawRate', fmt(t.yaw_rate_dps, 1));
    // ONLY what the IMU itself provides.
    //
    // This row first showed heading_deg and the ESP32 die temperature, which was
    // badly wrong: heading_deg is the FUSED heading and was coming from the
    // compass, and the die temperature is the chip's, not the sensor's. So the
    // row showed plausible numbers under an "IMU" heading while the IMU was
    // completely dead - the exact mistake this project keeps having to unlearn.
    //
    // yaw_rate_dps is the gyro, and it is the one field here that genuinely
    // comes from the BNO085. When the IMU is missing, say so instead of printing
    // a zero that looks like "not turning".
    var imuDead = (t.missing_sensors || []).indexOf('imu_missing') >= 0;
    text('tImuRpy', imuDead ? '✕ ไม่พบ / not reporting' : (fmt(t.imu_roll_deg, 1) + '°, ' + fmt(t.imu_pitch_deg, 1) + '°, ' + fmt(t.imu_yaw_deg, 1) + '°'));
    text('tImuAcc', imuDead ? '—'
      : ((t.missing_sensors || []).indexOf('imu_magnetometer_missing') < 0
          ? 'mag ok'
          // "mag < 2" was a guess dressed as a fact. The flag it reads means
          // the magnetometer-referenced heading is UNUSABLE, and that has two
          // completely different causes: the sensor not being calibrated, or
          // its report not arriving. The Calibration tab was showing a proud
          // 3/3 while this line insisted the accuracy was below 2. Only one of
          // them was reading the accuracy.
          : (t.imu_accuracy >= 2
              ? 'mag ' + t.imu_accuracy + '/3 แต่รายงานไม่มา / calibrated, '
                + 'but its report is not arriving'
              : 'mag ' + (t.imu_accuracy || 0) + '/3 ยังไม่พอ / needs 2 to be used')));
    // Delivered against asked, per report. The verdict matters more than the
    // numbers: "the bus is full" and "the magnetometer is broken" look the same
    // on a dashboard, and telling them apart is the whole reason this row
    // exists. rv is the 10 Hz one and it is the one that starves first.
    var rvWant = 10, fastWant = 100, accWant = 50;

    // CPU temperature on its own line in the power card, where someone actually
    // looks for it - and judged, not just printed. The number alone means
    // nothing to most people; "hot" or "too hot" does. The ESP32 throttles
    // around 80 C and its absolute maximum is 125 C.
    var temp = t.esp32_temp_c;
    if (temp === null || temp === undefined || isNaN(temp)) {
      text('tCpuTemp', '—');
      // Reported as NaN until the reading is seen to change: most WROOM-32
      // silicon has the internal sensor fused off, and a fixed 53.3 C would
      // look like a real measurement.
      text('tCpuTempNote', 'เซ็นเซอร์ไม่รายงาน / this chip has no usable temperature sensor');
    } else {
      text('tCpuTemp', fmt(temp, 1));
      text('tCpuTempNote', temp >= 80 ? '⚠ ร้อนเกิน / too hot, the chip will throttle'
                         : temp >= 65 ? 'อุ่น / warm'
                         : 'ปกติ / normal');
    }

    renderSensors(t);
    renderHero(snap);
    checkFirmware(snap);

    var bv = t.battery_v || 0;
    text('tBattV', fmt(bv, 2));
    var statEl = $('tBattStat');
    if (statEl) {
      if (bv > 0) {
        var cells = 3;
        var cutoff_v = 11.2;
        var warn_v = 11.7;
        if (typeof paramsState !== 'undefined' && paramsState.entries && paramsState.entries.length) {
          paramsState.entries.forEach(function (p) {
            if (p.key === 'battery.cells') cells = p.value;
            if (p.key === 'battery.cutoff_v') cutoff_v = p.value;
            if (p.key === 'battery.warn_v') warn_v = p.value;
          });
        }
        
        var p100 = cells * 4.2;
        var p75 = cells * 4.0;
        var p50 = cells * 3.85;
        var p25 = warn_v;
        var p0 = cutoff_v;

        var pct = 0;
        if (bv >= p100) pct = 100;
        else if (bv >= p75) pct = 75 + ((bv - p75) / (p100 - p75)) * 25;
        else if (bv >= p50) pct = 50 + ((bv - p50) / (p75 - p50)) * 25;
        else if (bv >= p25) pct = 25 + ((bv - p25) / (p50 - p25)) * 25;
        else if (bv >= p0) pct = ((bv - p0) / (p25 - p0)) * 25;
        else pct = 0;
        
        var statusStr = Math.round(pct) + '%';
        if (bv < p0) {
          statEl.style.color = '#ff6060';
          statusStr += ' (หมด! / DEAD)';
        } else if (bv < warn_v) {
          statEl.style.color = '#ffb020';
          statusStr += ' (ต่ำ! / LOW)';
        } else {
          statEl.style.color = ''; 
          statusStr += ' (ปกติ / OK)';
        }
        statEl.textContent = statusStr;
      } else {
        statEl.style.color = '';
        statEl.textContent = '—';
      }
    }
    // Total draw on the pack = every INA added together.
    //
    // Each INA226 measures the current on the BATTERY side of its own buck, so
    // all four are drawing from the same pack and they sum. Showing only the
    // two motor rails - as this did at first - undercounted by the processor
    // and the fans, which together are around 0.4 A: not a rounding error when
    // you are working out how long a charge will last. (The voltages are each
    // measured AFTER their own buck, so those are per-rail and do not sum.)
    var parts = [t.battery_a, t.fan_a, t.motor_a_a, t.motor_b_a];
    var packA = 0, haveAll = true;
    parts.forEach(function (v) {
      if (typeof v === 'number' && !isNaN(v)) { packA += v; } else { haveAll = false; }
    });
    text('tBattA', haveAll ? fmt(packA, 2) : '—');
    // INA 0x41: the rail the ESP32 and every sensor run from - 3V3 since the
    // board was moved onto its own buck.
    text('tProcV', fmt(t.esp32_rail_v, 3));
    text('tProcA', fmt(t.battery_a, 3));
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
    // status < 0 is NavSatStatus.STATUS_NO_FIX. A receiver with no lock still
    // publishes a latitude and longitude, and those numbers are nonsense - seen
    // here as -11.23, 86.52, the middle of the Indian Ocean, from a robot in
    // Chiang Mai. Testing only for non-zero let that straight through.
    //
    // It matters far beyond a misplaced dot: the FIRST accepted fix becomes the
    // map's datum, every local coordinate is measured from it, and a datum taken
    // from garbage silently puts the whole map - waypoints, trail and satellite
    // imagery alike - in the wrong hemisphere.
    var fixUsable = fix.lat !== undefined && fix.lon !== undefined
                 && (fix.lat !== 0 || fix.lon !== 0)
                 && (fix.status === undefined || fix.status === null || fix.status >= 0);
    if (fixUsable) {
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

  /* ------------------------------------------------------- help / connect */
  /* The Help panel was pure markup: qrBox, cxMdns, cxLan, cxLocal and cxNote
   * existed in index.html and nothing in this file ever wrote to them, so the
   * QR stayed as its "..." placeholder and every address read "—". The server
   * had been serving a perfectly good QR from /api/qr the whole time. */
  var helpLoaded = false;

  /* Find-the-robot, polled on its own.
   *
   * The web never looked for the board at all - the connection only goes one
   * way - so when it went quiet the page could say nothing beyond "no
   * telemetry", and answering "is it even on the network?" meant running avahi
   * and a ping sweep by hand. Four facts, because they fail independently and
   * each points somewhere different: linked, on the network, and whether we are
   * announcing the agent for it to find.
   *
   * Polled slowly, and only while the robot is NOT linked - once telemetry is
   * flowing the question is answered and the lookup is just noise.
   */
  function refreshFindRobot() {
    return api('/api/find_robot').then(function (r) {
      if (!r || !r.ok) return;
      text('frLinked', r.linked ? 'ใช่ / yes' : 'ยัง / not yet');
      text('frFound', r.found_at
        ? (r.found_at + '  (' + (r.robot_name || '') + ')')
        : 'ไม่เจอ / not found');
      text('frBeacon', r.beacon
        ? ('กำลังประกาศ / yes, as ' + (r.agent_ip || '?') + ':8888')
        : 'ไม่ได้ประกาศ / NO - the beacon service is not running');
      text('frHint', r.hint || '');
    });
  }

  if ($('frNow')) $('frNow').addEventListener('click', refreshFindRobot);

  // Keep looking while the robot is missing, and stop once it is here.
  setInterval(function () {
    var snap = state.snap || {};
    var age = snap.telemetry_age_s;
    var linked = (age !== undefined && age !== null && age < 5);
    var help = $('tab-help');
    if (!linked || (help && help.classList.contains('active'))) refreshFindRobot();
  }, 10000);

  function loadHelp(force) {
    if (helpLoaded && !force) return;
    helpLoaded = true;
    api('/api/connect').then(function (r) {
      var c = (r && r.connect) || {};
      text('cxMdns', c.mdns || '—');
      text('cxLan', c.lan || '—');
      text('cxLocal', c.local || '—');
      // mDNS is the nicer address but it needs multicast, which guest Wi-Fi,
      // VLANs and VPNs routinely drop. Say so rather than letting someone
      // conclude the robot is down when it is the name that failed.
      text('cxNote', c.mdns_may_be_blocked
        ? 'ถ้าชื่อ .local ใช้ไม่ได้ ให้พิมพ์เลข IP แทน · if the .local name does '
          + 'not resolve, type the IP address instead - some networks block mDNS.'
        : '');
    });
    api('/api/qr').then(function (r) {
      var box = $('qrBox');
      if (!box) return;
      if (r && r.ok && r.svg) {
        box.innerHTML = r.svg;
        // The QR encodes the IP, not the .local name, on purpose: the person
        // scanning it is standing here now and cannot debug a name lookup.
        box.setAttribute('title', r.url || '');
      } else {
        box.textContent = 'สร้าง QR ไม่ได้ / could not build a QR code';
      }
    });
  }

  /** "123 rpm · 0.52 m/s", using the wheel diameter the board reports. */
  function wheelRpmAndMps(rpm) {
    if (rpm === null || rpm === undefined || isNaN(rpm)) { return '—'; }
    var d = state.wheelDiameterM;
    if (!d) { return fmt(rpm, 0) + ' rpm'; }      // diameter not known yet
    var mps = rpm / 60 * Math.PI * d;
    return fmt(rpm, 0) + ' rpm · ' + fmt(mps, 2) + ' m/s';
  }

  /* ------------------------------------------------- sensor health chips */
  /* One chip per sensor. The robot keeps driving when one of them dies - the
   * chips show which source the heading and the speed fell back to. */
  function renderSensors(t) {
    var missing = t.missing_sensors || [];
    var has = function (name) { return missing.indexOf(name) < 0; };
    /* ALWAYS THE SAME CHIPS, IN THE SAME ORDER.
     *
     * These sit above the drive controls, so anything that changes their NUMBER
     * changes their height and moves the jog pad down the page - a fault
     * appearing mid-drive shifted the buttons out from under the user's thumb.
     * Appending fault chips only when a fault existed did exactly that.
     *
     * Making the strip scroll sideways instead stopped the movement but hid the
     * warnings off the right-hand edge, which is worse - a warning you cannot
     * see is not a warning. So the set is FIXED: every chip is always present
     * and only its colour and wording change. Constant count, constant height,
     * nothing hidden, nothing moves. */
    var faults = t.motor_fault_names || [];
    var faulted = function (name) { return faults.indexOf(name) >= 0; };

    /* A chip's LABEL never changes and its STATE is one glyph in a fixed track.
     *
     * The wording used to carry the detail - "motor L \u2715 \u0e44\u0e21\u0e48\u0e01\u0e34\u0e19\u0e44\u0e1f / open" - and
     * that made every chip a different width depending on what was wrong. The
     * row then re-wrapped, the block changed height, and the drive pad below it
     * moved. Reserving more rows only traded a moving pad for a wall of chips
     * two hundred pixels tall, which is what it briefly became.
     *
     * So the chip answers one question - is this thing all right - and nothing
     * else. Every detail it used to carry is still on the page, in the row that
     * already existed for it: which heading reference is live is in "Heading
     * from", the compass being disturbed is in the compass row, and what to do
     * about a motor fault is in the advice line under these chips. Nothing was
     * hidden to make this fit; it was moved to where it was already explained.
     */
    /* Labels are SHORT on purpose. "\u0e21\u0e2d\u0e40\u0e15\u0e2d\u0e23\u0e4c\u0e0b\u0e49\u0e32\u0e22 / Motor L" is honest and it did
     * not fit: in two columns it rendered as "\u0e21\u0e2d\u0e40\u0e15\u0e2d\u0e23\u0e4c......", which names
     * nothing. A chip has one job - say whether this thing is all right - and a
     * label the reader cannot finish reading cannot do that job. The long form
     * lives in the tooltip, where there is room for it. */
    function chip(label, state, why) { return [label, state, why]; }
    function motorChip(label, stalled, open) {
      return chip(label, (open || stalled) ? 'bad' : 'ok',
                  open ? '\u0e44\u0e21\u0e48\u0e01\u0e34\u0e19\u0e44\u0e1f\u0e40\u0e25\u0e22 / no current at all - see the advice below'
                : stalled ? '\u0e01\u0e34\u0e19\u0e44\u0e1f\u0e41\u0e15\u0e48\u0e44\u0e21\u0e48\u0e2b\u0e21\u0e38\u0e19 / drawing current but not turning'
                : '\u0e1b\u0e01\u0e15\u0e34 / no fault reported');
    }

    // The BNO085's magnetometer is two separate questions and they were being
    // answered as one: is it CALIBRATED, and is its report ARRIVING. Accuracy
    // 3/3 with no report reaching us is not a bad magnetometer - it is a bus or
    // a driver problem - and showing it as a red cross next to a perfectly
    // calibrated sensor is how hours get spent hunting magnets.
    var magAcc = t.imu_accuracy;
    var magUsable = has('imu_magnetometer_missing');
    var magState = magUsable ? 'ok' : (magAcc >= 2 ? 'warn' : 'bad');

    var chips = [
      chip('IMU', has('imu_missing') ? 'ok' : 'bad',
           'BNO085 gyro and rotation vector'),
      // A cross is now "no position at all", not "the fix is too rough to take a
      // speed from". Those were the same flag, and a real 7-satellite fix at
      // HDOP 1.8 was being drawn as a dead sensor while the robot navigated on
      // it. The firmware answers the two questions separately; the tooltip says
      // which one this is, and names the satellite count so the panel and the
      // top bar cannot appear to contradict each other.
      chip('GPS', has('gps_missing') ? 'ok' : 'bad',
           has('gps_missing')
             ? 'satellite fix present (' + (t.gps_satellites || 0) + ' satellites, HDOP '
               + fmt(t.gps_hdop, 1) + '). A rough fix still counts - how much to '
               + 'trust it is what HDOP is for'
             : 'no position at all - fewer than 4 satellites, or no fix. Indoors '
               + 'this is normal'),
      chip('\u0e40\u0e02\u0e47\u0e21\u0e17\u0e34\u0e28',
           t.mag_status === 2 ? 'warn' : (t.mag_status === 1 ? 'ok' : 'bad'),
           t.mag_status === 2 ? '\u0e2a\u0e19\u0e32\u0e21\u0e41\u0e21\u0e48\u0e40\u0e2b\u0e25\u0e47\u0e01\u0e23\u0e1a\u0e01\u0e27\u0e19 / magnetic disturbance'
                              : 'QMC5883L in the GPS module'),
      chip('IMU mag', magState,
           magUsable ? 'calibrated and reporting'
             : (magAcc >= 2
                ? '\u0e1b\u0e23\u0e31\u0e1a\u0e40\u0e17\u0e35\u0e22\u0e1a\u0e41\u0e25\u0e49\u0e27 ' + magAcc + '/3 \u0e41\u0e15\u0e48\u0e23\u0e32\u0e22\u0e07\u0e32\u0e19\u0e44\u0e21\u0e48\u0e21\u0e32 / calibrated '
                  + magAcc + '/3 but its report is not arriving'
                : '\u0e22\u0e31\u0e07\u0e1b\u0e23\u0e31\u0e1a\u0e40\u0e17\u0e35\u0e22\u0e1a\u0e44\u0e21\u0e48\u0e1e\u0e2d / accuracy ' + (magAcc || 0) + '/3, needs 2')),
      chip('\u0e17\u0e34\u0e28\u0e17\u0e32\u0e07',
           missing.indexOf('heading_relative_only') >= 0 ? 'warn' : 'ok',
           missing.indexOf('heading_relative_only') >= 0
             ? '\u0e44\u0e21\u0e48\u0e21\u0e35\u0e17\u0e34\u0e28\u0e08\u0e23\u0e34\u0e07 / no absolute reference - gyro only, it drifts'
             : '\u0e21\u0e35\u0e17\u0e34\u0e28\u0e08\u0e23\u0e34\u0e07 / locked to an absolute reference'),
      motorChip('\u0e25\u0e49\u0e2d L', faulted('left_stalled'), faulted('left_open_circuit')),
      motorChip('\u0e25\u0e49\u0e2d R', faulted('right_stalled'), faulted('right_open_circuit'))
    ];
    var GLYPH = { ok: '\u2713', warn: '\u26a0', bad: '\u2715' };

    // Built once and then only updated in place. Rebuilding the row every frame
    // meant seven elements were destroyed and recreated ten times a second,
    // which is both wasteful and a chance for the layout to settle differently
    // each time.
    var box = $('sensorChips');
    if (box.childNodes.length !== chips.length) {
      box.textContent = '';
      chips.forEach(function () {
        var span = document.createElement('span');
        var name = document.createElement('b');
        var mark = document.createElement('i');
        span.appendChild(name);
        span.appendChild(mark);
        box.appendChild(span);
      });
    }
    chips.forEach(function (c, i) {
      var span = box.childNodes[i];
      if (span.className !== c[1]) span.className = c[1];
      if (span.firstChild.textContent !== c[0]) span.firstChild.textContent = c[0];
      var g = GLYPH[c[1]];
      if (span.lastChild.textContent !== g) span.lastChild.textContent = g;
      if (c[2] && span.title !== c[2]) span.title = c[2];
    });

    text('tHeadingRef', t.heading_ref_name || '—');
    // Agreement between the two absolute sources, and whether one is being
    // refused. "Refused" is not a fault - it is the cross-check working.
    var innov = t.heading_innov_deg;
    if (innov === undefined || innov === null) {
      text('tHeadingAgree', '—');
    } else {
      var gated = t.heading_gated >= 0.5;
      var mag = Math.abs(innov);
      text('tHeadingAgree', fmt(innov, 1) + '°  ' + (gated
        ? '⚠ ไม่เชื่อเข็มทิศตอนนี้ / reference refused, the IMU disagrees - '
          + 'a disturbance, not a turn'
        : (mag < 5 ? '✓ ตรงกัน / the two agree'
                   : 'ต่างกันเล็กน้อย / drifting apart but still believed')));
    }
    text('tMagHeading', fmt(t.mag_heading_deg, 0));
    text('tMagStatus', t.mag_status_name || '—');
    text('tSpeedSource', t.speed_source_name || '—');
    // Show speed beside rpm: rpm is what the back-EMF model computes, but m/s is
    // what anyone actually reasons about when driving. The wheel diameter comes
    // from the BOARD's own parameter (robot.wheel_diameter), never a constant
    // copied into this file - a copy would go stale the day someone fits
    // different wheels, and would then disagree with the robot silently.
    text('tMotorRpmL', wheelRpmAndMps(t.motor_rpm_left));
    text('tMotorRpmR', wheelRpmAndMps(t.motor_rpm_right));

    // A fault name is not an instruction. "open circuit" reads like a broken
    // wire, and on this robot it was a driver latching into protection - which
    // needs a power-down, not a soldering iron.
    text('tAccX', fmt(t.imu_acc_x, 2));
    text('tAccY', fmt(t.imu_acc_y, 2));
    text('tAccZ', fmt(t.imu_acc_z, 2));
    var imuDead = (t.missing_sensors || []).indexOf('imu_missing') >= 0;
    if (imuDead || t.imu_roll_deg === undefined) {
      text('tGravX', '—'); text('tGravY', '—'); text('tGravZ', '—');
    } else {
      var r = t.imu_roll_deg * Math.PI / 180.0;
      var p = t.imu_pitch_deg * Math.PI / 180.0;
      var gx = -9.81 * Math.sin(p);
      var gy = 9.81 * Math.sin(r) * Math.cos(p);
      var gz = 9.81 * Math.cos(r) * Math.cos(p);
      text('tGravX', fmt(gx, 2));
      text('tGravY', fmt(gy, 2));
      text('tGravZ', fmt(gz, 2));
    }

    text('tOdomX', fmt(t.odom_x, 2));
    text('tOdomY', fmt(t.odom_y, 2));

    text('tMotorFaultWhy', (t.motor_fault_advice || []).join(' · '));

    // Say which position source is steering, and how far it may have drifted.
    // Indoors the robot runs on IMU heading, the compass and the motor model;
    // outdoors it runs on GPS. The switch is automatic, so the ONLY way anyone
    // knows which is live is if the page says so.
    var nav = (state.snap || {}).nav || {};
    var pm = $('posMode');
    if (pm) {
      // Never `hidden`: this banner sits on top of the START button, and taking
      // it out of the flow moves that button up the card the moment the robot
      // loses or regains GPS. The `off` class hides it without giving its space
      // back, so nothing below it ever moves.
      // Write to the text span, never to the banner itself: the banner also
      // holds the invisible sizer twin that reserves its height, and setting
      // textContent on the parent would delete it - after which the banner
      // would resize with its own message again and move START, which is the
      // whole thing being prevented here.
      var pmText = $('posModeText') || pm;
      var src = nav.fix_source;
      if (src === 'dead_reckoning') {
        pm.className = 'posmode warn';
        // Capped at the sizer's own 99.9: a drift estimate wider than the twin
        // would out-grow the space the twin reserved. Past about ten metres the
        // exact figure has stopped mattering anyway - it says "do not trust the
        // map", and it says that just as well at 99.9.
        var sigma = t.pos_sigma_m ? Math.min(t.pos_sigma_m, 99.9) : 0;
        pmText.textContent = 'ในอาคาร: ใช้ IMU เข็มทิศ และมอเตอร์ (ไม่ใช้ GPS) — '
          + 'ตำแหน่งจะคลาดเคลื่อนขึ้นเรื่อย ๆ / INDOOR: navigating on the IMU, '
          + 'compass and motor model. No GPS - the position drifts, several '
          + 'metres per minute'
          + (sigma ? '  (±' + sigma.toFixed(1) + ' m)' : '');
      } else if (src === 'gps') {
        pm.className = 'posmode ok';
        pmText.textContent = 'กลางแจ้ง: ใช้ GPS / OUTDOOR: navigating on GPS'
          + (t.gps_satellites ? '  (' + Math.round(t.gps_satellites) + ' sats)' : '');
      } else {
        pm.className = 'posmode off';
      }
    }

    renderCalibration(t);
  }

  /* ------------------------------------------------------------ calibration */
  /**
   * The Calibration tab, driven entirely by telemetry.
   *
   * Both calibrations used to be invisible while they ran. The compass gave a
   * button and no feedback, so the only way to know whether the turn had been
   * enough was to press Finish and see whether it was refused. The IMU gave
   * nothing at all from a browser - its accuracy number existed only in the USB
   * console's `info` output, which meant that from the web a magnetometer that
   * merely needed waving about and one that was broken looked identical:
   * imu_magnetometer_missing, indefinitely.
   */
  function renderCalibration(t) {
    // --- compass ---------------------------------------------------------
    var calibrating = t.mag_calib_state === 1;
    var progress = Math.max(0, Math.min(1, t.mag_calib_progress || 0));
    var pct = Math.round(progress * 100);
    var bar = $('magProgBar');
    if (bar) bar.style.width = pct + '%';
    // Below three quarters the board refuses to save, so say so before the
    // press rather than after it.
    if (bar) bar.className = progress >= 0.75 ? 'ok' : (calibrating ? 'busy' : '');
    text('magProgText', calibrating || pct > 0 ? pct + '%' : '—');
    text('cMagStatus', t.mag_status_name || '—');
    text('cMagCalState', t.mag_calib_name || '—');
    text('cMagHeading', fmt(t.mag_heading_deg, 0));
    // Degrees, not just a bar. The gyro measures the turn independently of the
    // magnetometer, so this is the number that says whether the robot really
    // went round - and a number you can act on beats a proportion you cannot.
    text('cMagTurned', calibrating ? fmt(Math.abs(t.mag_turned_deg || 0), 0) : '—');

    var viewer = state.mode === 'viewer';
    $('btnMagStart').disabled = calibrating || viewer;
    $('btnMagFinish').disabled = !calibrating || viewer;
    $('btnMagClear').disabled = calibrating || viewer;
    // Only offered while a sweep is actually running - there is nothing to stop
    // otherwise, and an always-live button invites pressing it for no reason.
    $('btnMagCancel').disabled = !calibrating || viewer;
    if (calibrating) {
      var turned = Math.abs(t.mag_turned_deg || 0);
      $('magHint').textContent = progress >= 0.75
        ? 'ครบรอบแล้ว กดบันทึกได้เลย / enough of the circle — press Finish'
        : ('หมุนต่ออีก ' + Math.max(0, Math.round(360 - turned)) + '° / keep turning, '
           + Math.max(0, Math.round(360 - turned)) + '° to go');
    } else if (t.mag_calib_state === 2) {
      $('magHint').textContent = 'บันทึกลงบอร์ดแล้ว ใช้ได้แม้ปิดเครื่อง / saved on the board';
    } else if (t.mag_calib_state === 3) {
      $('magHint').textContent = 'หมุนไม่ครบรอบ บอร์ดไม่รับ ลองใหม่ / '
        + 'the sweep did not cover enough of the circle, so nothing was saved';
    } else {
      $('magHint').textContent = 'หมุนหุ่นช้า ๆ ครบ 1 รอบ แล้วกดบันทึก';
    }

    // --- IMU -------------------------------------------------------------
    var acc = t.imu_accuracy;
    var pips = $('imuPips');
    if (pips) {
      Array.prototype.forEach.call(pips.children, function (pip, i) {
        // A pip is lit when the accuracy has reached it. 2 is the line the
        // heading chain draws, so 0 and 1 are warnings, not progress.
        pip.className = (acc >= i) ? (i >= 2 ? 'lit ok' : 'lit warn') : '';
      });
    }
    text('imuAccText', acc === undefined ? '—' : Math.round(acc) + '/3');
    text('cImuAcc', t.imu_accuracy_name || '—');
    text('cImuState', t.imu_calib_name || '—');
    $('btnImuStart').disabled = viewer;
    $('btnImuSave').disabled = viewer || acc < 2;
    $('btnImuClear').disabled = viewer;
    // A refusal says WHY. "Refused" on its own is not a diagnosis: a sensor
    // that is absent, one that is busy and one that rejected the command
    // outright all look the same, and they need different actions.
    if (t.imu_calib_state === 3) {
      $('imuHint').textContent = 'เซ็นเซอร์ไม่รับคำสั่ง / the sensor refused: '
        + (t.imu_calib_result_name || '?');
    } else if (acc >= 2) {
      $('imuHint').textContent =
        'ใช้ได้แล้ว กดบันทึกลงเซ็นเซอร์ / good enough — press Save so it survives a reboot';
    } else if (t.imu_calib_state === 1) {
      $('imuHint').textContent =
        'กำลังเรียน… ขยับเป็นเลขแปดช้า ๆ ห่างจากมอเตอร์ / learning — slow figure-of-eights, away from the motors';
    } else {
      $('imuHint').textContent =
        'ยังไม่พอ ต้องได้ 2 ขึ้นไป / not yet: 2 or better is what the heading chain needs';
    }
    text('cImuResult', t.imu_calib_result_name || '—');

    // --- wheels, motor model, position -----------------------------------
    text('cWheelDia', fmt(paramValue('robot.wheel_diameter'), 3));
    text('cTrack', fmt(paramValue('robot.track'), 3));
    text('cMpsL', fmt(t.wheel_mps_left, 2));
    text('cMpsR', fmt(t.wheel_mps_right, 2));
    text('cMaV', fmt(t.motor_a_v, 2));
    text('cMaA', fmt(t.motor_a_a, 2));
    text('cMbV', fmt(t.motor_b_v, 2));
    text('cMbA', fmt(t.motor_b_a, 2));
    text('cPwmL', fmt(t.pwm_left, 0));
    text('cPwmR', fmt(t.pwm_right, 0));
    text('cModelL', fmt(t.motor_rpm_left, 1));
    text('cModelR', fmt(t.motor_rpm_right, 1));
    text('cWheelL', fmt(t.wheel_rpm_left, 1));
    text('cWheelR', fmt(t.wheel_rpm_right, 1));
    text('cOdomX', fmt(t.odom_x, 2));
    text('cOdomY', fmt(t.odom_y, 2));
    text('cSigma', fmt(t.pos_sigma_m, 2));

    // The dashboard keeps a one-line summary so a bad calibration is visible
    // from the page people actually watch, not only from the one they visit
    // twice a year.
    text('sensorCalibHint', acc >= 2 && t.mag_status === 1
      ? 'เซ็นเซอร์ทิศพร้อม / heading sensors calibrated'
      : 'IMU ' + (acc === undefined ? '—' : Math.round(acc)) + '/3 · '
        + 'เข็มทิศ ' + (t.mag_status_name || '—'));
  }

  /** One parameter's current value, or undefined before the board has answered.
   *  Never a hardcoded fallback: a wheel diameter copied into this file would go
   *  stale the day someone fits different wheels and then disagree with the
   *  robot in silence. A dash is honest; a wrong number is not. */
  function paramValue(key) {
    var list = paramsState && paramsState.entries;
    if (!list) return undefined;
    for (var i = 0; i < list.length; i++) {
      if (list[i].key === key) return list[i].value;
    }
    return undefined;
  }

  function calib(endpoint, action, note) {
    if (blockedInViewer()) return;
    api(endpoint, { action: action });
    if (note) toast(note);
  }

  $('btnMagStart').addEventListener('click', function () {
    calib('/api/mag_calib', 'start',
      'หมุนหุ่นช้า ๆ ครบ 1 รอบ / turn the robot slowly through a full circle');
  });
  $('btnMagFinish').addEventListener('click', function () {
    calib('/api/mag_calib', 'finish');
  });
  $('btnMagCancel').addEventListener('click', function () {
    calib('/api/mag_calib', 'cancel', 'หยุดแล้ว ไม่ได้บันทึก / stopped, nothing saved');
  });
  $('btnImuCancel').addEventListener('click', function () {
    calib('/api/imu_calib', 'cancel', 'หยุดแล้ว / stopped');
  });

  $('btnMagClear').addEventListener('click', function () {
    if (!window.confirm('ล้างค่าเข็มทิศ กลับไปใช้ค่าดิบ?\n\n'
        + 'Clear the compass calibration and go back to raw readings?')) return;
    calib('/api/mag_calib', 'clear', 'ล้างค่าเข็มทิศแล้ว / compass calibration cleared');
  });

  $('btnImuStart').addEventListener('click', function () {
    calib('/api/imu_calib', 'start',
      'ขยับหุ่นเป็นเลขแปดช้า ๆ ห่างจากมอเตอร์ / slow figure-of-eights, away from the motors');
  });
  $('btnImuSave').addEventListener('click', function () {
    calib('/api/imu_calib', 'save', 'บันทึกลงเซ็นเซอร์แล้ว / saved into the sensor');
  });
  // Bringing a dead IMU up again without a reflash. The automatic retry stops
  // after ten attempts, on the reasoning that software cannot reset a sensor
  // whose reset line is not wired - but a person CAN, by unplugging the power,
  // and after that the board should try again on request.
  $('btnImuRetry').addEventListener('click', function () {
    calib('/api/imu_calib', 'retry',
      'กำลังลองเริ่ม IMU ใหม่ / trying to start the IMU again');
  });

  $('btnImuClear').addEventListener('click', function () {
    if (!window.confirm('ล้างค่า IMU และรีสตาร์ทเซ็นเซอร์?\n\n'
        + 'Clear the IMU calibration and restart the sensor?')) return;
    calib('/api/imu_calib', 'clear', 'ล้างค่า IMU แล้ว / IMU calibration cleared');
  });

  $('btnCalibResetOdom').addEventListener('click', function () {
    if (blockedInViewer()) return;
    api('/api/reset_odom', {});
    toast('ตั้งศูนย์แล้ว / position zeroed');
  });

  /* Cross-tab links: "edit these" jumps to the right page already filtered,
     instead of telling people to go and find the field themselves. */
  Array.prototype.forEach.call(document.querySelectorAll('[data-goto-tab]'), function (b) {
    b.addEventListener('click', function () { showTab(b.dataset.gotoTab); });
  });
  Array.prototype.forEach.call(document.querySelectorAll('[data-goto-params]'), function (b) {
    b.addEventListener('click', function () {
      showTab('board');
      var filter = $('paramsFilter');
      if (filter) {
        filter.value = b.dataset.gotoParams;
        filter.dispatchEvent(new Event('input'));
      }
    });
  });

  function setPill(id, cls, value, detail) {
    var pill = $(id);
    pill.className = 'pill ' + cls;
    pill.lastElementChild.textContent = value;
    // The detail is the evidence behind the word - the exact age, the HDOP.
    // Kept on the element rather than in it, so the row stays readable at a
    // glance and the number is still one hover away.
    if (detail !== undefined) pill.title = detail;
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
      state.jog.timer = setInterval(sendJog, jogRepeatMs);
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

  /* --------------------------------------------------------------- theme ---
   * Three states: follow the device, or force light or dark.
   *
   * Kept in localStorage rather than on the robot, because it belongs to the
   * screen looking at the robot and not to the robot itself - a phone in
   * sunlight and a laptop in a dark workshop want opposite answers from the
   * same machine at the same moment.
   */
  function applyTheme(choice) {
    var root = document.documentElement;
    if (choice === 'light' || choice === 'dark') root.setAttribute('data-theme', choice);
    else root.removeAttribute('data-theme');      // auto: the media query decides
    Array.prototype.forEach.call(document.querySelectorAll('[data-theme-set]'),
      function (b) {
        b.className = 'chipbtn' + (b.dataset.themeSet === choice ? ' active' : '');
      });
    var note = $('themeNote');
    if (note) {
      note.textContent = choice === 'auto'
        ? 'ตามเครื่อง / following this device'
        : 'บังคับไว้ / forced, ignoring the device setting';
    }
    try { localStorage.setItem('gpsloc.theme', choice); } catch (e) { /* private mode */ }
  }

  (function setupTheme() {
    var saved = 'auto';
    try { saved = localStorage.getItem('gpsloc.theme') || 'auto'; } catch (e) { saved = 'auto'; }
    applyTheme(saved);
    Array.prototype.forEach.call(document.querySelectorAll('[data-theme-set]'),
      function (b) {
        b.addEventListener('click', function () { applyTheme(b.dataset.themeSet); });
      });
  })();

  /* ------------------------------------------------------------ joystick ---
   * One gesture for both axes. The arrow pad sends forward OR turn, so an arc
   * had to be driven by alternating two buttons, which gives a zig-zag; the
   * firmware has always accepted both in the same command.
   *
   * Up is forward and right is clockwise, matching cmd_manual's linear.x and
   * angular.z, so what the operator does and what goes on the wire are the same
   * shape. Magnitude is proportional to how far the knob is pushed, capped at
   * the edge of the ring, and the value is scaled by the SAME speed limit the
   * arrow buttons use - a joystick that ignored the speed setting would be a
   * second, hidden throttle.
   */
  (function setupJoystick() {
    var pad = $('joyPad');
    var knob = $('joyKnob');
    if (!pad || !knob) return;

    var active = false;
    var pointerId = null;
    var timer = null;
    var cur = { forward: 0, turn: 0 };

    function show(forward, turn) {
      var speed = state.jog.speedMps || 0;
      var rate = state.jog.turnDps || 0;
      text('joyFwd', (forward * speed).toFixed(2));
      text('joyTurn', Math.round(turn * rate));
    }

    function send() {
      if (blockedInViewer()) return;
      api('/api/manual', { forward: cur.forward, turn: cur.turn });
    }

    function moveTo(event) {
      var box = pad.getBoundingClientRect();
      var half = Math.min(box.width, box.height) / 2;
      // The knob's travel stops short of the rim so it stays inside the pad.
      var reach = half - 27;
      var dx = event.clientX - (box.left + box.width / 2);
      var dy = event.clientY - (box.top + box.height / 2);
      var dist = Math.sqrt(dx * dx + dy * dy);
      if (dist > reach && dist > 0) { dx *= reach / dist; dy *= reach / dist; }
      knob.style.transform = 'translate(' + dx.toFixed(1) + 'px,' + dy.toFixed(1) + 'px)';
      // Screen y grows downward; forward is up, hence the negation.
      cur.forward = Math.max(-1, Math.min(1, -dy / reach));
      cur.turn = Math.max(-1, Math.min(1, dx / reach));
      show(cur.forward, cur.turn);
    }

    function release() {
      if (!active) return;
      active = false;
      pointerId = null;
      pad.classList.remove('active');
      knob.style.transform = 'translate(0,0)';
      cur.forward = 0; cur.turn = 0;
      show(0, 0);
      if (timer) { clearInterval(timer); timer = null; }
      // Stop is unconditional and never merely "send zero": the firmware's own
      // command watchdog would stop it anyway after 500 ms, but half a second
      // of unwanted motion is exactly what a release is meant to prevent.
      api('/api/manual', { action: 'stop' });
    }

    pad.addEventListener('pointerdown', function (event) {
      if (blockedInViewer()) return;
      event.preventDefault();
      active = true;
      pointerId = event.pointerId;
      pad.classList.add('active');
      pad.setPointerCapture(event.pointerId);
      moveTo(event);
      send();
      // Repeat while held: the firmware stops after 500 ms with no command, so
      // a joystick held still must keep saying so.
      if (timer) clearInterval(timer);
      timer = setInterval(send, 80);
    });

    pad.addEventListener('pointermove', function (event) {
      if (!active || event.pointerId !== pointerId) return;
      event.preventDefault();
      moveTo(event);
    });

    ['pointerup', 'pointercancel', 'pointerleave'].forEach(function (name) {
      pad.addEventListener(name, function (event) {
        if (event.pointerId !== pointerId) return;
        release();
      });
    });
    // A pointer lost outside the element, or the tab going away, must stop it.
    window.addEventListener('blur', release);
  })();

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
               ArrowUp: [1, 0], ArrowDown: [-1, 0], ArrowLeft: [0, -1], ArrowRight: [0, 1],
               Up: [1, 0], Down: [-1, 0], Left: [0, -1], Right: [0, 1],
               KeyW: [1, 0], KeyS: [-1, 0], KeyA: [0, -1], KeyD: [0, 1] };
  /* A SET of held keys, not one.
   *
   * This used to remember a single key, so pressing W and then A replaced the
   * forward command with a turn instead of adding to it, and letting go of
   * either one stopped the robot outright. Driving a curve from the keyboard
   * was impossible - which was a limit of this handler, not of the robot: the
   * firmware has always taken speed and yaw rate in the same command.
   *
   * Summed and clamped, so W+A is forward-and-left and W+A+D is just forward.
   */
  var keysDown = Object.create(null);

  function applyKeys() {
    var forward = 0, turn = 0;
    for (var k in keysDown) {
      var v = KEYS[k];
      if (v) { forward += v[0]; turn += v[1]; }
    }
    forward = Math.max(-1, Math.min(1, forward));
    turn = Math.max(-1, Math.min(1, turn));
    if (forward === 0 && turn === 0) stopJog();
    else startJog(forward, turn);
  }

  window.addEventListener('keydown', function (event) {
    if (event.target.tagName === 'INPUT' || event.target.tagName === 'SELECT') return;
    if (event.key === ' ' || event.key === 'Spacebar' || event.code === 'Space') {   // space = stop everything now
      keysDown = Object.create(null);
      stopJog();
      api('/api/nav', { action: 'pause', reason: 'spacebar' });
      toast('หยุดแล้ว (Space) / stopped', false);
      event.preventDefault();
      return;
    }
    
    var k = KEYS[event.code] ? event.code : (KEYS[event.key] ? event.key : null);
    if (!k || keysDown[k]) return;
    keysDown[k] = true;
    applyKeys();
    event.preventDefault();
  });
  window.addEventListener('keyup', function (event) {
    var k = keysDown[event.code] ? event.code : (keysDown[event.key] ? event.key : null);
    if (!k) return;
    delete keysDown[k];
    applyKeys();
  });
  // Losing the window must stop the robot: keyup never arrives for a key that
  // was held when focus went away, so it would otherwise drive on for ever.
  window.addEventListener('blur', function () {
    keysDown = Object.create(null);
    stopJog();
  });

  /* ------------------------------------------------------------------ map */
  var canvas = $('map');
  var ctx = canvas.getContext('2d');

  $('mapFit').addEventListener('click', function () { state.fitRequested = true; drawMap(); });
  if ($('mapWpToggle')) {
    $('mapWpToggle').addEventListener('click', function() {
      var content = $('mapWpContent');
      var chevron = $('mapWpChevron');
      if (content.style.display === 'none') {
        content.style.display = 'block';
        chevron.textContent = '▼';
      } else {
        content.style.display = 'none';
        chevron.textContent = '◀';
      }
    });
  }

  if ($('mapCentre')) {
    $('mapCentre').addEventListener('click', function () {
      // Centre on the robot at the CURRENT zoom. Different from Fit, which
      // re-frames everything and zooms out: while driving you want to keep the
      // scale you chose and simply stop hunting for the robot.
      var snap = state.snap || {};
      var fix = snap.fix || {}, pose = snap.pose || {};
      var lat = (fix.lat === undefined || fix.lat === null) ? pose.lat : fix.lat;
      var lon = (fix.lon === undefined || fix.lon === null) ? pose.lon : fix.lon;
      if (!state.datum || lat === undefined || lat === null) return;
      var q = enu(lat, lon, state.datum);
      view.cx = q.x; view.cy = q.y;
      state.fitRequested = false;
      drawMap();
    });
  }
  /* Two free basemaps, neither needing an API key or an account.
   *
   * Street is the default rather than satellite: a plain grid tells you nothing
   * about where the robot is, and roads and buildings are what most people
   * actually navigate by. Satellite is one press away when you want to see the
   * ground itself.
   *
   * Both require internet on the machine running the BROWSER, not on the robot.
   * With none, tiles simply never arrive and the grid shows through - the map
   * keeps working, it just has no picture behind it. */
  var BASEMAPS = {
    // maxZoom is what each source actually publishes. Ask past it and the tile
    // server returns nothing, so the map goes blank at exactly the zoom where
    // it was becoming useful - which reads as a broken satellite view rather
    // than a zoom level that does not exist. Below the cap the last real tiles
    // are drawn larger: blurry, but still a street and still a roof.
    street: {
      url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
      label: '🗺 แผนที่ถนน / Street',
      attrib: '© OpenStreetMap contributors',
      maxZoom: 19
    },
    satellite: {
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/' +
           'World_Imagery/MapServer/tile/{z}/{y}/{x}',
      label: '🛰 ภาพดาวเทียม / Satellite',
      attrib: 'ภาพ / imagery © Esri, Maxar, Earthstar Geographics',
      maxZoom: 19
    },
    none: { url: null, label: '▦ ตารางอย่างเดียว / Grid only', attrib: '' }
  };
  var BASEMAP_ORDER = ['street', 'satellite', 'none'];

  /* Where to look when the robot has no fix yet.
   *
   * Without this the map was a blank grid until the first satellite lock, which
   * is most of the time indoors and all of the time on a bench. A blank grid
   * cannot be panned to anywhere useful either, because with no datum there is
   * no latitude to turn screen metres into - so the imagery never loaded and
   * the basemap buttons appeared broken.
   *
   * Bangkok, because that is where this robot is. It is a VIEW, never a
   * position: no robot marker is drawn against it, and the banner still says
   * plainly that there is no fix. The moment a real fix arrives the datum
   * becomes that fix and this is not used again.
   */
  var FALLBACK_DATUM = { lat: 13.7563, lon: 100.5018, fallback: true };

  function applyBasemap() {
    var def = BASEMAPS[state.basemap] || BASEMAPS.street;
    var b = $('mapSat');
    if (b) {
      b.textContent = def.label;
      b.className = (state.basemap === 'none') ? 'btn' : 'btn go';
    }
    text('mapAttrib', def.attrib || '');
    try { window.localStorage.setItem('gpsLocalizeBasemap', state.basemap); } catch (e) { /* private */ }
  }
  if ($('mapSat')) {
    $('mapSat').addEventListener('click', function () {
      // Cycle rather than toggle: three states do not fit one on/off button,
      // and the label always names what you are about to look at.
      var i = BASEMAP_ORDER.indexOf(state.basemap);
      state.basemap = BASEMAP_ORDER[(i + 1) % BASEMAP_ORDER.length];
      applyBasemap();
      drawMap();
    });
    try {
      var saved = window.localStorage.getItem('gpsLocalizeBasemap');
      if (saved && BASEMAPS[saved]) state.basemap = saved;
    } catch (e) { /* private mode */ }
    applyBasemap();
  }
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

  /* ------------------------------------------------- satellite imagery layer */
  /* Esri World Imagery: free, no API key, no account. The tiles are Web
   * Mercator, while this map is a flat local-metre (ENU) grid centred on the
   * datum - so each tile is projected through latLon() rather than blitted, and
   * over the tens of metres a robot works in the two agree closely enough that
   * the error is far smaller than a GPS fix.
   *
   * Needs internet on the machine running the browser. When there is none the
   * tiles simply never load and the plain grid shows through, which is the
   * right failure: the map still works, it just has no picture behind it.
   *
   * Esri's terms require the attribution drawn in drawMap(). */
  var tileCache = {};      // "layer/z/x/y" -> Image


  function lonToTileX(lon, z) { return (lon + 180) / 360 * Math.pow(2, z); }
  function latToTileY(lat, z) {
    var r = lat * Math.PI / 180;
    return (1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2 * Math.pow(2, z);
  }
  function tileXToLon(x, z) { return x / Math.pow(2, z) * 360 - 180; }
  function tileYToLat(y, z) {
    var n = Math.PI - 2 * Math.PI * y / Math.pow(2, z);
    return 180 / Math.PI * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
  }

  function tile(layer, z, x, y) {
    var key = layer + '/' + z + '/' + x + '/' + y;
    if (tileCache[key]) return tileCache[key];
    var def = BASEMAPS[layer];
    if (!def || !def.url) return { ready: false };
    var img = new Image();
    // No crossOrigin: the tiles are only ever drawn, never read back with
    // getImageData, so requesting CORS buys nothing and would make the load
    // fail outright against any host that did not grant it.
    img.onload = function () { img.ready = true; drawMap(); };
    img.onerror = function () { img.failed = true; };
    img.src = def.url.replace('{z}', z).replace('{x}', x).replace('{y}', y);
    tileCache[key] = img;
    return img;
  }

  /** Zoom whose tiles are closest to the current pixels-per-metre. */
  function tileZoomFor(datum) {
    // Web Mercator ground resolution at this latitude, metres per pixel.
    // Stop at what the providers publish. Asking for zoom 21 from a source that
    // ends at 19 returns nothing at all, and the map goes blank exactly when it
    // is zoomed in far enough to be useful - which reads as "the satellite view
    // is broken" rather than "that zoom does not exist".
    var top = (BASEMAPS[state.basemap] && BASEMAPS[state.basemap].maxZoom) || 19;
    for (var z = top; z >= 1; z--) {
      var mpp = 156543.03392 * Math.cos(datum.lat * Math.PI / 180) / Math.pow(2, z);
      if (mpp * view.scale >= 0.75) return z;   // one tile pixel >= ~1 screen pixel
    }
    return 1;
  }

  /* A compass rose, always in the same corner of the map.
   *
   * The robot triangle already points where the robot points, but on a wide
   * view it is a few pixels across and over aerial imagery it disappears - so
   * "which way is it facing" became a question the map could not answer. This
   * shows north and the robot's heading side by side, at a fixed size that does
   * not shrink with zoom, and states the number as well as drawing it. */
  function drawCompassRose(tel) {
    var headingDeg = tel.heading_deg;
    var ref = tel.heading_ref_name;
    var r = 34, cx = canvas.width - r - 20, cy = r + 20;

    ctx.save();
    // Its own dark disc, so it is legible over pale street tiles and dark
    // aerial imagery alike. Everything on top of it is white or a bright
    // accent, never a theme colour - theme colours are chosen to sit on the
    // page background, and over imagery half of them vanish.
    ctx.beginPath();
    ctx.arc(cx, cy, r + 7, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0,0,0,.62)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,.45)';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Cardinals. The map is north-up, so these never move.
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    var marks = [['N', 0], ['E', 90], ['S', 180], ['W', 270]];
    for (var i = 0; i < marks.length; i++) {
      var a = marks[i][1] * Math.PI / 180;
      mapLabel(marks[i][0],
               cx + Math.sin(a) * (r - 6), cy - Math.cos(a) * (r - 6),
               marks[i][0] === 'N' ? '#ff6b6b' : 'rgba(255,255,255,.85)',
               'bold 11px system-ui, sans-serif', 'center');
    }

    // The RAW magnetometer, hollow, only when a healthy compass is fitted.
    // When the two needles separate the magnetometer is being pulled by
    // something, and that is the moment you want to see it - not as a number
    // on another tab.
    if (tel.mag_status === 1 && tel.mag_heading_deg !== undefined &&
        tel.mag_heading_deg !== null) {
      needle(ctx, cx, cy, r - 12, tel.mag_heading_deg, '#ffb020', false);
    }
    // The fused heading, filled: what the robot actually steers by.
    if (headingDeg !== null && headingDeg !== undefined && !isNaN(headingDeg)) {
      needle(ctx, cx, cy, r - 7, headingDeg, '#4da3ff', true);
    }
    ctx.beginPath();
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    // Caption below the disc: the number, then where it came from. Naming the
    // SOURCE matters - a heading from the gyro alone drifts, and it should not
    // look as trustworthy as one from the compass or the GPS course.
    var ty = cy + r + 20;
    if (headingDeg !== null && headingDeg !== undefined && !isNaN(headingDeg)) {
      mapLabel(Math.round(((headingDeg % 360) + 360) % 360) + '\u00B0', cx, ty,
               '#ffffff', 'bold 14px system-ui, sans-serif', 'center');
      ty += 14;
    }
    if (ref) {
      mapLabel(ref, cx, ty, 'rgba(255,255,255,.85)',
               '10px system-ui, sans-serif', 'center');
      ty += 13;
    }
    if (tel.mag_status === 2) {
      mapLabel('\u0E41\u0E21\u0E48\u0E40\u0E2B\u0E25\u0E47\u0E01\u0E23\u0E1A\u0E01\u0E27\u0E19 / disturbed',
               cx, ty, '#ffb020', 'bold 10px system-ui, sans-serif', 'center');
      ty += 13;
    }
    // Position uncertainty. A map dot that looks equally confident while
    // coasting blind is a lie to the driver.
    if (tel.pos_sigma_m !== undefined && tel.pos_sigma_m !== null && tel.pos_anchored) {
      mapLabel('\u00B1' + tel.pos_sigma_m.toFixed(1) + ' m', cx, ty,
               tel.pos_sigma_m > 3 ? '#ffb020' : 'rgba(255,255,255,.8)',
               '10px system-ui, sans-serif', 'center');
    }

    ctx.textAlign = 'left';
    ctx.textBaseline = 'alphabetic';
    ctx.restore();
  }

  /* Text that stays readable on ANY background.
   *
   * The map draws over three completely different grounds - a dark grid, pale
   * street tiles and dark aerial imagery - and a single fill colour cannot work
   * on all of them. Every label on the map is stroked with a dark casing first
   * and filled after, which is what paper maps have always done.
   */
  function mapLabel(text, x, y, colour, font, align) {
    ctx.save();
    ctx.font = font || 'bold 12px system-ui, sans-serif';
    ctx.textAlign = align || 'left';
    ctx.lineJoin = 'round';
    ctx.lineWidth = 3.5;
    ctx.strokeStyle = 'rgba(0,0,0,.75)';
    ctx.strokeText(text, x, y);
    ctx.fillStyle = colour || '#ffffff';
    ctx.fillText(text, x, y);
    ctx.restore();
  }

  /** A rounded plate to sit controls and readouts on, over any imagery. */
  function mapPlate(x, y, w, h, r) {
    r = r === undefined ? 6 : r;
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
    ctx.fillStyle = 'rgba(0,0,0,.6)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,.25)';
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  function drawBasemap(datum) {
    var layer = state.basemap || 'street';
    if (layer === 'none' || !BASEMAPS[layer] || !datum) return;
    var z = tileZoomFor(datum);
    // Corners of the visible area, in local metres, then in tile coordinates.
    var w = canvas.width / view.scale, h = canvas.height / view.scale;
    var nw = latLon(view.cx - w / 2, view.cy + h / 2, datum);
    var se = latLon(view.cx + w / 2, view.cy - h / 2, datum);
    var x0 = Math.floor(lonToTileX(nw.lon, z)), x1 = Math.floor(lonToTileX(se.lon, z));
    var y0 = Math.floor(latToTileY(nw.lat, z)), y1 = Math.floor(latToTileY(se.lat, z));
    var span = Math.pow(2, z);
    // A runaway loop here would hang the page, so cap what one frame may draw.
    if ((x1 - x0 + 1) * (y1 - y0 + 1) > 64) return;

    for (var tx = x0; tx <= x1; tx++) {
      for (var ty = y0; ty <= y1; ty++) {
        if (tx < 0 || ty < 0 || tx >= span || ty >= span) continue;
        var img = tile(layer, z, tx, ty);
        if (!img.ready) continue;
        // Project the tile's own corners, so it lands where it belongs on the
        // ENU grid rather than being assumed square in screen space.
        var tl = toScreen(enu(tileYToLat(ty, z), tileXToLon(tx, z), datum));
        var br = toScreen(enu(tileYToLat(ty + 1, z), tileXToLon(tx + 1, z), datum));
        var dw = br.x - tl.x, dh = br.y - tl.y;
        if (dw <= 0 || dh <= 0) continue;
        try { ctx.drawImage(img, tl.x, tl.y, dw, dh); } catch (e) { /* not decoded yet */ }
      }
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

    // The datum the PICTURE is drawn against. A real fix when there is one,
    // Bangkok otherwise, so the map is something to look at rather than a blank
    // grid - see FALLBACK_DATUM. Positions are still only drawn from a real fix.
    var datum = state.datum || FALLBACK_DATUM;

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

    // The banner stays even though there is now a picture behind it. Imagery is
    // not a fix, and a map that looks populated while the robot has no idea
    // where it is would be the most misleading thing on the page.
    $('mapEmpty').className = state.datum ? 'empty hidden' : 'empty';
    if (!state.datum) {
      // Name the actual reason. "No satellites" and "the robot is offline" need
      // completely different responses, and an empty map alone cannot tell them
      // apart - which is exactly how a working robot gets reported as dead.
      var sats = (snap.telemetry || {}).gps_satellites;
      var linked = (snap.telemetry_age_s !== undefined && snap.telemetry_age_s !== null
                    && snap.telemetry_age_s < 5);
      text('mapEmptyWhy', !linked
        ? '— ยังไม่ได้รับข้อมูลจากหุ่น / no telemetry from the robot yet. '
        : ('— หุ่นเชื่อมต่ออยู่ แต่ GPS ยังจับดาวเทียมไม่ได้ (' + (Math.round(sats || 0))
           + ' ดวง) / robot IS connected, GPS has ' + (Math.round(sats || 0))
           + ' satellites and no fix. '));
      text('mapEmptyWhere', 'กำลังแสดงกรุงเทพฯ ไว้ก่อน — ยังไม่ใช่ตำแหน่งหุ่น / '
           + 'showing Bangkok until there is a fix - this is NOT the robot\u2019s position');
    }

    // Position readout: the raw numbers, so a fix can be checked against a phone
    // or typed into a map, without reading them off the canvas.
    var sats = (snap.telemetry || {}).gps_satellites;
    text('mapLatLon', (fix.lat !== undefined && fix.lat !== null)
      ? (fix.lat.toFixed(6) + ', ' + fix.lon.toFixed(6)) : '—');
    text('mapAlt', (fix.alt !== undefined && fix.alt !== null)
      ? (fix.alt.toFixed(1) + ' m') : '—');
    text('mapSats', (sats === undefined || sats === null) ? '—' : String(Math.round(sats)));

    // Imagery first, so everything else is drawn on top of the picture.
    drawBasemap(datum);

    // grid, 5 m spacing (or coarser when zoomed out)
    /* Grid and scale-bar step, on a 1-2-5 ladder starting at ten centimetres.
     *
     * It used to start at 5 m and double: 5, 10, 20, 40, 80, 160, 320. Zoomed
     * in, the finest thing it could draw was 5 m and in practice it sat at 20,
     * so a compact space - a courtyard, a dormitory car park - had no usable
     * scale at all; everything worth measuring was smaller than one grid
     * square. Doubling also produced 40, 80 and 160, which nobody thinks in.
     *
     * 1-2-5 gives 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100 ... - the steps a
     * tape measure has, and it reaches the small end where the testing is. */
    var LADDER = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500,
                  1000, 2000, 5000, 10000];
    var step = LADDER[LADDER.length - 1];
    for (var li = 0; li < LADDER.length; li++) {
      if (LADDER[li] * view.scale >= 40) { step = LADDER[li]; break; }
    }
    // Build the grid path once, then paint it twice.
    if ((state.basemap || 'street') === 'none') {
      ctx.save();
      var half = Math.ceil(Math.max(canvas.width, canvas.height) / view.scale / 2) + step;
      ctx.beginPath();
      for (var g = -half; g <= half; g += step) {
        var a = toScreen({ x: view.cx + g, y: view.cy - half });
        var b = toScreen({ x: view.cx + g, y: view.cy + half });
        ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
        var c = toScreen({ x: view.cx - half, y: view.cy + g });
        var d = toScreen({ x: view.cx + half, y: view.cy + g });
        ctx.moveTo(c.x, c.y); ctx.lineTo(d.x, d.y);
      }
      ctx.strokeStyle = line;
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.restore();
    }

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

    // path between waypoints
    if (wpPoints.length > 1) {
      ctx.save();
      ctx.strokeStyle = muted;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);

      for (var i = 0; i < wpPoints.length - 1; i++) {
        var s1 = toScreen(wpPoints[i]);
        var s2 = toScreen(wpPoints[i+1]);
        
        ctx.beginPath();
        ctx.moveTo(s1.x, s1.y);
        ctx.lineTo(s2.x, s2.y);
        ctx.stroke();

        var d = Math.sqrt((wpPoints[i+1].x - wpPoints[i].x) * (wpPoints[i+1].x - wpPoints[i].x) + (wpPoints[i+1].y - wpPoints[i].y) * (wpPoints[i+1].y - wpPoints[i].y));
        mapLabel(d.toFixed(1) + ' m', (s1.x + s2.x) / 2, (s1.y + s2.y) / 2, '#9aa6b8', '11px system-ui, sans-serif', 'center');
      }

      var loopCheck = $('wpLoopToggle');
      if (loopCheck && loopCheck.checked && wpPoints.length > 2) {
        var first = toScreen(wpPoints[0]);
        var last = toScreen(wpPoints[wpPoints.length - 1]);
        ctx.beginPath();
        ctx.moveTo(last.x, last.y);
        ctx.lineTo(first.x, first.y);
        ctx.stroke();
        var dLoop = Math.sqrt((wpPoints[0].x - wpPoints[wpPoints.length - 1].x) * (wpPoints[0].x - wpPoints[wpPoints.length - 1].x) + (wpPoints[0].y - wpPoints[wpPoints.length - 1].y) * (wpPoints[0].y - wpPoints[wpPoints.length - 1].y));
        mapLabel(dLoop.toFixed(1) + ' m', (first.x + last.x) / 2, (first.y + last.y) / 2, '#9aa6b8', '11px system-ui, sans-serif', 'center');
      }
      ctx.restore();
    }

    // waypoints
    wpPoints.forEach(function (p, i) {
      var s = toScreen(p);
      var isTarget = nav.index === i && nav.state === 'RUNNING';
      ctx.fillStyle = isTarget ? '#ffb020' : accent;
      ctx.beginPath();
      ctx.arc(s.x, s.y, isTarget ? 9 : 6, 0, Math.PI * 2);
      ctx.fill();

      // Draw heading arrow if dragging this WP, or if it has a saved heading
      var heading = null;
      if (state.wpDrag && state.wpDrag.index === i) {
        var dx = state.wpDrag.to.x - enu(state.wpDrag.from.lat, state.wpDrag.from.lon, state.datum).x;
        var dy = state.wpDrag.to.y - enu(state.wpDrag.from.lat, state.wpDrag.from.lon, state.datum).y;
        if (Math.hypot(dx, dy) > 0.3) heading = (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
      } else if (state.waypoints[i] && state.waypoints[i].heading_deg !== undefined && state.waypoints[i].heading_deg !== null) {
        heading = state.waypoints[i].heading_deg;
      }
      
      if (heading !== null) {
        var rad = heading * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(s.x, s.y);
        ctx.lineTo(s.x + Math.sin(rad) * 20, s.y - Math.cos(rad) * 20);
        ctx.strokeStyle = ctx.fillStyle;
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // Cased, so a waypoint name is readable over aerial imagery too.
      mapLabel((i + 1) + '. ' + (p.name || ''), s.x + 11, s.y - 7, '#ffffff',
               'bold 12px system-ui, sans-serif', 'left');
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
      ctx.moveTo(0, -16); ctx.lineTo(10, 11); ctx.lineTo(0, 6); ctx.lineTo(-10, 11);
      ctx.closePath();
      // Dark outline first, so the marker reads against grass, tarmac or a pale
      // roof alike. A green triangle on aerial imagery of a green field is
      // invisible, and this is the one thing on the map you must never lose.
      ctx.strokeStyle = 'rgba(0,0,0,.85)';
      ctx.lineWidth = 4;
      ctx.stroke();
      if (reckoned) {                      // hollow orange = estimated, not measured
        ctx.strokeStyle = warnColor;
        ctx.lineWidth = 2;
        ctx.stroke();
      } else {
        ctx.fillStyle = okColor;
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }
      ctx.restore();
      if (reckoned) {
        ctx.fillStyle = warnColor;
        ctx.font = '11px system-ui, sans-serif';
        ctx.fillText('ประมาณจากล้อ/IMU (ไม่มี GPS)', robot.x + 14, robot.y + 4);
      }

      // Line to the next waypoint, drawn whether or not a mission is running.
      //
      // It used to appear only while RUNNING, which is backwards: the moment you
      // most want to see where the robot is heading is BEFORE you press start,
      // to check you picked the right point. Solid while driving there, faint
      // while merely planned, and labelled with the distance so the map answers
      // "how far?" without measuring it by eye.
      var targetObj = null;
      var targetScreen = null;
      var running = (nav.state === 'RUNNING' || nav.state === 'DISCOVER' || nav.state === 'BLOCKED');
      var isTempTarget = false;

      if (running && nav.target && nav.target.lat !== undefined) {
        targetObj = enu(nav.target.lat, nav.target.lon, state.datum);
        targetScreen = toScreen(targetObj);
        isTempTarget = (nav.index === undefined || nav.index < 0);
      } else if (!running && state.waypoints.length > 0) {
        targetObj = wpPoints[0];
        targetScreen = toScreen(targetObj);
      }

      if (targetScreen) {
        var runningDash = (nav.state === 'RUNNING');
        ctx.save();
        ctx.strokeStyle = warnColor;
        ctx.lineWidth = runningDash ? 2 : 1.5;
        if (!runningDash) ctx.globalAlpha = 0.55;
        ctx.setLineDash(runningDash ? [6, 6] : [3, 7]);
        ctx.beginPath();
        ctx.moveTo(robot.x, robot.y);
        ctx.lineTo(targetScreen.x, targetScreen.y);
        ctx.stroke();
        ctx.setLineDash([]);
        
        // If it's a temp target, draw a dot so it's visible on the map
        if (isTempTarget) {
            ctx.fillStyle = warnColor;
            ctx.beginPath();
            ctx.arc(targetScreen.x, targetScreen.y, 9, 0, Math.PI * 2);
            ctx.fill();
            mapLabel('จุดที่กด / Clicked', targetScreen.x + 11, targetScreen.y - 7, '#ffffff',
                     'bold 12px system-ui, sans-serif', 'left');
        }

        var rp = enu(fixForDraw.lat, fixForDraw.lon, state.datum);
        var d = Math.sqrt((targetObj.x - rp.x) * (targetObj.x - rp.x) + (targetObj.y - rp.y) * (targetObj.y - rp.y));
        ctx.globalAlpha = 1;
        mapLabel(d.toFixed(1) + ' m' + (runningDash ? '' : ' (ยังไม่เริ่ม / not started)'),
                 (robot.x + targetScreen.x) / 2 + 8, (robot.y + targetScreen.y) / 2 - 6,
                 '#ffd479', 'bold 12px system-ui, sans-serif', 'left');
        ctx.restore();
      }
    }

    // Compass rose LAST, so it sits over everything - imagery, grid and trail -
    // and is never hidden by whatever the map happens to be showing.
    var tel = snap.telemetry || {};
    drawCompassRose(tel);

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

    // new waypoint arrow being dragged
    if (typeof newWpDrag !== 'undefined' && newWpDrag && state.datum) {
      var from = toScreen(enu(newWpDrag.from.lat, newWpDrag.from.lon, state.datum));
      var to = toScreen(newWpDrag.to);
      ctx.strokeStyle = '#35c46a';
      ctx.fillStyle = '#35c46a';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(from.x, from.y, 6, 0, Math.PI * 2);
      ctx.fill();
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

    // Scale bar, on its own plate with end ticks.
    //
    // It used to be a thin --mut line and --mut text straight onto the map,
    // which over imagery was invisible - so the one control that tells you how
    // big anything is could not be read in satellite view. The plate makes it
    // work on any ground, and the ticks make the length readable rather than
    // just suggested.
    var barLen = Math.max(30, step * view.scale);
    var bx = 14, by = canvas.height - 30, bh = 24;
    mapPlate(bx - 6, by - 4, barLen + 70, bh, 6);
    var y0 = by + 14;
    ctx.save();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(bx, y0); ctx.lineTo(bx + barLen, y0);
    ctx.moveTo(bx, y0 - 5); ctx.lineTo(bx, y0 + 5);
    ctx.moveTo(bx + barLen, y0 - 5); ctx.lineTo(bx + barLen, y0 + 5);
    ctx.stroke();
    ctx.restore();
    // Kilometres once the step passes a thousand metres, and no trailing zeros
    // on the fractional steps - "0.5 m" rather than "0.5000000000000001 m",
    // which is what a float ladder prints if it is handed straight to a string.
    var stepText = step >= 1000 ? (step / 1000) + ' km'
                 : (Math.round(step * 100) / 100) + ' m';
    mapLabel(stepText, bx + barLen + 8, y0 + 4, '#ffffff',
             'bold 12px system-ui, sans-serif', 'left');
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
    canvas.style.cursor = state.poseTool.armed ? 'cell' : 'grab';
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

  /* --------------------------------------- robot parameters (T69) ---------
   * The settings page is BUILT from whatever the board declares, so adding a
   * tunable is one line in the firmware table and nothing here.
   *
   * Two things it must be honest about, because both are invisible otherwise:
   * a value the board has not reported reads "—" rather than showing the
   * default as though it were live, and a STOPPED parameter is disabled while
   * the robot is moving with the reason attached rather than failing on save. */
  var paramsState = { moving: false, entries: [], group: null };

  function paramRow(p) {
    var row = document.createElement('div');
    row.className = 'cfg-row';

    var label = document.createElement('label');
    label.textContent = p.key + (p.unit ? ' (' + p.unit + ')' : '');
    label.title = p.help || '';
    label.htmlFor = 'param_' + p.index;

    var input = document.createElement('input');
    input.type = 'number';
    input.step = 'any';
    input.min = p.min;
    input.max = p.max;
    input.id = 'param_' + p.index;
    input.value = (p.value === null || p.value === undefined) ? '' : p.value;
    input.placeholder = (p.value === null || p.value === undefined) ? '—' : '';
    // What the BOARD said, kept beside what the field now shows, so "write all"
    // can send only what someone actually changed rather than all eighty-five.
    input.dataset.key = p.key;
    if (p.value !== null && p.value !== undefined) {
      input.dataset.boardValue = p.value;
    }

    var blocked = (p.when === 'stopped' && paramsState.moving);
    input.disabled = blocked;

    var note = document.createElement('span');
    note.className = 'mut';
    note.textContent = '[' + p.min + ' … ' + p.max + ']' +
      (p.when === 'stopped' ? '  · stopped only' : '') +
      (blocked ? '  · หยุดหุ่นก่อน / stop the robot first' : '');
    note.title = p.help || '';

    var save = document.createElement('button');
    save.className = 'btn';
    save.textContent = 'บันทึก / Set';
    save.disabled = blocked;
    save.addEventListener('click', function () {
      var value = parseFloat(input.value);
      if (isNaN(value)) { $('paramsMsg').textContent = p.key + ': ใส่ตัวเลข / needs a number'; return; }
      $('paramsMsg').textContent = 'กำลังบันทึก / saving ' + p.key + '…';
      api('/api/params', { key: p.key, value: value }).then(function (r) {
        if (r && r.ok) {
          $('paramsMsg').textContent = p.key + ' = ' + r.value + ' ✓';
          // Re-read rather than trust the write: the board is the authority on
          // what it holds, and a set that was refused downstream would
          // otherwise leave the page showing a value nothing is using.
          setTimeout(loadParams, 800);
        } else {
          $('paramsMsg').textContent = (r && r.error) || 'ไม่สำเร็จ / failed';
        }
      });
    });

    row.appendChild(label);
    row.appendChild(input);
    row.appendChild(save);
    row.appendChild(note);
    return row;
  }

  /* Grouped and filterable, because 23 values in one flat scroll is a list you
   * hunt through rather than one you use. The group tabs mirror the Settings
   * page so both feel like the same control, and the filter searches the help
   * text as well as the key - people look for "diode" or "battery", not for
   * "motor.a.diode_vf". */
  function renderParamGroups() {
    var box = $('paramsForm');
    var tabs = $('paramGroups');
    var f = $('paramsFilter');
    var filter = ((f && f.value) || '').trim().toLowerCase();
    if (!box) return;

    var matches = paramsState.entries.filter(function (p) {
      if (!filter) return true;
      return (p.key + ' ' + (p.help || '') + ' ' + (p.group || ''))
        .toLowerCase().indexOf(filter) >= 0;
    });

    var groups = {};
    matches.forEach(function (p) { (groups[p.group] = groups[p.group] || []).push(p); });
    var names = Object.keys(groups).sort();

    if (tabs) { tabs.textContent = ''; }
    box.textContent = '';

    // A filter that matches nothing must say so, not leave a blank page.
    if (!names.length) {
      var none = document.createElement('p');
      none.className = 'hint';
      none.textContent = 'ไม่พบ / nothing matches "' + filter + '"';
      box.appendChild(none);
      return;
    }

    if (names.indexOf(paramsState.group) < 0) { paramsState.group = names[0]; }
    if (tabs) {
      names.forEach(function (name) {
        var b = document.createElement('button');
        b.textContent = name + ' (' + groups[name].length + ')';
        b.className = (name === paramsState.group) ? 'active' : '';
        b.addEventListener('click', function () {
          paramsState.group = name;
          renderParamGroups();
        });
        tabs.appendChild(b);
      });
    }
    (groups[paramsState.group] || []).forEach(function (p) {
      box.appendChild(paramRow(p));
    });
  }

  /* ------------------------------------------------------- geometry tab ---
   * The same parameters as the settings page, laid out by the THING they
   * describe rather than as one alphabetical list.
   *
   * Flat, they were unusable: 42 rows where every mount looks like every other
   * mount, and "geometry.caster.fl.qy" tells you nothing about which part it
   * belongs to until you have parsed the whole name. Grouped, each card is one
   * physical object you can point at on the robot, with its position and its
   * rotation together - which is how you measure it, and therefore how you
   * should enter it.
   */
  var GEOM_MOUNTS = [
    ['wheel.l',   'ล้อขับซ้าย (หลัง) / Left drive wheel, REAR',   'ใช้จริงในการคำนวณ / used by the maths'],
    ['wheel.r',   'ล้อขับขวา (หลัง) / Right drive wheel, REAR',   'ใช้จริงในการคำนวณ / used by the maths'],
    ['caster.fl', 'ล้อหมุนหน้าซ้าย / Front-LEFT caster',          'บันทึกไว้เฉย ๆ / recorded only'],
    ['caster.fr', 'ล้อหมุนหน้าขวา / Front-RIGHT caster',          'บันทึกไว้เฉย ๆ / recorded only'],
    ['gps',      'เสาอากาศ GPS / GPS antenna',  'ตำแหน่งใช้จริง ทิศทางไม่ใช้ / position used, rotation not'],
    ['imu',      'ไอเอ็มยู / IMU',              'ใช้จริงในการคำนวณ / used by the maths']
  ];

  function geomParam(key) {
    var list = paramsState && paramsState.entries;
    if (!list) return null;
    for (var i = 0; i < list.length; i++) if (list[i].key === key) return list[i];
    return null;
  }

  function renderGeometry() {
    var box = $('geomCards');
    if (!box) return;
    if (!(paramsState && paramsState.entries.length)) {
      box.textContent = '';
      text('geomMsg', 'ยังไม่ได้รับค่าจากบอร์ด / no values from the board yet');
      return;
    }
    text('geomMsg', '');
    box.textContent = '';

    GEOM_MOUNTS.forEach(function (m) {
      var prefix = 'geometry.' + m[0] + '.';
      var card = document.createElement('div');
      card.className = 'card';

      var h = document.createElement('h3');
      h.textContent = m[1];
      var small = document.createElement('small');
      small.textContent = m[2];
      h.appendChild(small);
      card.appendChild(h);

      var pos = document.createElement('div');
      pos.className = 'cfg-form';
      ['x', 'y', 'z'].forEach(function (ax) {
        var p = geomParam(prefix + ax);
        if (p) pos.appendChild(paramRow(p));
      });
      card.appendChild(pos);

      // Which rotation form this mount is using. Showing both at once would
      // invite entering both and expecting them to agree; only one is read.
      var mode = geomParam(prefix + 'rot_mode');
      var useQuat = mode && Math.round(mode.value) === 1;
      var rot = document.createElement('div');
      rot.className = 'cfg-form';
      if (mode) rot.appendChild(paramRow(mode));
      (useQuat ? ['qw', 'qx', 'qy', 'qz'] : ['roll', 'pitch', 'yaw']).forEach(function (k) {
        var p = geomParam(prefix + k);
        if (p) rot.appendChild(paramRow(p));
      });
      var note = document.createElement('p');
      note.className = 'hint';
      note.textContent = useQuat
        ? 'โหมดควอเทอร์เนียน — ค่ามุม roll/pitch/yaw ถูกละไว้ / quaternion mode: the '
          + 'roll, pitch and yaw fields are ignored, not blended in'
        : 'โหมดมุม — ค่าควอเทอร์เนียนถูกละไว้ / angle mode: the quaternion fields are '
          + 'ignored, not blended in';
      rot.appendChild(note);
      card.appendChild(rot);

      box.appendChild(card);
    });
  }

  if ($('geomReload')) $('geomReload').addEventListener('click', loadParams);

  function loadParams() {
    var box = $('paramsForm');
    if (!box) return;
    api('/api/params').then(function (r) {
      (((r && r.params) || [])).forEach(function (p) {
        if (p.key === 'robot.wheel_diameter') { state.wheelDiameterM = p.value; }
      });
      if (!r || !r.ok) { box.textContent = 'อ่านค่าไม่ได้ / cannot read parameters'; return; }
      paramsState.moving = !!r.moving;
      paramsState.entries = r.params || [];
      box.textContent = '';

      if (!r.from_robot) {
        var warn = document.createElement('p');
        warn.className = 'hint';
        warn.textContent = 'ยังไม่ได้รับค่าจากบอร์ด — ช่องที่ว่างคือยังไม่รู้ค่าจริง / ' +
          'the board has not reported its values yet, so blank fields are unknown ' +
          'rather than zero';
        box.appendChild(warn);
      }

      renderParamGroups();
      renderGeometry();
    });
  }

  if ($('paramsFilter')) {
    $('paramsFilter').addEventListener('input', renderParamGroups);
  }
  if ($('paramsReload')) {
    $('paramsReload').addEventListener('click', loadParams);
  }

  /* Write every field that differs from what the board holds, in one press.
   *
   * Setting values one at a time is fine for changing one number and miserable
   * for the case this exists for: bringing a robot's whole configuration back
   * after a reset, or copying a set of geometry measurements in. Only the
   * CHANGED fields are sent - writing all eighty-five every time would put
   * pointless wear on the board's flash and would step on values somebody else
   * had just set.
   *
   * Sent one after another rather than all at once, because each write is
   * bounds-checked on the board and can be refused on its own, and a refusal
   * has to name the parameter it refused. */
  if ($('paramsWriteAll')) {
    $('paramsWriteAll').addEventListener('click', function () {
      var inputs = $('paramsForm').querySelectorAll('input[data-key]');
      var pending = [];
      Array.prototype.forEach.call(inputs, function (input) {
        var value = parseFloat(input.value);
        if (isNaN(value)) return;
        // dataset.boardValue is what the board said when the form was built.
        var onBoard = parseFloat(input.dataset.boardValue);
        if (!isNaN(onBoard) && Math.abs(onBoard - value) < 1e-9) return;
        if (input.disabled) return;            // stopped-only, robot is moving
        pending.push({ key: input.dataset.key, value: value });
      });
      if (!pending.length) {
        $('paramsMsg').textContent = 'ไม่มีอะไรเปลี่ยน / nothing has been changed';
        return;
      }
      var done = 0, failed = [];
      $('paramsMsg').textContent = 'กำลังเขียน / writing ' + pending.length + '…';
      (function next() {
        if (!pending.length) {
          $('paramsMsg').textContent = done + ' เขียนแล้ว / written' +
            (failed.length ? '  ·  ไม่สำเร็จ / refused: ' + failed.join(', ') : '  ✓');
          setTimeout(loadParams, 800);
          return;
        }
        var item = pending.shift();
        api('/api/params', item).then(function (r) {
          if (r && r.ok) done++; else failed.push(item.key);
          next();
        });
      })();
    });
  }
  if ($('paramsReset')) {
    $('paramsReset').addEventListener('click', function () {
      if (!window.confirm('คืนค่าเริ่มต้นทั้งหมดในบอร์ด?\nReset every parameter on the robot to its default?')) return;
      api('/api/params/reset', {}).then(function (r) {
        $('paramsMsg').textContent = (r && r.ok) ? 'คืนค่าแล้ว / reset ✓'
                                                 : ((r && r.error) || 'failed');
        setTimeout(loadParams, 800);
      });
    });
  }

  /* ------------------------------------------------- bug reports ---------
   * Saved on this machine with the robot's state attached. The attached state
   * is the whole point: "it stopped and I don't know why" is the report people
   * actually write, and the answer is nearly always in the stop flags, the
   * firmware version or the last few log lines - all long gone by the time
   * anyone asks about it. */
  function refreshBugList() {
    var box = $('bugList');
    if (!box) return;
    api('/api/bugreport').then(function (response) {
      if (!response || !response.ok) { box.textContent = ''; return; }
      if (!response.reports.length) {
        box.textContent = 'ยังไม่มีรายงาน / no reports yet';
        return;
      }
      box.textContent = response.reports.map(function (r) {
        return (r.saved_at || '?') + '  ' + (r.run_state || '') + '  ' + (r.text || '');
      }).join('\n') + '\n\n' + (response.where || '');
    });
  }

  if ($('bugSave')) {
    $('bugSave').addEventListener('click', function () {
      var text = ($('bugText').value || '').trim();
      if (!text) { $('bugResult').textContent = 'เขียนสั้น ๆ ก็ได้ / say something first'; return; }
      $('bugResult').textContent = 'กำลังบันทึก / saving…';
      api('/api/bugreport', { text: text }).then(function (response) {
        if (response && response.ok) {
          $('bugText').value = '';
          $('bugResult').textContent = 'บันทึกแล้ว / saved: ' + response.saved;
          refreshBugList();
        } else {
          $('bugResult').textContent = 'ไม่สำเร็จ / failed: ' +
            ((response && response.error) || 'unknown');
        }
      });
    });
  }

  /* ------------------------------------------- firmware version banner ----
   * The board and the host update separately, so they drift apart as a matter
   * of course. An older board still works - it just cannot fill in fields it
   * predates, and "a value that is always zero" looks exactly like "a broken
   * sensor" unless someone says otherwise. This says otherwise. */
  var firmwareNoticeShown = false;
  function checkFirmware(snap) {
    if (firmwareNoticeShown) return;
    var t = (snap && snap.telemetry) || {};
    if (!('firmware_version' in t)) return;
    var board = Number(t.firmware_version) || 0;
    if (board === 0) {
      firmwareNoticeShown = true;
      toast('เฟิร์มแวร์เก่า ไม่ได้บอกเวอร์ชัน / the board predates version reporting' +
            ' — flash it to get the newer telemetry', false);
    }
  }

  /* ------------------------------------------------- pan and zoom ---------
   * The map used to be fixed: it auto-fitted once and there was no way to look
   * anywhere else, so anything outside that box was simply unreachable.
   *
   * Hold and drag to move it, wheel to zoom, two fingers to pinch. The pose
   * tool owns the drag while it is armed, so panning stands aside rather than
   * the two fighting over one gesture.
   *
   * Zooming about the CURSOR rather than the centre is what makes it feel like
   * a map - the point under your finger stays under your finger. */
  /* How far in and out the map will go, in pixels per metre.
   *
   * The old floor of 0.15 gave about six kilometres across a laptop canvas,
   * which sounds like plenty until you want to see where the robot is relative
   * to somewhere it is not. The old ceiling was generous in theory and useless
   * in practice, because the grid could not label anything finer than 20 m -
   * see the step ladder in drawMap. Testing in a courtyard or a dormitory car
   * park means reading metres, not tens of metres.
   *
   * 0.02 px/m is roughly 45 km across; 600 px/m is about a metre and a half.
   * Both ends are far past what is useful, which is the point: the limit should
   * be a backstop, not something anybody meets while working. */
  /* 0.0008 px/m is about 1000 km across a laptop canvas; 900 px/m is about one
   * metre. Both were asked for directly: the wide end to see where the robot is
   * relative to somewhere it is not, and the close end to test in a courtyard
   * where everything worth measuring is smaller than a car.
   *
   * The imagery keeps up at both ends. tileZoomFor picks the tile zoom that
   * matches the pixels per metre and stops at what the providers actually
   * publish, so beyond that the last real tiles are simply drawn larger -
   * blurry, but still a street and still a roof, which is what the close end is
   * for. */
  var MIN_SCALE = 0.0008, MAX_SCALE = 900;
  var pan = null;
  var pointers = {};
  var pinchStart = null;

  function clampScale(value) {
    return Math.max(MIN_SCALE, Math.min(MAX_SCALE, value));
  }

  function screenToWorld(sx, sy) {
    return {
      x: view.cx + (sx - canvas.width / 2) / view.scale,
      y: view.cy - (sy - canvas.height / 2) / view.scale
    };
  }

  function zoomAbout(sx, sy, factor) {
    var before = screenToWorld(sx, sy);
    view.scale = clampScale(view.scale * factor);
    var after = screenToWorld(sx, sy);
    view.cx += before.x - after.x;
    view.cy += before.y - after.y;
    state.fitRequested = false;        // the operator is driving the view now
    drawMap();
  }

  canvas.addEventListener('wheel', function (event) {
    event.preventDefault();
    var at = eventToCanvas(event);
    // Only the SIGN of deltaY: a trackpad and a mouse wheel report wildly
    // different magnitudes, and using them directly makes one of the two
    // unusable.
    zoomAbout(at.x, at.y, event.deltaY < 0 ? 1.15 : 1 / 1.15);
  }, { passive: false });

  var wpDragWasClick = false;
  var newWpTimer = null;
  var newWpDrag = null;
  var rightClickWp = -1;
  var pointerDownLoc = null;

  canvas.addEventListener('pointerdown', function (event) {
    pointers[event.pointerId] = eventToCanvas(event);
    if (Object.keys(pointers).length === 2) {
      var pts = Object.keys(pointers).map(function (k) { return pointers[k]; });
      pinchStart = {
        dist: Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y),
        scale: view.scale
      };
      pan = null;
      state.wpDrag = null;
      newWpDrag = null;
      if (newWpTimer) { clearTimeout(newWpTimer); newWpTimer = null; }
      return;
    }
    if (state.poseTool.armed) return;

    var at = eventToCanvas(event);
    state.wpDrag = null;
    newWpDrag = null;
    rightClickWp = -1;
    pointerDownLoc = at;
    if (newWpTimer) { clearTimeout(newWpTimer); newWpTimer = null; }

    // Check if clicked on a waypoint
    var hitWp = -1;
    if (state.waypoints && state.datum) {
      for (var i = 0; i < state.waypoints.length; i++) {
        var wp = state.waypoints[i];
        var p = enu(wp.lat, wp.lon, state.datum);
        var s = toScreen(p);
        if (Math.hypot(s.x - at.x, s.y - at.y) < 20) {
          hitWp = i;
          break;
        }
      }
    }

    if (event.button === 2) {
      if (hitWp >= 0) {
        rightClickWp = hitWp;
        canvas.setPointerCapture(event.pointerId);
      } else {
        pan = { at: at, cx: view.cx, cy: view.cy };
        canvas.style.cursor = 'grabbing';
        canvas.setPointerCapture(event.pointerId);
      }
      return;
    }

    if (event.button === 0) {
      if (hitWp >= 0) {
        state.wpDrag = { index: hitWp, from: { lat: state.waypoints[hitWp].lat, lon: state.waypoints[hitWp].lon }, to: pointerToWorld(event) };
        canvas.style.cursor = 'crosshair';
        canvas.setPointerCapture(event.pointerId);
        event.preventDefault();
      } else {
        // Start panning immediately so mobile 1-finger drag works.
        pan = { at: at, cx: view.cx, cy: view.cy };
        canvas.style.cursor = 'grabbing';
        canvas.setPointerCapture(event.pointerId);
        var worldAt = pointerToWorld(event);
        newWpTimer = setTimeout(function() {
          newWpTimer = null;
          pan = null;
          newWpDrag = { from: worldAt, to: worldAt };
          canvas.style.cursor = 'crosshair';
          drawMap();
          if (navigator.vibrate) navigator.vibrate(50);
        }, 400);
      }
      return;
    }
  });

  // Prevent browser context menu on right click so we can pan
  canvas.addEventListener('contextmenu', function(e) { e.preventDefault(); });

  canvas.addEventListener('pointermove', function (event) {
    if (!(event.pointerId in pointers)) return;
    pointers[event.pointerId] = eventToCanvas(event);

    if (pinchStart && Object.keys(pointers).length >= 2) {
      var pts = Object.keys(pointers).map(function (k) { return pointers[k]; });
      var dist = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
      if (dist > 4 && pinchStart.dist > 4) {
        var mid = { x: (pts[0].x + pts[1].x) / 2, y: (pts[0].y + pts[1].y) / 2 };
        var target = clampScale(pinchStart.scale * (dist / pinchStart.dist));
        zoomAbout(mid.x, mid.y, target / view.scale);
      }
      return;
    }

    var now = eventToCanvas(event);
    if (newWpTimer) {
      if (Math.hypot(now.x - pointerDownLoc.x, now.y - pointerDownLoc.y) > 10) {
        clearTimeout(newWpTimer);
        newWpTimer = null;
      }
    }

    if (newWpDrag) {
      var world = pointerToWorld(event);
      if (world) {
        newWpDrag.to = world;
        var dx = world.x - enu(newWpDrag.from.lat, newWpDrag.from.lon, state.datum).x;
        var dy = world.y - enu(newWpDrag.from.lat, newWpDrag.from.lon, state.datum).y;
        var heading = (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
        $('mapHud').textContent = 'เพิ่มจุดใหม่ / New WP\n' + heading.toFixed(0) + '°';
        drawMap();
      }
      return;
    }

    if (state.wpDrag) {
      var worldDrag = pointerToWorld(event);
      if (worldDrag) {
        state.wpDrag.to = worldDrag;
        var dxw = worldDrag.x - enu(state.wpDrag.from.lat, state.wpDrag.from.lon, state.datum).x;
        var dyw = worldDrag.y - enu(state.wpDrag.from.lat, state.wpDrag.from.lon, state.datum).y;
        var hw = (Math.atan2(dxw, dyw) * 180 / Math.PI + 360) % 360;
        var name = state.waypoints[state.wpDrag.index].name || ('WP' + (state.wpDrag.index + 1));
        $('mapHud').textContent = 'ตั้งทิศทาง ' + name + ' / set heading\n' + hw.toFixed(0) + '°';
        drawMap();
      }
      return;
    }

    if (!pan) return;
    view.cx = pan.cx - (now.x - pan.at.x) / view.scale;
    view.cy = pan.cy + (now.y - pan.at.y) / view.scale;
    state.fitRequested = false;
    drawMap();
  });

  function endPan(event) {
    if (newWpTimer) { clearTimeout(newWpTimer); newWpTimer = null; }
    delete pointers[event.pointerId];
    if (Object.keys(pointers).length < 2) pinchStart = null;
    if (Object.keys(pointers).length === 0) {
      pan = null;
      
      if (rightClickWp >= 0) {
        var dp = Math.hypot(eventToCanvas(event).x - pointerDownLoc.x, eventToCanvas(event).y - pointerDownLoc.y);
        if (dp < 10) {
          state.picked = { lat: state.waypoints[rightClickWp].lat, lon: state.waypoints[rightClickWp].lon };
          drawMap();
          openMapMenu(event, state.picked, rightClickWp);
        }
        rightClickWp = -1;
        try { canvas.releasePointerCapture(event.pointerId); } catch(e){}
        if (!state.poseTool.armed) canvas.style.cursor = 'grab';
        return;
      }

      if (newWpDrag) {
        wpDragWasClick = true;
        var dn = newWpDrag.to.x - enu(newWpDrag.from.lat, newWpDrag.from.lon, state.datum).x;
        var dy2 = newWpDrag.to.y - enu(newWpDrag.from.lat, newWpDrag.from.lon, state.datum).y;
        if (Math.hypot(dn, dy2) > 0.3) {
          var hd = (Math.atan2(dn, dy2) * 180 / Math.PI + 360) % 360;
          if (confirm('สร้างจุดหมายใหม่ที่นี่ หันหน้า ' + hd.toFixed(0) + '° หรือไม่?\nCreate new waypoint here heading ' + hd.toFixed(0) + '°?')) {
            var points = state.waypoints.slice();
            points.push({
              name: 'WP' + (points.length + 1),
              lat: Number(newWpDrag.from.lat.toFixed(7)),
              lon: Number(newWpDrag.from.lon.toFixed(7)),
              heading_deg: hd
            });
            api('/api/waypoints', { waypoints: points }).then(function (res) {
              if (res.ok) {
                state.waypoints = res.waypoints || points;
                markWaypointsSaved('เพิ่มจุดใหม่แล้ว / added');
                renderWaypoints();
                drawMap();
              }
            });
          }
        }
        newWpDrag = null;
        if (!state.poseTool.armed) canvas.style.cursor = 'grab';
        drawMap();
        try { canvas.releasePointerCapture(event.pointerId); } catch(e){}
        return;
      }

      if (state.wpDrag) {
        var dx = state.wpDrag.to.x - enu(state.wpDrag.from.lat, state.wpDrag.from.lon, state.datum).x;
        var dy = state.wpDrag.to.y - enu(state.wpDrag.from.lat, state.wpDrag.from.lon, state.datum).y;
        var dist = Math.hypot(dx, dy);
        if (dist > 0.3) {
          var heading = (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
          state.waypoints[state.wpDrag.index].heading_deg = heading;
          markWaypointsDirty();
          renderWaypoints();
          api('/api/waypoints', { waypoints: state.waypoints }).then(function (res) {
            if (res.ok) { markWaypointsSaved(); renderWaypoints(); toast('บันทึกทิศทางแล้ว / heading saved', false); }
          });
          wpDragWasClick = false;
        } else {
          wpDragWasClick = true;
          state.picked = { lat: state.waypoints[state.wpDrag.index].lat, lon: state.waypoints[state.wpDrag.index].lon };
          drawMap();
          openMapMenu(event, state.picked, state.wpDrag.index);
        }
        try { canvas.releasePointerCapture(event.pointerId); } catch(e){}
        state.wpDrag = null;
      }
      if (!state.poseTool.armed) canvas.style.cursor = 'grab';
    }
  }
  canvas.addEventListener('pointerup', endPan);
  canvas.addEventListener('pointercancel', endPan);
  canvas.addEventListener('pointerleave', endPan);

  /* Buttons too: a phone has no wheel, and pinching a small map is fiddly. */
  if ($('mapZoomIn')) {
    $('mapZoomIn').addEventListener('click', function () {
      zoomAbout(canvas.width / 2, canvas.height / 2, 1.4);
    });
  }
  if ($('mapZoomOut')) {
    $('mapZoomOut').addEventListener('click', function () {
      zoomAbout(canvas.width / 2, canvas.height / 2, 1 / 1.4);
    });
  }

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
    if (wpDragWasClick) {
      wpDragWasClick = false;
      return;
    }
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

  function openMapMenu(event, world, wpIndex) {
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

    if (wpIndex !== undefined) {
      $('mapMenuPos').textContent = state.waypoints[wpIndex].name || ('WP' + (wpIndex + 1));
      $('mapMenuGoto').classList.remove('hidden');
      $('mapMenuAdd').classList.add('hidden');
      $('mapMenuCopy').classList.remove('hidden');
      if ($('mapMenuDelete')) $('mapMenuDelete').classList.remove('hidden');
      menu.dataset.wpIndex = wpIndex;
    } else {
      $('mapMenuPos').textContent =
        world.lat.toFixed(7) + ', ' + world.lon.toFixed(7) + describePoint(world, '\n');
      $('mapMenuGoto').classList.remove('hidden');
      $('mapMenuAdd').classList.remove('hidden');
      $('mapMenuCopy').classList.remove('hidden');
      if ($('mapMenuDelete')) $('mapMenuDelete').classList.add('hidden');
      delete menu.dataset.wpIndex;
    }
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
      var menu = $('mapMenu');
      var wpIndex = menu.dataset.wpIndex !== undefined ? parseInt(menu.dataset.wpIndex, 10) : -1;
      
      if (!world && wpIndex < 0) { closeMapMenu(); return; }

      if (button.dataset.map === 'goto') {
        if (blockedInViewer()) return;
        if (blockedByEmergency()) return;
        if (wpIndex >= 0) {
          api('/api/nav', { action: 'goto', index: wpIndex });
        } else {
          api('/api/nav', { action: 'goto_point', lat: world.lat, lon: world.lon })
            .then(function (response) {
              toast(response && response.ok
                ? ''
                : 'สั่งไปจุดนี้ไม่ได้: ' + ((response && response.error) || ''), !(response && response.ok));
            });
        }
      } else if (button.dataset.map === 'add') {
        var points = state.waypoints.slice();
        points.push({
          name: 'WP' + (points.length + 1),
          lat: Number(world.lat.toFixed(7)),
          lon: Number(world.lon.toFixed(7))
        });
        api('/api/waypoints', { waypoints: points }).then(function (response) {
          if (response.ok) {
            state.waypoints = response.waypoints || points;
            markWaypointsSaved('เพิ่มเข้ารายการแล้ว / added');
            renderWaypoints();
          } else {
            toast('เพิ่มไม่ได้: ' + response.error, true);
          }
        });
      } else if (button.dataset.map === 'delete') {
        if (wpIndex >= 0) {
          var removed = state.waypoints[wpIndex];
          state.waypoints.splice(wpIndex, 1);
          markWaypointsDirty();
          renderWaypoints();
          api('/api/waypoints', { waypoints: state.waypoints }).then(function (res) {
            if (res.ok) markWaypointsSaved();
          });
          offerUndo('ลบ "' + (removed.name || '') + '" แล้ว / deleted', function () {
            state.waypoints.splice(wpIndex, 0, removed);
            markWaypointsDirty();
            renderWaypoints();
            api('/api/waypoints', { waypoints: state.waypoints }).then(function (res) {
              if (res.ok) markWaypointsSaved();
            });
            drawMap();
          });
        }
      } else if (button.dataset.map === 'copy') {
        var wp = (wpIndex >= 0) ? state.waypoints[wpIndex] : world;
        copyCoordinates(wp.lat.toFixed(7) + ', ' + wp.lon.toFixed(7));
      }
      closeMapMenu();
      state.picked = null;
      drawMap();
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
        // Optional facing. Left blank the robot stops in whatever direction it
        // arrived, which is what most waypoints want; filled in, it turns on the
        // spot to that compass bearing after arriving, before the next leg.
        '<td><input data-k="heading_deg" type="number" step="1" min="0" max="359" ' +
          'placeholder="—" title="ทิศที่ต้องการหันเมื่อถึงจุดนี้ (0-359 องศา) เว้นว่าง = ไม่กำหนด' +
          ' / compass heading to face on arrival, blank = any" aria-label="heading ' + n +
          '" value="' + (wp.heading_deg === undefined || wp.heading_deg === null
                         ? '' : wp.heading_deg) + '"></td>' +
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
          if (key === 'name') {
            state.waypoints[index].name = input.value;
          } else if (key === 'heading_deg') {
            // Blank is a real answer here - it means "any direction" - so it
            // must delete the field rather than become Number('') === 0, which
            // would silently order the robot to face north.
            if (input.value.trim() === '') delete state.waypoints[index].heading_deg;
            else state.waypoints[index].heading_deg = ((Number(input.value) % 360) + 360) % 360;
          } else {
            state.waypoints[index][key] = Number(input.value);
          }
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
    
    var mapCard = $('mapWpCard');
    if (mapCard) {
      if (isEmpty) {
        mapCard.style.display = 'none';
      } else {
        mapCard.style.display = 'block';
        var mapBody = $('mapWpTable').querySelector('tbody');
        mapBody.textContent = '';
        state.waypoints.forEach(function (wp, index) {
          var row = document.createElement('tr');
          var n = index + 1;
          var headStr = (wp.heading_deg === undefined || wp.heading_deg === null) ? '—' : wp.heading_deg + '°';
          row.innerHTML =
            '<td>' + n + '</td>' +
            '<td>' + escapeHtml(wp.name || ('WP' + n)) + '</td>' +
            '<td><small>' + fmt(wp.lat, 6) + '<br>' + fmt(wp.lon, 6) + '</small></td>' +
            '<td>' + headStr + '</td>' +
            '<td><button class="iconbtn" title="ไปดูบนแผนที่ / Zoom on map">🔍</button></td>';
          row.querySelector('button').addEventListener('click', function() {
            if (!state.datum) return;
            var q = enu(wp.lat, wp.lon, state.datum);
            view.cx = q.x; view.cy = q.y;
            view.scale = Math.max(view.scale, 8);
            state.fitRequested = false;
            drawMap();
            // Scroll up to the map so the user sees the result
            var mapWrap = document.querySelector('.mapwrap');
            if (mapWrap) mapWrap.scrollIntoView({ behavior: 'smooth', block: 'center' });
          });
          mapBody.appendChild(row);
        });
      }
    }
    
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

    if ($('wpLoopToggle')) {
    api('/api/config?group=nav').then(function (res) {
      if (res.ok && res.data) $('wpLoopToggle').checked = !!res.data.loop_waypoints;
    });
    $('wpLoopToggle').addEventListener('change', function () {
      api('/api/config', { group: 'nav', data: { loop_waypoints: this.checked } }).then(function (res) {
        if (!res.ok) alert('Failed to save setting: ' + res.error);
        else toast('ตั้งโหมดวนซ้ำแล้ว');
      });
    });
  }

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
    'nav.arrive_radius_m': ['รัศมีถือว่าถึง', 'Arrive radius', 'เข้าใกล้เท่านี้ = ถึงจุดหมายแล้ว (m) - ต่ำกว่า 2 m ไม่ได้ GPS ไม่แม่นพอ'],
    'nav.arrive_hold_s': ['ต้องอยู่ในรัศมีนาน', 'Arrive hold', 'ต้องอยู่ในรัศมีนานเท่านี้จึงนับว่าถึง (s)'],
    'nav.arrive_sigma_k': ['ตัวคูณความไม่แน่นอน', 'Arrive sigma factor', 'รัศมีขยายตามความไม่แน่นอนของตำแหน่ง'],
    'nav.align_timeout_s': ['เวลาหมุนหาทิศสูงสุด', 'Align timeout', 'หมุนหาทิศของจุดหมายนานสุด (s) เกินนี้ไปต่อเลย'],
    'nav.heading_tolerance_deg': ['ความคลาดเคลื่อนทิศ', 'Heading tolerance', 'ถ้าเบี่ยงเกินนี้จะหมุนตัวก่อนวิ่ง (°)'],
    'nav.command_rate_hz': ['ความถี่ส่งคำสั่ง', 'Command rate', 'ครั้งต่อวินาที'],
    'nav.loop_waypoints': ['วนซ้ำ', 'Loop waypoints', 'ถึงจุดสุดท้ายแล้วเริ่มใหม่'],
    'nav.waypoint_timeout_s': ['หมดเวลาต่อจุด', 'Waypoint timeout', 'ไปไม่ถึงภายในเวลานี้จะหยุด (s)'],
    'nav.require_fix': ['ต้องมีสัญญาณ GPS', 'Require fix', 'ปิดได้เฉพาะตอนทดสอบในร่ม'],
    'nav.min_satellites': ['ดาวเทียมขั้นต่ำ', 'Min satellites', 'น้อยกว่านี้จะไม่ยอมวิ่ง'],
    'nav.max_hdop': ['HDOP สูงสุด', 'Max HDOP', 'ค่ายิ่งน้อยยิ่งแม่น (แนะนำ 3)'],
    'nav.fix_timeout_s': ['GPS ขาดได้นานสุด', 'Fix timeout', 'วินาที'],

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

    'gps.min_sats_speed': ['ดาวเทียมขั้นต่ำ (ความเร็ว)', 'Min sats for speed', 'ต่ำกว่านี้ไม่เชื่อความเร็วจาก GPS ใช้ IMU + มอเตอร์แทน'],
    'gps.max_hdop_speed': ['HDOP แย่สุด (ความเร็ว)', 'Max HDOP for speed', 'แย่กว่านี้ไม่เชื่อความเร็วจาก GPS'],
    'gps.min_sats_fix': ['ดาวเทียมขั้นต่ำ (มีตำแหน่ง)', 'Min sats for a fix', 'ต่ำกว่านี้ถือว่า GPS ใช้ไม่ได้เลย (4 = fix 3 มิติ)'],
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
    // The theme is a setting of this browser, not of the robot, so it appears
    // with the rest of the "เว็บ / Web" group and nowhere else.
    // The .hidden CLASS, not the hidden attribute: .row sets display:flex, which
    // beats the browser's [hidden] rule, so the attribute silently did nothing.
    var themeRow = $('themeRow');
    if (themeRow) themeRow.classList.toggle('hidden', state.cfg.group !== 'web');
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
  setView();
  loadWaypoints();
  buildGroupTabs();
  setupTopics();
  setConnection('connecting');
  connect();
  // Fetch the wheel diameter once at start-up, so wheel speed can be shown in
  // m/s straight away. loadParams() also picks it up, but only when someone
  // opens the settings tab - and a speed reading should not depend on that.
  api('/api/params').then(function (r) {
    (((r && r.params) || [])).forEach(function (pp) {
      if (pp.key === 'robot.wheel_diameter') { state.wheelDiameterM = pp.value; }
    });
  });
  armHeartbeat(heartbeatMs);
  setInterval(watchFreshness, 1000);   // degrade to stale/offline on its own
  heartbeat();

  /* The error banner is fixed and sits BELOW the toolbar, so it needs the
   * toolbar's height. The CSS asks for --toolbar-height with a 240px fallback,
   * and nothing was setting it - so on a laptop, where the toolbar is about
   * 64px, the banner floated a quarter of the way down the page over unrelated
   * content, and on a phone it landed somewhere else again. It has to be
   * measured rather than assumed: the toolbar wraps differently at every width
   * and grows a row when the pills wrap.
   *
   * The banner was moved below the toolbar in the first place because at a
   * higher z-index it covered the EMERGENCY STOP and swallowed the taps meant
   * for it. That is the one control that must never be unreachable, so the
   * measurement is not cosmetic. */
  function measureToolbar() {
    var bar = document.querySelector('.topbar');
    if (!bar) return;
    var h = Math.round(bar.getBoundingClientRect().height);
    if (h > 0) document.documentElement.style.setProperty('--toolbar-height', h + 'px');
  }
  measureToolbar();
  window.addEventListener('resize', measureToolbar);
  window.addEventListener('orientationchange', measureToolbar);
  // The toolbar also changes height without the window changing size - the
  // state subtitle and the pills are rewritten on every telemetry frame - so
  // watch the element itself where the browser supports it.
  if (typeof ResizeObserver === 'function') {
    var bar = document.querySelector('.topbar');
    if (bar) new ResizeObserver(measureToolbar).observe(bar);
  }

  window.addEventListener('beforeunload', function () { stopJog(); });
})();
