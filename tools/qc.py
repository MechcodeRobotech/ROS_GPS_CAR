#!/usr/bin/env python3
"""Project QC - checks the contracts that span more than one file.

    python3 tools/qc.py

Compilers catch mistakes inside a file. The bugs that actually bite this
project live *between* files: the firmware writes telemetry slot 38 and the
Python side never names it, the UI calls an endpoint the viewer does not serve,
a CSS utility class silently loses to a component rule, a YAML key is read with
one spelling and written with another. That is what this script checks.

Exit code 0 = clean, 1 = something to fix. Safe to run any time, it only reads.
"""

import os
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
FW = ROOT / 'firmware'
PKG = ROOT / 'gps_localize_ws' / 'src' / 'gps_localize'
WEB = PKG / 'web'

problems = []
notes = []


def read(path):
    return path.read_text(encoding='utf-8')


def check(ok, message):
    print(('  ok    ' if ok else '  FAIL  ') + message)
    if not ok:
        problems.append(message)


def section(title):
    print('\n[%s]' % title)


# ===========================================================================
def check_telemetry(main_cpp, telemetry_py):
    """The firmware sets every field the message declares.

    Superseded the old index check: there is no TELEMETRY_LEN and no t[i] any
    more, because the packed Float32MultiArray became a named message. What can
    still go wrong is a field declared in the .msg and never assigned in
    publishTelemetry - it would publish a silent zero rather than fail, which is
    worse than a mismatch that stops the build.
    """
    section('telemetry: every declared field is actually set')
    msg = read(ROOT / 'gps_localize_ws' / 'src' / 'gps_localize_msgs' / 'msg' / 'Telemetry.msg')
    declared = re.findall(r'^float32\s+([a-z0-9_]+)', msg, re.M)

    start = main_cpp.index('static void publishTelemetry')
    body = main_cpp[start:main_cpp.index('rcl_publish(&pub_telemetry', start)]
    assigned = set(re.findall(r'^\s*m\.([a-z0-9_]+)\s*=', body, re.M))

    never_set = [f for f in declared if f not in assigned]
    check(not never_set,
          'every field in Telemetry.msg is assigned in publishTelemetry %s'
          % (never_set or '(%d fields)' % len(declared)))

    unknown = sorted(assigned - set(declared))
    check(not unknown,
          'publishTelemetry sets nothing that the message does not declare %s'
          % (unknown or ''))


def check_config_selectors(main_cpp, telemetry_py, web_py):
    section('config topic selectors: firmware <-> web_server')
    fw = sorted({int(m) for m in re.findall(r'sel == (\d+)', main_cpp)})
    py = sorted({int(float(m)) for m in
                 re.findall(r'^CONFIG_[A-Z_]+ = ([\d.]+)', telemetry_py, re.M)})
    check(set(py).issubset(set(fw)), 'python selectors %s all exist in firmware %s' % (py, fw))
    for name in sorted(set(re.findall(r'tel\.(CONFIG_[A-Z_]+)', web_py))):
        check(name in telemetry_py, '%s is defined' % name)


def check_topics(main_cpp, py_all):
    section('ROS topics: firmware <-> nodes')
    # Published for other tools rather than for our own nodes - not a mismatch.
    EXTERNAL_ONLY = {
        '/cmd_vel': 'standard teleop input (teleop_twist_keyboard, joysticks)',
        '/imu/data': 'sensor output for rviz / bag files',
        '/odom': 'odometry output for rviz / bag files',
    }
    fw = sorted(set(re.findall(r'ROS_NS "(/[a-z_/]+)"', main_cpp)))
    py = set(re.findall(r"NS \+ '(/[a-z_/]+)'", py_all))
    for topic in fw:
        if topic in EXTERNAL_ONLY:
            notes.append('%s is an external interface (%s)' % (topic, EXTERNAL_ONLY[topic]))
            continue
        check(topic in py, 'firmware topic %s has a node on the other end' % topic)


def check_endpoints(app_js, web_py, viewer_py):
    section('HTTP endpoints: UI <-> robot server <-> windows viewer')
    ui = sorted(set(re.findall(r"api\('(/api/[a-z_/]+)'", app_js)) | {'/api/stream'})
    robot = set(re.findall(r"route\('(?:GET|POST)', '(/api/[a-z_/]+)'", web_py)) | {'/api/stream'}
    viewer = set(re.findall(r"route\('(?:GET|POST)', '(/api/[a-z_/]+)'", viewer_py)) | {'/api/stream'}
    missing_robot = [p for p in ui if p not in robot]
    missing_viewer = [p for p in ui if p not in viewer]
    check(not missing_robot, 'robot server serves every endpoint the UI calls %s'
          % (missing_robot or ''))
    check(not missing_viewer, 'windows viewer serves every endpoint the UI calls %s'
          % (missing_viewer or ''))


def check_web_assets(app_js, html, css):
    section('web UI: ids, hooks and the hide utility')

    # A botched search-and-replace once spliced the whole document onto itself.
    # Singletons are the cheapest way to notice that immediately.
    for marker, want in (('<html', 1), ('<body>', 1), ('<main>', 1), ('</main>', 1),
                         ('</html>', 1), ('<footer', 1), ('class="jog', 5)):
        got = html.count(marker)
        check(got == want, 'index.html contains %s exactly %d time(s) (found %d)'
              % (marker, want, got))
    for tab in ('dash', 'map', 'wp', 'cfg', 'topics', 'log'):
        check(html.count('id="tab-%s"' % tab) == 1, 'exactly one #tab-%s section' % tab)
    ids_html = set(re.findall(r'id="([A-Za-z0-9_]+)"', html))
    ids_js = set(re.findall(r"\$\('([A-Za-z0-9_]+)'\)", app_js))
    missing = sorted(ids_js - ids_html)
    check(not missing, 'every element the JS looks up exists in the HTML %s' % (missing or ''))

    dupes = sorted(i for i in ids_html if len(re.findall('id="%s"' % i, html)) > 1)
    check(not dupes, 'no duplicate element ids %s' % (dupes or ''))

    for attr in ('data-nav', 'data-map', 'data-speed', 'data-turnrate',
                 'data-compass', 'data-turn-deg', 'data-tab'):
        if ('%s="' % attr) not in html:
            continue
        # either selected with [data-x] or read through element.dataset.x
        camel = re.sub(r'-(\w)', lambda m: m.group(1).upper(), attr[len('data-'):])
        wired = ('[%s]' % attr) in app_js or ('dataset.%s' % camel) in app_js
        check(wired, '%s buttons are wired up in JS' % attr)

    # A utility class must beat component rules that set `display`, whatever the
    # order in the file. This is exactly how the map menu got stuck open once.
    hidden = re.search(r'\.hidden\s*\{[^}]*\}', css)
    check(bool(hidden) and 'none !important' in hidden.group(0),
          '.hidden uses !important so it always wins over component rules')
    for classes in set(re.findall(r'class="([^"]*\bhidden\b[^"]*)"', html)):
        for name in classes.split():
            if name == 'hidden':
                continue
            rule = re.search(r'\.' + re.escape(name) + r'\s*\{[^}]*\}', css)
            if rule and 'display:' in rule.group(0):
                notes.append('.%s sets display and is used with .hidden '
                             '(safe: .hidden is !important)' % name)


def check_design_system(css, html):
    """The rules from the webapp-design method, as far as they can be checked
    statically: token contract, spacing scale, type scale, responsive floor."""
    section('design system')
    root = re.search(r':root\s*\{(.*?)\}', css, re.S)
    root_body = root.group(1) if root else ''

    for token in ('--bg', '--card', '--sunk', '--line', '--txt', '--mut',
                  '--acc', '--on-acc', '--ok', '--warn', '--err'):
        check(token + ':' in root_body, 'token %s is defined' % token)

    # spacing: only the 4px scale, used through tokens
    scale = set(re.findall(r'--sp-\d+:\s*(\d+)px', root_body))
    check(scale.issubset({'4', '8', '12', '16', '24', '32'}),
          'spacing tokens stay on the 4px scale (%s)' % sorted(scale, key=int))

    # type: four sizes as tokens; component rules may use a couple of one-offs
    type_tokens = re.findall(r'--fs-\d+:\s*(\d+)px', root_body)
    check(len(type_tokens) <= 5, 'at most 5 type-size tokens (%s)' % type_tokens)
    literal_sizes = sorted({int(v) for v in re.findall(r'font-size:\s*(\d+)px', css)})
    check(len(literal_sizes) <= 6,
          'few hardcoded font sizes outside the scale (%s)' % literal_sizes)

    # a colour repeated outside :root should have been a token
    outside = css[:root.start()] + css[root.end():] if root else css
    outside = re.sub(r'@media \(prefers-color-scheme: light\)\s*\{.*?\n\}', '', outside, flags=re.S)
    repeated = [c for c, n in
                ((c, outside.count(c)) for c in set(re.findall(r'#[0-9a-fA-F]{3,6}', outside)))
                if n >= 3]
    check(not repeated, 'no colour repeated 3+ times outside :root %s' % (repeated or ''))

    # responsive floor
    check('@media (pointer: coarse)' in css, 'touch targets: @media (pointer: coarse) block exists')
    check('prefers-reduced-motion' in css, 'motion: prefers-reduced-motion block exists')
    check('overflow-x: hidden' in css, 'body cannot scroll sideways')
    check('overflow-x: auto' in css, 'wide content scrolls inside its own container')
    breakpoints = sorted({int(v) for v in re.findall(r'max-width:\s*(\d+)px\)', css)})
    check(len(breakpoints) >= 1, 'has content breakpoints %s' % breakpoints)
    check('minmax(min(' in css, 'grids shrink instead of overflowing (minmax(min(...)))')

    # the five states have somewhere to live
    for marker, what in (('.skeleton', 'loading'), ('.empty', 'empty'),
                         ('bannerRetry', 'error + retry'), ('data-conn="stale"', 'stale')):
        check(marker in css or marker in html, 'state styling present: %s' % what)

    # accessibility floor
    check(':focus-visible' in css, 'focus ring is styled, not removed')
    check(html.count('aria-live') >= 2, 'status regions announce themselves (aria-live)')
    inputs = re.findall(r'<input\b[^>]*>', html)
    unlabelled = [i for i in inputs
                  if 'aria-label' not in i and 'type="range"' not in i
                  and not re.search(r'id="(\w+)"', i)]
    check(not unlabelled, 'every input has a label or aria-label %s' % (unlabelled or ''))

    # one primary button per tab page
    for page in re.findall(r'<section id="tab-(\w+)".*?(?=<section id="tab-|</main>)', html, re.S):
        pass
    pages = re.split(r'<section id="tab-', html)[1:]
    for page in pages:
        name = page.split('"')[0]
        primaries = page.count('class="btn go"')
        check(primaries <= 2, 'tab "%s" has at most one primary button per view (%d)'
              % (name, primaries))


def check_yaml(py_all):
    section('configuration files')
    try:
        import yaml
    except ImportError:
        notes.append('PyYAML not installed, config checks skipped')
        return
    groups = re.findall(r"'([a-z]+)'",
                        re.search(r'GROUPS = \(([^)]*)\)',
                                  read(PKG / 'gps_localize' / 'configstore.py')).group(1))
    for name in groups:
        path = PKG / 'config' / (name + '.yaml')
        if not path.exists():
            check(False, 'config/%s.yaml exists' % name)
            continue
        try:
            data = yaml.safe_load(read(path))
        except yaml.YAMLError as exc:
            check(False, '%s.yaml parses (%s)' % (name, exc))
            continue
        check(isinstance(data, dict) and name in data,
              '%s.yaml has its "%s:" section' % (name, name))

    # every key the firmware-facing push reads must exist in the shipped file
    for group, keys in (('motor', ('accel_ramp_s', 'decel_ramp_s', 'ke_v_per_rpm',
                                   'resistance_ohm', 'stall_current_a',
                                   'diode_a_vf_v', 'diode_b_vf_v')),
                        ('estimator', ('heading_source', 'mag_enable', 'mag_tau_s',
                                       'mag_declination_deg', 'track_m')),
                        ('nav', ('cruise_speed_mps', 'arrive_radius_m',
                                 'allow_dead_reckoning', 'discover_heading')),
                        ('safety', ('heartbeat_rate_hz', 'require_web_heartbeat'))):
        path = PKG / 'config' / (group + '.yaml')
        if not path.exists():
            continue
        text = read(path)
        missing = [k for k in keys if (k + ':') not in text]
        check(not missing, '%s.yaml defines the keys the code reads %s' % (group, missing or ''))


def check_firmware_headers(main_cpp):
    section('firmware config split')
    config_h = read(FW / 'config' / 'config.h')
    included = re.findall(r'#include "([a-z0-9_]+\.h)"', config_h)
    # network_secrets.h holds the real Wi-Fi credentials and is gitignored;
    # network_secrets.example.h is its committed template. Neither is included
    # by config.h - network.h pulls the secrets in - so they are not part of
    # this contract and must not make it fail.
    on_disk = sorted(p.name for p in (FW / 'config').glob('*.h')
                     if p.name != 'config.h' and not p.name.startswith('network_secrets'))
    check(sorted(included) == on_disk,
          'config.h includes exactly the headers on disk (%d)' % len(on_disk))
    for header in included:
        check((FW / 'config' / header).exists(), '%s exists' % header)
    # Every project header main.cpp includes must exist somewhere we build
    # from - lib/ for reusable code, src/ for this robot's own modules.
    lib_headers = ({p.name for p in (FW / 'lib').rglob('*.h')} |
                   {p.name for p in (FW / 'src').rglob('*.h')})
    for inc in re.findall(r'#include <([a-zA-Z0-9_]+\.h)>', main_cpp):
        if inc in lib_headers or inc.startswith(('rcl', 'rmw', 'std_msgs', 'sensor_msgs',
                                                 'nav_msgs', 'geometry_msgs', 'builtin')):
            continue
        # Arduino core, ESP32 SDK and PlatformIO lib_deps live outside lib/
        if inc in ('Arduino.h', 'Wire.h', 'Preferences.h', 'ESPmDNS.h', 'config.h',
                   'stdio.h', 'math.h', 'string.h', 'micro_ros_platformio.h'):
            continue
        check(False, 'main.cpp includes <%s> but no such header is in lib/ or src/' % inc)


def check_native_tests():
    section('host tests')
    test_dir = FW / 'test' / 'test_native'
    total = 0
    runners = []
    for path in sorted(test_dir.glob('*.cpp')):
        count = len(re.findall(r'RUN_TEST\(', read(path)))
        if count:
            total += count
            print('  info   %-26s %3d tests' % (path.name, count))
        runners += re.findall(r'^int (run_\w+)\(void\)', read(path), re.M)
    main = read(test_dir / 'main.cpp')
    for runner in sorted(set(runners)):
        check(runner + '();' in main, '%s is called by the test runner' % runner)
    check(total >= 100, 'test suite has %d cases' % total)


def check_viewer_build():
    section('windows viewer build')
    exe = ROOT / 'webapp' / 'exe' / 'GPS_Localize_Viewer.exe'
    # The frozen viewer is a Windows convenience build. On Linux the UI is served
    # straight from web/ by web_server, so a stale exe cannot affect what anyone
    # sees here and must not fail the run. It is still reported, so the staleness
    # is not forgotten the next time the Windows build is made.
    if sys.platform != 'win32':
        if exe.exists():
            notes.append('exe freshness not checked on %s (Windows-only artifact; '
                         'the Linux UI is served from web/)' % sys.platform)
            print('  info   exe present, freshness not checked on this platform')
        else:
            print('  info   exe not built (Windows-only, not needed on this platform)')
        return
    if not exe.exists():
        notes.append('webapp/exe/GPS_Localize_Viewer.exe not built yet '
                     '(run webapp/build_exe.bat)')
        print('  info   exe not built')
        return
    # The exe bundles a COPY of web/ and config/, so editing the UI without
    # rebuilding leaves the Windows demo showing the old page.
    newest = max((p.stat().st_mtime for p in (WEB.glob('*'))), default=0)
    newest = max(newest, max((p.stat().st_mtime for p in (PKG / 'config').glob('*.yaml')),
                             default=0))
    stale = exe.stat().st_mtime < newest
    check(not stale, 'exe is newer than web/ and config/ (rebuild with build_exe.bat if not)')



def check_firmware_layout():
    """lib/ is for reusable code, src/ is for this robot - and every header
    must actually be reachable and linkable.

    Written after moving the application headers out of lib/. Two failures came
    out of that move and neither showed up until a build: an include path that
    was never added, and a local library that stopped being compiled because
    the dependency finder only discovers them from what src/main.cpp includes.
    Both are cheap to check here and expensive to hit later.
    """
    section('firmware layout: lib/ vs src/')
    ini = read(FW / 'platformio.ini')

    # Application code must not live in lib/. These are the pieces that only
    # make sense for this robot.
    APP = {'state_estimator.h', 'motor_model.h', 'drive_controller.h',
           'safety_manager.h', 'power_monitor.h', 'pose_ekf.h'}
    in_lib = {p.name for p in (FW / 'lib').rglob('*.h')}
    stray = sorted(APP & in_lib)
    check(not stray, 'no robot-specific headers left in lib/ %s' % (stray or ''))

    # Every include directory referenced by build_flags must exist, and every
    # directory holding headers must be on an include path.
    inc_dirs = set(re.findall(r'-I\s+(\S+)', ini))
    missing = sorted(d for d in inc_dirs if not (FW / d).is_dir())
    check(not missing, 'every -I path in platformio.ini exists %s' % (missing or ''))

    header_dirs = {str(p.parent.relative_to(FW))
                   for p in list((FW / 'src').rglob('*.h')) + list((FW / 'lib').rglob('*.h'))}
    # lib/<name>/ is added automatically by PlatformIO; src subdirectories are not.
    need_explicit = {d for d in header_dirs if d.startswith('src/')}
    unreachable = sorted(d for d in need_explicit if d not in inc_dirs)
    check(not unreachable,
          'every src/ header directory is on an include path %s' % (unreachable or ''))

    # A local library whose .cpp must be linked has to be named in lib_deps,
    # or the header resolves and the link fails on a missing symbol.
    for lib_dir in sorted((FW / 'lib').iterdir()):
        if not lib_dir.is_dir() or not list(lib_dir.glob('*.cpp')):
            continue
        used = any(lib_dir.name.lower() in read(p).lower()
                   for p in (FW / 'src').rglob('*.h'))
        if used:
            check(re.search(r'^\s*%s\s*$' % re.escape(lib_dir.name), ini, re.M) is not None,
                  'local library %s with a .cpp is declared in lib_deps' % lib_dir.name)



def check_branch_publisher():
    """Every file publish_branches.sh claims to write, it must actually write.

    A heredoc block was once dropped by a search-and-replace whose pattern had
    a trailing space. The script still ran, still reported success, and the
    branch was published without docker-compose.windows.yml - nothing failed,
    the file was simply absent. Checking that each named output has a matching
    heredoc catches that silently-missing case.
    """
    section('branch publisher')
    script = ROOT / 'tools' / 'publish_branches.sh'
    if not script.exists():
        notes.append('tools/publish_branches.sh not present, skipped')
        return
    body = read(script)

    written = set(re.findall(r'cat > "\$wt/([^"]+)" <<', body))
    for name in sorted(written):
        # the closing marker must exist too, or the heredoc is truncated
        marker = re.search(r'cat > "\$wt/%s" <<\x27([A-Z]+)\x27' % re.escape(name), body)
        check(bool(marker) and re.search(r'^%s$' % marker.group(1), body, re.M) is not None,
              'publish_branches.sh writes %s with a closed heredoc' % name)

    for expected in ('Dockerfile', 'docker-compose.yml', 'docker-compose.windows.yml',
                     'docker-compose.serial.yml',
                     'docker-entrypoint.sh', 'install.sh', 'README.md'):
        check(expected in written,
              'docker branch gets %s' % expected)

    # every branch named in the dispatch must have a subset defined
    declared = set(re.findall(r'^\s+(workspace|firmware|docker|main|[a-z_]+)\)\s+echo "',
                              body, re.M))
    default = re.search(r'BRANCHES=\("\$\{ARGS\[@\]:-([^}]*)\}"\)', body)
    if default:
        for b in default.group(1).split():
            check(b in declared, 'branch %s has a subset defined' % b)



def check_js_syntax():
    """Parse the browser JavaScript.

    Nothing else here would notice a broken app.js: Python compiles, the
    firmware builds, QC passes, and the page is simply dead when opened. A
    real parse is the only way to catch it before the robot is in a field.
    """
    section('web UI: javascript parses')
    try:
        import esprima
    except ImportError:
        notes.append('esprima not installed, app.js not parsed '
                     '(pip install --user esprima)')
        print('  info   esprima not installed, skipped')
        return
    for js in sorted(WEB.glob('*.js')):
        try:
            esprima.parseScript(read(js), tolerant=False)
            check(True, '%s parses' % js.name)
        except Exception as exc:
            check(False, '%s: %s' % (js.name, str(exc).splitlines()[0]))



def check_compose_merge():
    """The Windows Docker override must really override.

    network_mode: host is Linux-only; on Docker Desktop the container sits in
    a VM, so "host" is that VM and the robot on your Wi-Fi can never reach the
    agent. The override has to replace it with published ports. Compose
    REPLACES sequences rather than appending, which is why devices must be
    reset explicitly - get that wrong and Windows tries to bind a serial port
    that does not exist.

    Checked by extracting both compose files from the publisher and merging
    them here, so it is verified on every run without Docker installed.
    """
    section('docker compose merge')
    script = ROOT / 'tools' / 'publish_branches.sh'
    if not script.exists():
        return
    try:
        import yaml
    except ImportError:
        notes.append('pyyaml not installed, compose merge not checked')
        print('  info   pyyaml not installed, skipped')
        return

    body = read(script)

    def heredoc(name, marker):
        m = re.search(r"cat > \"\$wt/%s\" <<'%s'\n(.*?)\n%s\n"
                      % (re.escape(name), marker, marker), body, re.S)
        return m.group(1) if m else None

    base_txt = heredoc('docker-compose.yml', 'COMPOSE')
    over_txt = heredoc('docker-compose.windows.yml', 'WINCOMPOSE')
    if not base_txt or not over_txt:
        check(False, 'both compose files are extractable from the publisher')
        return

    # Compose 2.24+ understands !reset, which is how a value is really removed
    # in an override. PyYAML does not know the tag, so teach it: for our purpose
    # the tag means "this key goes away", which we model as None.
    class ComposeLoader(yaml.SafeLoader):
        pass

    ComposeLoader.add_constructor(
        '!reset', lambda loader, node: None)
    ComposeLoader.add_constructor(
        '!override', lambda loader, node: loader.construct_sequence(node)
        if isinstance(node, yaml.SequenceNode) else loader.construct_mapping(node))

    try:
        base = yaml.load(base_txt, Loader=ComposeLoader)
        over = yaml.load(over_txt, Loader=ComposeLoader)
    except Exception as exc:
        check(False, 'compose files parse as YAML (%s)' % exc)
        return
    check(True, 'both compose files parse as YAML')

    svc_base = base['services']['gps_localize']
    check(svc_base.get('network_mode') == 'host',
          'linux uses host networking (needed for DDS discovery)')

    merged = dict(svc_base)
    merged.update(over['services']['gps_localize'])
    ports = merged.get('ports') or []
    check(merged.get('network_mode') != 'host',
          'windows override drops host networking')
    check(any('8888' in p for p in ports),
          'windows override publishes UDP 8888 for the robot')
    check(any('8080' in p for p in ports),
          'windows override publishes TCP 8080 for the browser')
    # Compose MERGES device lists rather than replacing them, so `devices: []`
    # silently leaves /dev/ttyUSB0 in place and the container will not start on
    # Windows. Only the !reset tag actually removes it. This check previously
    # asserted the merged value was [] - which passed while the real
    # `docker compose config` still emitted the device.
    # The base must declare NO devices: Docker refuses to create a container
    # whose device is missing, and /dev/ttyUSB0 does not exist unless a board
    # is plugged in. Found by actually running it - the stack would not start
    # on a machine without the robot attached, which is the normal case.
    check('devices:' not in base_txt,
          'base compose declares no devices (a missing one blocks container creation)')
    check('docker-compose.serial.yml' in body,
          'the serial port is available as an opt-in override')
    # docker exec bypasses the entrypoint, so without this an interactive
    # shell has no ros2 on PATH - which is how most people look at the graph.
    check('/root/.bashrc' in body,
          'interactive container shells source ROS (docker exec skips the entrypoint)')
    check('healthcheck' in merged, 'healthcheck survives the merge')



def check_installer():
    """The one-file installer must be exercised, not just written.

    Almost everything it does needs root, so it cannot be run for real here.
    --dry-run walks the identical decision tree and prints what it would do,
    and --check runs the detection half; both are safe and both are run. That
    is what stops the installer being the one script nobody tests until it
    breaks on somebody else's machine.
    """
    section('installer')
    script = ROOT / 'setup' / 'install.sh'
    if not script.exists():
        check(False, 'setup/install.sh exists')
        return

    rc = subprocess.run(['bash', '-n', str(script)], capture_output=True, text=True)
    check(rc.returncode == 0, 'install.sh is valid bash %s'
          % (rc.stderr.strip()[:120] if rc.returncode else ''))
    check(os.access(script, os.X_OK), 'install.sh is executable')

    for args, label in ((['--check'], '--check'),
                        (['--dry-run'], '--dry-run'),
                        (['--dry-run', '--docker'], '--dry-run --docker'),
                        (['--uninstall', '--dry-run'], '--uninstall --dry-run')):
        r = subprocess.run(['bash', str(script)] + args,
                           capture_output=True, text=True, timeout=120, cwd=str(ROOT))
        # --docker on a non-docker checkout reports a missing compose file and
        # exits non-zero on purpose, so accept that specific outcome.
        expected_fail = ('--docker' in args and
                         'not the docker branch' in (r.stdout + r.stderr))
        check(r.returncode == 0 or expected_fail,
              'install.sh %s runs cleanly' % label)
        check('would run: rm -rf /' not in r.stdout.replace(str(ROOT), 'ROOT'),
              'install.sh %s never targets a bare root path' % label)



def check_docs_current():
    """Documentation must not still describe the old behaviour.

    Every entry here is a value that changed once and left a document behind
    saying something untrue. Docs that disagree with the code are worse than no
    docs, because they get believed - so the numbers that matter are read out
    of the source and compared with what the .md files claim.
    """
    section('docs match the code')
    docs = sorted(set(ROOT.glob('*.md')) | set(ROOT.glob('docs/*.md')) |
                  set(ROOT.glob('setup/*.md')) | set(ROOT.glob('webapp/*.md')))
    text = {d: read(d) for d in docs}

    # CLAUDE.md lists superseded values on purpose - it is the record of what
    # went stale and had to be swept. Scanning it would report every example
    # it exists to document.
    def nowhere(pattern, why):
        hits = sorted(str(d.relative_to(ROOT)) for d, t in text.items()
                      if d.name != 'CLAUDE.md' and re.search(pattern, t))
        check(not hits, '%s %s' % (why, hits or ''))

    power = read(FW / 'config' / 'power.h')
    motor = read(FW / 'config' / 'motor.h')

    def const(src, name):
        m = re.search(r'#define\s+%s\s+([0-9.]+)f?' % name, src)
        return m.group(1).rstrip('.') if m else None

    # The live values, read from the firmware rather than assumed.
    cutoff = const(power, 'BATT_CUTOFF_V')
    warn_v = const(power, 'BATT_WARN_V')
    diode_a = const(motor, 'MOTOR_A_DIODE_VF_V')
    check(cutoff is not None and warn_v is not None,
          'battery thresholds are readable from power.h (%s / %s)' % (warn_v, cutoff))

    # Superseded values that must not survive anywhere in the docs.
    nowhere(r'9\.9\s*V', 'no doc still quotes the old 9.9 V cutoff')
    nowhere(r'10\.8\s*V', 'no doc still quotes the old 10.8 V warning')
    nowhere(r'0\.45f', 'no doc still quotes the old 0.45 V diode drop')
    nowhere(r'\bmor_luam\b', 'no doc still refers to mor_luam (removed)')
    nowhere(r'ros2-agent', 'no doc still uses the old ros2-agent hostname')
    nowhere(r'lib/(estimator|control|safety|power)/',
            'no doc still points at headers that moved to src/')
    # Superseded by the named Telemetry message and the EKF.
    nowhere(r'Float32MultiArray.*42 floats|42 floats',
            'no doc still describes telemetry as 42 packed floats')
    nowhere(r'Position.*currently.*plain dead reckoning',
            'no doc still says position is uncorrected dead reckoning')
    # The telemetry TYPE, wherever it is named outside code that legitimately
    # uses Float32MultiArray for /config/pid.
    for f in (ROOT / 'docs' / 'topics.md', ROOT / 'tools' / 'planhtml.py',
              ROOT / 'webapp' / 'viewer_server.py'):
        if f.exists():
            bad = [l for l in read(f).splitlines()
                   if 'telemetry' in l.lower() and 'Float32MultiArray' in l]
            check(not bad, '%s names the telemetry type correctly %s'
                  % (f.name, bad[:1] or ''))

    # The current values should actually appear where they are documented.
    if cutoff:
        found = any(cutoff in t for t in text.values())
        check(found, 'the current cutoff (%s V) is documented somewhere' % cutoff)
    if diode_a:
        found = any(diode_a in t for t in text.values())
        check(found, 'the measured diode drop (%s V) is documented somewhere' % diode_a)

    # Paths named in the docs must exist.
    missing = []
    for d, t in text.items():
        for path in set(re.findall(r'`((?:firmware|gps_localize_ws|tools|setup|webapp|docs)/[\w./-]+)`', t)):
            if path.endswith('/'):
                continue
            if not (ROOT / path).exists():
                missing.append('%s -> %s' % (d.relative_to(ROOT), path))
    check(not missing, 'every project path named in the docs exists %s'
          % (sorted(missing)[:6] or ''))



def check_telemetry_message():
    """The named Telemetry message, and the two copies of it.

    micro-ROS compiles type support into the firmware image, so the .msg has to
    exist under firmware/extra_packages as a real directory - a symlink is not
    followed by the builder, and the firmware branch has no ROS workspace to
    point at. That means two copies, which is exactly the kind of duplication
    that goes stale, so it is checked rather than trusted.

    Also checks the message and telemetry.py's FIELDS still agree. FIELDS is
    what the web UI reads by name; a field in one and not the other is a value
    that silently reads zero.
    """
    section('telemetry message')
    ws_msg = PKG.parent / 'gps_localize_msgs' / 'msg' / 'Telemetry.msg'
    fw_msg = FW / 'extra_packages' / 'gps_localize_msgs' / 'msg' / 'Telemetry.msg'

    if not ws_msg.exists():
        check(False, 'gps_localize_msgs/msg/Telemetry.msg exists')
        return
    check(True, 'Telemetry.msg exists in the workspace')

    check(fw_msg.exists(),
          'firmware/extra_packages carries the message (micro-ROS needs it '
          'compiled in, and a symlink is not followed)')
    if not fw_msg.exists():
        return
    check(not fw_msg.is_symlink(),
          'the firmware copy is a real directory, not a symlink')
    check(read(ws_msg) == read(fw_msg),
          'the two copies of Telemetry.msg are identical')

    msg_fields = re.findall(r'^float32\s+([a-z0-9_]+)', read(ws_msg), re.M)
    py = read(PKG / 'gps_localize' / 'telemetry.py')
    py_fields = re.findall(r"^\s*'([a-z0-9_]+)',", py, re.M)[:len(msg_fields)]
    check(msg_fields == py_fields,
          'Telemetry.msg and telemetry.py FIELDS agree (%d fields)' % len(msg_fields))

    # Nothing should still subscribe to telemetry as a raw array.
    stale = []
    for f in sorted((PKG / 'gps_localize').glob('*.py')):
        body = read(f)
        if re.search(r"create_subscription\(\s*Float32MultiArray[^)]*/telemetry", body):
            stale.append(f.name)
    check(not stale, 'no node still subscribes to telemetry as Float32MultiArray %s'
          % (stale or ''))



def check_nodes_import():
    """Every node must actually import.

    Syntax checks pass on a file whose imports are wrong, and the contract
    checks here only look at subscriptions - so a bulk edit that moved a name
    into the wrong package sailed through QC and only surfaced as nodes dying
    at runtime with the service reporting "active". Importing each module for
    real is the only thing that catches that.
    """
    section('nodes import cleanly')
    install = ROOT / 'gps_localize_ws' / 'install'
    if not (install / 'setup.bash').exists():
        notes.append('workspace not built, import check skipped')
        print('  info   workspace not built, skipped')
        return

    mods = sorted((PKG / 'gps_localize').glob('*.py'))
    mods = [m for m in mods if m.name != '__init__.py']
    script = (
        'import importlib, sys; '
        'sys.path.insert(0, %r); '
        'fails = []\n'
        'for name in %r:\n'   # qc: allow-windows-path
        '    try: importlib.import_module("gps_localize." + name)\n'
        '    except Exception as e: fails.append(name + ": " + type(e).__name__ + ": " + str(e))\n'
        'print("|".join(fails))\n'
    ) % (str(PKG), [m.stem for m in mods])

    r = subprocess.run(
        ['bash', '-lc',
         'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
         'source "%s/setup.bash" >/dev/null 2>&1; '
         'python3 -c %s' % (install, repr(script))],
        capture_output=True, text=True, timeout=180)
    out = (r.stdout or '').strip()
    fails = [f for f in out.split('|') if f]
    check(not fails, 'every node module imports (%d checked) %s'
          % (len(mods), fails[:4] if fails else ''))



def check_portability():
    """Nothing committed may be specific to this one machine.

    The systemd unit had User=mannaja and /home/mannaja baked in, and
    install.sh copied it verbatim - so installing on any other machine would
    have produced a service pointing at a user and a directory that do not
    exist there. It is a template with placeholders now, and this keeps it one.
    """
    section('portability: nothing machine-specific in git')
    try:
        tracked = subprocess.run(['git', 'ls-files'], cwd=ROOT,
                                 capture_output=True, text=True, timeout=60).stdout.split()
    except Exception as exc:
        notes.append('git not available, portability check skipped (%s)' % exc)
        return

    # Comments and help text may show an example address; code may not depend
    # on one. Home paths are never acceptable.
    # Exempt by nature, not by convenience:
    #   this script      carries the pattern it hunts for, in a literal
    #   plan_state.json  is a historical record; past log entries name paths
    #   *.md             documentation legitimately shows example commands
    EXEMPT = {'tools/qc.py', 'tools/plan_state.json'}
    offenders = []
    for rel in tracked:
        if rel in EXEMPT or rel.endswith('.md'):
            continue
        f = ROOT / rel
        if not f.is_file() or f.suffix in ('.png', '.pdf', '.jpg', '.zip'):
            continue
        try:
            body = f.read_text(encoding='utf-8', errors='replace')
        except OSError:
            continue
        for i, line in enumerate(body.splitlines(), 1):
            if '/home/' in line and '__PROJECT__' not in line:
                stripped = line.lstrip()
                if stripped.startswith(('#', ';', '//', '*', '<!--')):
                    continue          # documenting a path is fine
                offenders.append('%s:%d' % (rel, i))
    check(not offenders,
          'no committed file hardcodes a home directory outside a comment %s'
          % (offenders[:5] or ''))

    unit = PKG / 'systemd' / 'gps_localize.service'
    if unit.exists():
        body = read(unit)
        check('__USER__' in body and '__PROJECT__' in body,
              'the systemd unit is a template, not one machine\'s copy')
        install = ROOT / 'setup' / 'install.sh'
        if install.exists():
            check('__USER__' in read(install),
                  'install.sh substitutes the placeholders rather than copying')



def check_microros_library_fresh():
    """A built micro-ROS library must be newer than the message it compiled.

    micro_ros_platformio SKIPS its entire build when libmicroros.a already
    exists - it never checks whether the message definitions changed. So after
    editing Telemetry.msg the firmware keeps linking the OLD type support and
    fails on a missing header, or worse, links a stale layout. It cost two
    "successful" 3-second builds and a whole environment before being spotted:
    the wifi env had been regenerated, serial had not.

    The fix is always the same, so it is printed with the failure.
    """
    section('micro-ROS library is not stale')
    msg = ROOT / 'gps_localize_ws' / 'src' / 'gps_localize_msgs' / 'msg' / 'Telemetry.msg'
    if not msg.exists():
        return
    msg_mtime = msg.stat().st_mtime

    ws = pathlib.Path.home() / '.pio_workspaces' / 'GPS_Localize' / 'libdeps'
    if not ws.is_dir():
        notes.append('no PlatformIO workspace yet, staleness check skipped')
        print('  info   firmware not built here yet, skipped')
        return

    stale = []
    for env in sorted(ws.iterdir()):
        lib = env / 'micro_ros_platformio' / 'libmicroros' / 'libmicroros.a'
        if not lib.exists():
            continue                      # not built yet: nothing to be stale
        if lib.stat().st_mtime < msg_mtime:
            stale.append(env.name)
    check(not stale,
          'every built micro-ROS library is newer than Telemetry.msg %s'
          % (stale or ''))
    if stale:
        for env in stale:
            problems.append(
                'fix: rm -rf ~/.pio_workspaces/GPS_Localize/libdeps/%s/'
                'micro_ros_platformio/libmicroros && pio run -e %s' % (env, env))



def check_docker_image_fresh():
    """A built Docker image must not predate the interfaces it has to carry.

    The image is the whole product on a PC that never installs ROS, so a stale
    one is not a stale cache - it is a broken install for everyone using that
    route. The image on this machine was built before telemetry became a named
    message and simply had no gps_localize_msgs in it, while `docker images`
    reported it as present and `docker ps` would have called it healthy.

    Compared against the message definition, because that is what changed and
    what the image has to have compiled.
    """
    section('docker image is not older than the interfaces')
    # Against the newest source the image has to contain, not just the message
    # definition: the first version of this check compared to Telemetry.msg
    # alone and would have called an image fresh that was missing a crash fix
    # in web_server.py made minutes earlier.
    src_dir = ROOT / 'gps_localize_ws' / 'src'
    sources = [f for f in src_dir.rglob('*')
               if f.is_file() and f.suffix in ('.py', '.msg', '.xml', '.yaml')
               and 'build/' not in str(f) and 'install/' not in str(f)]
    if not sources:
        return
    newest = max(sources, key=lambda f: f.stat().st_mtime)
    try:
        r = subprocess.run(
            ['docker', 'image', 'inspect', 'gps_localize:latest',
             '--format', '{{.Created}}'],
            capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.SubprocessError):
        print('  info   docker not available here, skipped')
        return
    if r.returncode != 0 or not r.stdout.strip():
        notes.append('no docker image built here, freshness check skipped')
        print('  info   image not built on this machine, skipped')
        return
    out = r.stdout
    import datetime
    stamp = out.strip().split('.')[0].rstrip('Z')
    try:
        built = datetime.datetime.fromisoformat(stamp).timestamp()
    except ValueError:
        print('  info   could not parse the image timestamp, skipped')
        return
    fresh = built >= newest.stat().st_mtime
    check(fresh, 'gps_localize:latest is newer than every workspace source %s'
          % ('' if fresh else '(stale against %s)' % newest.name))
    if not fresh:
        problems.append('fix: rebuild the image from the docker branch '
                        '(git worktree add <dir> origin/docker && '
                        'docker build -t gps_localize:latest <dir>)')



def check_telemetry_consumers():
    """Nobody may reach for .data on the telemetry message.

    Telemetry is a named message now. A subscriber that still does
    unpack(msg.data) or indexes msg.data raises AttributeError inside the
    callback, which kills the node - and because the whole stack is under
    systemd, it comes back and dies again on the next frame, forever.

    This is invisible without hardware: no board means no telemetry means the
    callback never runs and every other check passes. It was found by publishing
    a synthetic Telemetry message at the running stack, which is the only way to
    exercise the path on a bench with no robot. Three nodes had it.
    """
    section('telemetry consumers use the message, not .data')
    src_dir = ROOT / 'gps_localize_ws' / 'src'
    offenders = []
    for f in sorted(src_dir.rglob('*.py')):
        if 'build/' in str(f) or 'install/' in str(f):
            continue
        body = read(f)
        if not body:
            continue
        # Two shapes, both fatal: handing .data to unpack(), and binding
        # `data = msg.data` inside a telemetry callback to index positionally.
        # The first sweep only looked for the former and left the two safety
        # nodes - the watchdog and the navigator - still crashing.
        lines = body.splitlines()
        in_telemetry_cb = False
        for n, line in enumerate(lines, 1):
            stripped = line.strip()
            if stripped.startswith('def '):
                in_telemetry_cb = 'telemetry' in stripped
            if 'unpack(' in line and '.data' in line:
                offenders.append('%s:%d' % (f.name, n))
            elif in_telemetry_cb and re.search(r'=\s*msg\.data\b', line):
                offenders.append('%s:%d' % (f.name, n))
    check(not offenders,
          'no node unpacks telemetry from .data %s' % (offenders or ''))

    # And the decoder must genuinely accept a real message, not merely look like
    # it does. Built rather than imagined, so a broken FIELDS list shows up here.
    sys.path.insert(0, str(src_dir / 'gps_localize'))
    try:
        from gps_localize import telemetry as tel
        class Fake:
            pass
        fake = Fake()
        for key in tel.FIELDS:
            setattr(fake, key, 1.5)
        out = tel.unpack(fake)
        ok = len(out) >= len(tel.FIELDS) and out.get('heading_deg') == 1.5
        check(ok, 'unpack() decodes a named message (%d fields)' % len(out))
    except Exception as exc:
        check(False, 'unpack() handles a named message (%s)' % exc)
    finally:
        sys.path.pop(0)


def main():
    main_cpp = read(FW / 'src' / 'main.cpp')
    telemetry_py = read(PKG / 'gps_localize' / 'telemetry.py')
    web_py = read(PKG / 'gps_localize' / 'web_server.py')
    viewer_py = read(ROOT / 'webapp' / 'viewer_server.py')
    py_all = '\n'.join(read(p) for p in (PKG / 'gps_localize').glob('*.py'))

    print('GPS_Localize QC  -  %s' % ROOT)
    check_nodes_import()
    check_telemetry_message()
    check_microros_library_fresh()
    check_docker_image_fresh()
    check_telemetry_consumers()
    check_telemetry(main_cpp, telemetry_py)
    check_config_selectors(main_cpp, telemetry_py, web_py)
    check_topics(main_cpp, py_all)
    check_endpoints(read(WEB / 'app.js'), web_py, viewer_py)
    check_js_syntax()
    check_web_assets(read(WEB / 'app.js'), read(WEB / 'index.html'), read(WEB / 'style.css'))
    check_design_system(read(WEB / 'style.css'), read(WEB / 'index.html'))
    check_yaml(py_all)
    check_firmware_headers(main_cpp)
    check_firmware_layout()
    check_branch_publisher()
    check_compose_merge()
    check_installer()
    check_docs_current()
    check_portability()
    check_native_tests()
    check_viewer_build()

    if notes:
        print('\nnotes (expected, not problems):')
        for note in sorted(set(notes)):
            print('  - %s' % note)

    print()
    if problems:
        print('QC FAILED - %d problem(s):' % len(problems))
        for problem in problems:
            print('  - %s' % problem)
        return 1
    print('QC PASSED - all cross-file contracts hold')
    return 0


if __name__ == '__main__':
    sys.exit(main())
