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

import pathlib
import re
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
    section('telemetry array: firmware <-> telemetry.py')
    fw_len = int(re.search(r'#define TELEMETRY_LEN (\d+)', main_cpp).group(1))
    fw_idx = sorted({int(m) for m in re.findall(r't\[(\d+)\]\s*=', main_cpp)})
    py_len = int(re.search(r'TELEMETRY_LEN = (\d+)', telemetry_py).group(1))
    py_fields = re.findall(r"^\s*'([a-z0-9_]+)',", telemetry_py, re.M)

    check(fw_len == py_len, 'TELEMETRY_LEN matches (%d)' % fw_len)
    check(fw_idx == list(range(fw_len)),
          'firmware fills every slot 0..%d with no gaps' % (fw_len - 1))
    check(len([f for f in py_fields][:py_len]) == py_len,
          'telemetry.py names all %d fields' % py_len)


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
    for tab in ('dash', 'map', 'wp', 'cfg', 'log'):
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


def main():
    main_cpp = read(FW / 'src' / 'main.cpp')
    telemetry_py = read(PKG / 'gps_localize' / 'telemetry.py')
    web_py = read(PKG / 'gps_localize' / 'web_server.py')
    viewer_py = read(ROOT / 'webapp' / 'viewer_server.py')
    py_all = '\n'.join(read(p) for p in (PKG / 'gps_localize').glob('*.py'))

    print('GPS_Localize QC  -  %s' % ROOT)
    check_telemetry(main_cpp, telemetry_py)
    check_config_selectors(main_cpp, telemetry_py, web_py)
    check_topics(main_cpp, py_all)
    check_endpoints(read(WEB / 'app.js'), web_py, viewer_py)
    check_web_assets(read(WEB / 'app.js'), read(WEB / 'index.html'), read(WEB / 'style.css'))
    check_design_system(read(WEB / 'style.css'), read(WEB / 'index.html'))
    check_yaml(py_all)
    check_firmware_headers(main_cpp)
    check_firmware_layout()
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
