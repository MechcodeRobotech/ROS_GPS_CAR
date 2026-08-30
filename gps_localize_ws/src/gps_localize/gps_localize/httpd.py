"""Tiny dependency-free HTTP + Server-Sent-Events server.

Why not rosbridge/aiohttp/flask? The customer must be able to install this on a
plain Ubuntu 22.04 box with ROS 2 Humble and nothing else, and the UI has to
work with no internet connection. Everything here is Python standard library.

Telemetry goes browser-ward over Server-Sent Events (one long-lived GET), which
is plain HTTP text - no WebSocket framing to get wrong, and the browser
reconnects on its own. Commands come back as small JSON POSTs.

MEMORY: every stream client owns one bounded queue (QUEUE_SIZE). When a client
stops reading, the oldest frames are dropped instead of piling up, and the
number of clients is capped, so RAM use is constant no matter how long the
robot runs or how badly a client misbehaves.
"""

import json
import mimetypes
import os
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

QUEUE_SIZE = 10
MAX_BODY_BYTES = 256 * 1024
SSE_IDLE_PING_S = 5.0


def lan_address():
    """The address other devices on the Wi-Fi should type.

    Picking the "first" interface is wrong on machines with VirtualBox, WSL or
    a VPN adapter - those show up first and are unreachable from a phone.
    Opening a UDP socket towards a public address makes the OS pick the
    interface that owns the default route; no packet is actually sent, so this
    works with no internet connection.
    """
    import socket
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(('8.8.8.8', 53))
        address = probe.getsockname()[0]
    except OSError:
        try:
            address = socket.gethostbyname(socket.gethostname())
        except OSError:
            address = '127.0.0.1'
    finally:
        probe.close()
    return address


# The name the web UI answers to on the local network, published by
# gps_localize_mdns.service. Hyphens, not underscores: an underscore is not
# legal in a hostname (RFC 1123), and while avahi will happily publish one,
# browsers and resolvers reject it - the name would look right everywhere
# except where it is typed.
MDNS_WEB_NAME = 'gps-robot-web.local'


def connect_info(host, port, friendly_port=None):
    """Every way to reach this web UI, best first, with a QR for the phone.

    `friendly_port` is 80 when the server managed to bind it. Port 80 is the
    only way a URL can have no port at all, which is the difference between
    something a person can be told over the phone and something they have to be
    sent. Everything keeps working on `port` regardless.
    """
    local, lan = urls_for(host, port)
    short = friendly_port == 80

    # When port 80 bound, every address drops its port - including the IP one,
    # which is what the QR carries. Showing :8080 there while the page is also
    # on 80 is not wrong, but it is longer to read back and invites the question
    # of which one is right.
    if short and lan:
        lan = 'http://%s' % lan_address()

    if lan:
        mdns = ('http://%s' % MDNS_WEB_NAME if short
                else 'http://%s:%d' % (MDNS_WEB_NAME, port))
    else:
        mdns = None

    best = mdns or lan or local

    # The QR encodes the IP, NOT the name, and the difference matters.
    #
    # A .local name needs multicast DNS, and plenty of networks drop it: guest
    # and public Wi-Fi with client isolation, corporate networks that filter
    # multicast, anything crossing a VLAN, most VPNs. On those the name simply
    # does not resolve, and a QR is scanned by someone standing there right now
    # who gets no second chance to guess why.
    #
    # An IP works wherever the two devices can reach each other at all, which is
    # a strictly weaker requirement. It goes stale when this machine changes
    # address - but a QR is a moment in time, so staleness costs nothing: scan
    # it again. The name is still the better thing to TYPE or remember, and the
    # page shows both.
    qr_url = lan or local

    return {
        'local': 'http://localhost' if short else local,
        'lan': lan,
        'mdns': mdns,
        'mdns_name': MDNS_WEB_NAME if lan else None,
        'best': best,
        'ip': lan_address() if lan else None,
        'port': port,
        'friendly_port': friendly_port,
        'qr_url': qr_url,
        'mdns_may_be_blocked': bool(mdns),
    }


def urls_for(host, port):
    """(local url, lan url) - lan is None when bound to loopback only."""
    local = 'http://localhost:%d' % port
    if host in ('127.0.0.1', 'localhost'):
        return local, None
    return local, 'http://%s:%d' % (lan_address(), port)


class SseBroadcaster:
    """Fan-out of telemetry frames to every connected browser."""

    def __init__(self, max_clients=8):
        self._clients = []
        self._lock = threading.Lock()
        self.max_clients = max_clients
        self.dropped = 0

    def register(self):
        with self._lock:
            if len(self._clients) >= self.max_clients:
                return None
            client = queue.Queue(maxsize=QUEUE_SIZE)
            self._clients.append(client)
            return client

    def unregister(self, client):
        with self._lock:
            if client in self._clients:
                self._clients.remove(client)

    def count(self):
        with self._lock:
            return len(self._clients)

    def broadcast(self, event, payload):
        """payload is any JSON-serialisable object."""
        try:
            frame = 'event: %s\ndata: %s\n\n' % (event, json.dumps(payload))
        except (TypeError, ValueError):
            return
        with self._lock:
            clients = list(self._clients)
        for client in clients:
            try:
                client.put_nowait(frame)
            except queue.Full:
                # slow client: throw the oldest frame away, keep the newest
                try:
                    client.get_nowait()
                    client.put_nowait(frame)
                    self.dropped += 1
                except (queue.Empty, queue.Full):
                    pass


class WebApp:
    """Routing table + static file root shared by all handler threads."""

    def __init__(self, static_dir, broadcaster, logger=None):
        self.static_dir = os.path.realpath(static_dir)
        self.broadcaster = broadcaster
        self.logger = logger
        self.routes = {}

    def route(self, method, path, handler):
        """handler(body_dict, query_dict) -> (http_status, response_dict)"""
        self.routes[(method.upper(), path)] = handler

    def log(self, level, message):
        if self.logger is not None:
            getattr(self.logger, level, self.logger.info)(message)


class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'gps_localize/1.0'

    # -------------------------------------------------------------- plumbing
    @property
    def app(self):
        return self.server.app

    def log_message(self, fmt, *args):      # silence the default stderr spam
        self.app.log('debug', 'http %s' % (fmt % args))

    def _send_json(self, status, payload):
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, status, content_type, body, cache=False):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'max-age=60' if cache else 'no-store')
        self.end_headers()
        self.wfile.write(body)

    # ------------------------------------------------------------------ GET
    def do_GET(self):
        parsed = urlparse(self.path)
        path = unquote(parsed.path)
        query = {k: v[0] for k, v in parse_qs(parsed.query).items()}

        if path == '/api/stream':
            self._stream()
            return

        handler = self.app.routes.get(('GET', path))
        if handler is not None:
            self._run(handler, None, query)
            return

        self._static(path)

    # ----------------------------------------------------------------- POST
    def do_POST(self):
        parsed = urlparse(self.path)
        path = unquote(parsed.path)
        query = {k: v[0] for k, v in parse_qs(parsed.query).items()}

        handler = self.app.routes.get(('POST', path))
        if handler is None:
            self._send_json(404, {'ok': False, 'error': 'unknown endpoint'})
            return

        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            length = 0
        if length > MAX_BODY_BYTES:
            self._send_json(413, {'ok': False, 'error': 'body too large'})
            return

        body = {}
        if length:
            raw = self.rfile.read(length)
            try:
                body = json.loads(raw.decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                self._send_json(400, {'ok': False, 'error': 'invalid JSON'})
                return
            if not isinstance(body, dict):
                self._send_json(400, {'ok': False, 'error': 'expected a JSON object'})
                return

        self._run(handler, body, query)

    # -------------------------------------------------------------- helpers
    def _run(self, handler, body, query):
        try:
            status, payload = handler(body, query)
        except Exception as exc:                      # never kill the server thread
            self.app.log('error', 'handler %s failed: %r' % (self.path, exc))
            self._send_json(500, {'ok': False, 'error': str(exc)})
            return
        self._send_json(status, payload)

    def _static(self, path):
        if path in ('/', ''):
            path = '/index.html'
        # Containment is checked on the normalised path, not the realpath. normpath
        # collapses any '..' before the check, so directory traversal is still
        # rejected, but a file that is a symlink is allowed to resolve outside the
        # tree. colcon's --symlink-install links every installed web asset back to
        # src/, so resolving first would make every legitimate file look like an
        # escape and return 403. Windows never hit this: it has no symlink install.
        requested = os.path.normpath(os.path.join(self.app.static_dir, path.lstrip('/')))
        if requested != self.app.static_dir and not requested.startswith(self.app.static_dir + os.sep):
            self._send_json(403, {'ok': False, 'error': 'forbidden'})
            return
        target = os.path.realpath(requested)
        if not os.path.isfile(target):
            self._send_json(404, {'ok': False, 'error': 'not found'})
            return
        ctype = mimetypes.guess_type(target)[0] or 'application/octet-stream'
        if ctype.startswith('text/') or ctype == 'application/javascript':
            ctype += '; charset=utf-8'
        try:
            with open(target, 'rb') as handle:
                body = handle.read()
        except OSError:
            self._send_json(500, {'ok': False, 'error': 'cannot read file'})
            return
        self._send_bytes(200, ctype, body)

    def _stream(self):
        client = self.app.broadcaster.register()
        if client is None:
            self._send_json(503, {'ok': False, 'error': 'too many clients'})
            return

        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache, no-store')
        self.send_header('Connection', 'keep-alive')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()

        try:
            self.wfile.write(b'retry: 2000\n\n')
            self.wfile.flush()
            while not self.server.stopping:
                try:
                    frame = client.get(timeout=SSE_IDLE_PING_S)
                except queue.Empty:
                    frame = ': ping\n\n'          # keeps proxies and NAT awake
                self.wfile.write(frame.encode('utf-8'))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                                   # browser closed the tab
        finally:
            self.app.broadcaster.unregister(client)


class WebServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, host, port, app):
        super().__init__((host, port), _Handler)
        self.app = app
        self.stopping = False
        self._thread = None

    def handle_error(self, request, client_address):
        """A browser closing a tab is normal - do not dump a traceback for it."""
        import sys
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)):
            return
        self.app.log('warn', 'request from %s failed: %r' % (client_address[0], exc))

    def start(self):
        self._thread = threading.Thread(target=self.serve_forever, kwargs={'poll_interval': 0.5},
                                        name='gps_localize_http', daemon=True)
        self._thread.start()

    def stop(self):
        self.stopping = True
        self.shutdown()
        self.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
