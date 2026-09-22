#!/usr/bin/env python3
"""
SNI transparent proxy for Quest headset.

Quest 443 traffic is redirected here by iptables. We read the TLS
ClientHello to extract the SNI hostname, then tunnel through the upstream
HTTP proxy (10.173.105.0:3129) using CONNECT with the SNI hostname (not the
possibly-DNS-poisoned IP), so the proxy resolves the real Meta IP.

Usage:
    sudo python3 sni_proxy.py [--port 31290] [--upstream 10.173.105.0:3129]
"""

import socket
import threading
import struct
import sys
import argparse

UPSTREAM = ("10.173.105.0", 3129)


def parse_sni_from_clienthello(data):
    """Parse TLS ClientHello to extract SNI hostname."""
    try:
        if len(data) < 5 or data[0] != 0x16:  # TLS handshake record
            return None
        # record: type(1) version(2) length(2) | handshake
        hs = data[5:]
        if len(hs) < 4 or hs[0] != 0x01:  # ClientHello
            return None
        # client_version(2) + random(32)
        off = 4 + 2 + 32
        if len(hs) < off + 1:
            return None
        # session_id_len
        sid_len = hs[off]
        off += 1 + sid_len
        # cipher_suites_len
        if len(hs) < off + 2:
            return None
        cs_len = struct.unpack(">H", hs[off:off+2])[0]
        off += 2 + cs_len
        # compression_methods_len
        if len(hs) < off + 1:
            return None
        cm_len = hs[off]
        off += 1 + cm_len
        # extensions
        if len(hs) < off + 2:
            return None
        ext_total = struct.unpack(">H", hs[off:off+2])[0]
        off += 2
        end = off + ext_total
        while off + 4 <= end:
            ext_type = struct.unpack(">H", hs[off:off+2])[0]
            ext_len = struct.unpack(">H", hs[off+2:off+4])[0]
            off += 4
            if ext_type == 0:  # server_name
                if off + ext_len <= len(hs):
                    sni_data = hs[off:off+ext_len]
                    # list length(2) name_type(1) name_len(2) name
                    nl = struct.unpack(">H", sni_data[3:5])[0]
                    name = sni_data[5:5+nl].decode('ascii', 'ignore')
                    return name
            off += ext_len
        return None
    except Exception:
        return None


def handle(client_sock):
    try:
        # Read TLS ClientHello (up to 4KB)
        client_sock.settimeout(10)
        data = client_sock.recv(4096)
        if not data:
            return
        sni = parse_sni_from_clienthello(data)
        if not sni:
            return
        print(f"  [SNI] {sni}", flush=True)

        # Connect to upstream proxy
        up = socket.create_connection(UPSTREAM, timeout=10)
        connect_req = f"CONNECT {sni}:443 HTTP/1.1\r\nHost: {sni}:443\r\n\r\n"
        up.sendall(connect_req.encode())
        # Read proxy response
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = up.recv(4096)
            if not chunk:
                break
            resp += chunk
        if not resp.startswith(b"HTTP/1.1 200"):
            print(f"  [ERR] proxy rejected CONNECT {sni}: {resp[:80]}", flush=True)
            return

        # Forward buffered ClientHello, then bidirectional tunnel
        if data:
            up.sendall(data)

        def pipe(src, dst):
            try:
                while True:
                    chunk = src.recv(65536)
                    if not chunk:
                        break
                    dst.sendall(chunk)
            except Exception:
                pass
            finally:
                try:
                    dst.shutdown(socket.SHUT_WR)
                except Exception:
                    pass

        t1 = threading.Thread(target=pipe, args=(client_sock, up), daemon=True)
        t2 = threading.Thread(target=pipe, args=(up, client_sock), daemon=True)
        t1.start(); t2.start()
        t1.join(timeout=300); t2.join(timeout=300)
    except Exception as e:
        print(f"  [ERR] {e}", flush=True)
    finally:
        try:
            client_sock.close()
        except Exception:
            pass


def main():
    global UPSTREAM
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=31290)
    parser.add_argument('--upstream', default='10.173.105.0:3129')
    args = parser.parse_args()
    host, _, port = args.upstream.partition(':')
    UPSTREAM = (host, int(port))

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', args.port))
    srv.listen(128)
    print(f"SNI proxy listening on :{args.port} -> upstream {UPSTREAM}", flush=True)
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=handle, args=(conn,), daemon=True).start()


if __name__ == '__main__':
    main()
