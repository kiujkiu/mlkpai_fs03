import socket, time, os, sys
host = sys.argv[1]
c = socket.create_connection((host, 9600), timeout=10)
c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
buf = os.urandom(262144)
t0 = time.time(); n = 0
while time.time() - t0 < 10:
    c.sendall(buf); n += len(buf)
c.close()
dt = time.time() - t0
print('TX %.2f MB in %.2fs = %.2f MB/s (%.0f Mbps)' % (n/1e6, dt, n/dt/1e6, n/dt*8/1e6))
