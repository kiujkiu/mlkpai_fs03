#!/usr/bin/env python3
# -*- coding: ascii -*-
"""_serial_cmd.py -- serial expect (python port of _expect.ps1, board ctrl
scripts must be pure ASCII). Auto-answer login/Password, run CMD, print
output up to ___DONE___ sentinel.

usage: python _serial_cmd.py COM14 root root "systemctl restart pov"
"""
import sys
import time
import serial

port, user, pw = sys.argv[1], sys.argv[2], sys.argv[3]
cmd = sys.argv[4]
timeout_s = float(sys.argv[5]) if len(sys.argv) > 5 else 45.0

sp = serial.Serial(port, 115200, timeout=0.2)
time.sleep(0.3)
sp.reset_input_buffer()
sp.write(b'\n')
buf = ''
stage = 'login'
deadline = time.time() + timeout_s
while time.time() < deadline:
    time.sleep(0.15)
    buf += sp.read(4096).decode('utf-8', 'replace')
    if stage == 'login':
        if buf.rstrip().endswith('login:'):
            sp.write(user.encode() + b'\n'); buf = ''; continue
        if buf.rstrip().endswith('Password:'):
            sp.write(pw.encode() + b'\n'); buf = ''; stage = 'shell'; continue
        if buf.rstrip().endswith(('$', '#')):
            stage = 'ready'
    elif stage == 'shell':
        if 'Login incorrect' in buf:
            print('LOGIN_INCORRECT'); sp.close(); sys.exit(2)
        if buf.rstrip().endswith(('$', '#')):
            stage = 'ready'
    if stage == 'ready':
        # split sentinel in write so command echo can't false-match
        sp.write(cmd.encode() + b'; echo ___DO""NE___\n')
        buf = ''
        while time.time() < deadline:
            time.sleep(0.2)
            buf += sp.read(4096).decode('utf-8', 'replace')
            if '___DONE___' in buf:
                print(buf)
                sp.write(b'exit\n')          # leave no logged-in console
                sp.close(); sys.exit(0)
        break
print('TIMEOUT stage=%s tail: %s' % (stage, buf[-300:]))
sp.close()
sys.exit(1)
