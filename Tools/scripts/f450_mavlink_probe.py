#!/usr/bin/env python3
"""Capture bench telemetry without arming or changing persistent parameters."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import serial
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM5')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', required=True)
    parser.add_argument('--set-param', action='append', default=[], metavar='NAME=VALUE',
                        help='set a persistent parameter after heartbeat and record its echo')
    parser.add_argument('--reboot-at-end', action='store_true',
                        help='request a normal FC reboot after capture')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    prefix = output / stamp
    # Suppress DTR/RTS transitions when opening the USB-UART connection.
    port = serial.Serial(port=None, baudrate=args.baud, timeout=0.05)
    port.port = args.port
    port.dtr = False
    port.rts = False
    port.open()
    mav = mavutil.mavlink.MAVLink(port, srcSystem=255, srcComponent=190)
    mav.robust_parsing = True
    counts = Counter()
    latest = {}
    ranges = {}
    statuses = []
    params = {}
    param_sets = {}
    for item in args.set_param:
        name, separator, value = item.partition('=')
        if not separator or not name or len(name) > 16:
            parser.error(f'invalid --set-param value: {item!r}')
        param_sets[name] = float(value)
    target = None
    start = time.monotonic()
    last_heartbeat = 0
    last_progress = 0
    parameter_names = [
        'FRAME_CLASS', 'FRAME_TYPE', 'SCHED_LOOP_RATE', 'MOT_PWM_TYPE',
        'RC_SPEED', 'LOG_BACKEND_TYPE', 'BATT_MONITOR', 'AHRS_ORIENTATION',
        'SERIAL0_PROTOCOL', 'SERIAL2_PROTOCOL', 'SERIAL3_PROTOCOL',
        'INS_ACC_ID', 'INS_GYR_ID', 'COMPASS_DEV_ID', 'BARO1_DEVID',
        'RCMAP_ROLL', 'RCMAP_PITCH', 'RCMAP_THROTTLE', 'RCMAP_YAW',
        'SERVO1_FUNCTION', 'SERVO2_FUNCTION', 'SERVO3_FUNCTION', 'SERVO4_FUNCTION',
    ]
    try:
        with prefix.with_suffix('.raw').open('xb') as rawlog, \
                prefix.with_suffix('.jsonl').open('x', encoding='utf-8') as log:
            while time.monotonic() - start < args.seconds:
                now = time.monotonic()
                if now - last_heartbeat >= 1:
                    mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                       0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
                    last_heartbeat = now
                data = port.read(max(1, min(port.in_waiting, 8192)))
                rawlog.write(data)
                for message in mav.parse_buffer(data) or []:
                    kind = message.get_type()
                    counts[kind] += 1
                    if kind == 'BAD_DATA':
                        continue
                    payload = message.to_dict()
                    log.write(json.dumps({'elapsed': now-start, **payload}, default=str) + '\n')
                    latest[kind] = payload
                    for key, value in payload.items():
                        if isinstance(value, (int, float)):
                            bounds = ranges.setdefault(kind + '.' + key, [value, value])
                            bounds[0] = min(bounds[0], value)
                            bounds[1] = max(bounds[1], value)
                    if kind == 'STATUSTEXT':
                        statuses.append(payload)
                        print('FC:', payload['text'], flush=True)
                    if kind == 'PARAM_VALUE':
                        params[payload['param_id']] = payload['param_value']
                    if target is None and kind == 'HEARTBEAT' and \
                            message.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                        target = (message.get_srcSystem(), message.get_srcComponent())
                        print('ArduPilot heartbeat:', target, flush=True)
                        for msgid, interval in [(30, 100000), (27, 100000), (36, 100000),
                                                (65, 100000), (29, 200000), (1, 1000000),
                                                (24, 500000), (193, 1000000), (241, 500000),
                                                (125, 1000000), (147, 1000000)]:
                            mav.command_long_send(*target, 511, 0, msgid, interval, 0, 0, 0, 0, 0)
                        mav.command_long_send(*target, 512, 0, 148, 0, 0, 0, 0, 0, 0)
                        for name in parameter_names:
                            mav.param_request_read_send(*target, name.encode('ascii'), -1)
                        for name, value in param_sets.items():
                            mav.param_set_send(*target, name.encode('ascii'), value,
                                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                if now - last_progress >= 5:
                    print('Telemetry:', dict(counts), flush=True)
                    last_progress = now
            if args.reboot_at_end and target is not None:
                mav.command_long_send(*target, 246, 0, 1, 0, 0, 0, 0, 0, 0)
                port.flush()
                time.sleep(0.5)
    finally:
        port.close()
    summary = {'port': args.port, 'baud': args.baud, 'seconds': time.monotonic()-start,
               'target': target, 'counts': dict(counts), 'latest': latest,
               'ranges': ranges, 'parameters': params, 'status_text': statuses}
    summary['requested_param_sets'] = param_sets
    summary_path = prefix.with_suffix('.summary.json')
    with summary_path.open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2, default=str)
    print('Summary:', summary_path, flush=True)
    if target is None:
        raise SystemExit('No ArduPilot heartbeat received; raw serial capture saved.')


if __name__ == '__main__':
    main()
