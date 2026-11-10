# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Runs ardurover next to the relay and starts it again if it exits.

The working directory is the app's data directory: ArduPilot keeps its parameter storage and logs
there and reads the robot from APM/nnm/<robot>/ under it. A file site.parm in that directory adds
parameter defaults on top of the ones built into the binary.
"""
from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path


class Supervisor:
    def __init__(self, app_dir: Path, bus='127.0.0.1:9003', gcs='127.0.0.1:14550', ground_station_port=5760):
        self.binary = app_dir / 'bin' / 'ardurover'
        self.data = app_dir / 'data'
        self.bus, self.gcs = bus, gcs
        self.ground_station_port = ground_station_port      # MAVLink over TCP for Mission Planner, QGC, MAVProxy
        self.proc = None
        self.starts = 0
        self.last_exit = None
        self.error = None
        self._stop = threading.Event()

    def command(self):
        # serial0: the web gateway. serial1: a ground station (TCP server, one client). serial2: the joint bus.
        cmd = [str(self.binary), '--serial0', f'udp:{self.gcs}', '--serial2', f'udp:{self.bus}']
        if self.ground_station_port:
            cmd += ['--serial1', f'tcp:0.0.0.0:{self.ground_station_port}']
        site = self.data / 'site.parm'
        if site.exists():
            cmd += ['--defaults', f'@ROMFS/defaults.parm,{site}']
        return cmd

    def run(self):
        backoff = 1.0
        while not self._stop.is_set():
            if not self.binary.exists():
                self.error = f'{self.binary} is missing: build it with unoq/build_ardurover.sh'
                time.sleep(5.0)
                continue
            try:
                os.chmod(self.binary, 0o755)
            except OSError:
                pass
            self.data.mkdir(parents=True, exist_ok=True)
            log = open(self.data / 'ardurover.log', 'ab')
            started = time.monotonic()
            try:
                self.proc = subprocess.Popen(self.command(), cwd=self.data, stdout=log, stderr=subprocess.STDOUT,
                                             stdin=subprocess.DEVNULL)
            except OSError as e:
                self.error = f'cannot start ardurover: {e}'
                log.close()
                time.sleep(5.0)
                continue
            self.starts += 1
            self.error = None
            self.proc.wait()
            self.last_exit = self.proc.returncode
            log.close()
            if self._stop.is_set():
                break
            backoff = 1.0 if time.monotonic() - started > 30.0 else min(backoff * 2.0, 15.0)
            self.error = f'ardurover exited with code {self.last_exit}, restarting in {backoff:.0f} s'
            time.sleep(backoff)

    def start(self):
        threading.Thread(target=self.run, name='ardurover', daemon=True).start()
        return self

    def stop(self):
        self._stop.set()
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def status(self) -> dict:
        return dict(running=self.proc is not None and self.proc.poll() is None, starts=self.starts,
                    last_exit=self.last_exit, error=self.error, mavlink_tcp_port=self.ground_station_port)
