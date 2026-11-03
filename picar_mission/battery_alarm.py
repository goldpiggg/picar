"""Battery alarm state and sound preference, without importing hardware."""
import json
import math
from pathlib import Path
from threading import Event, RLock
import time
import uuid

ROOT = Path(__file__).resolve().parent.parent


class BatteryAlarm:
    def __init__(self, settings_path=None, threshold_volts=6.3, full_volts=8.4, clock=time.monotonic):
        self.path = None if settings_path is None else Path(settings_path)
        self.threshold_volts = threshold_volts
        self.full_volts = full_volts
        self.clock = clock
        self.lock = RLock()
        self.interrupt = Event()
        self.sound_enabled = True
        self.voltage = None
        self.updated_at = None
        self.sounding = False
        self.revision = 0
        self.session_id = uuid.uuid4().hex
        self.settings_error = None
        self._play = None
        self._stop = None
        if self.path is not None and self.path.exists():
            try:
                values = json.loads(self.path.read_text(encoding='utf-8'))
                if not isinstance(values, dict) or type(values.get('sound_enabled')) is not bool:
                    raise ValueError('Invalid battery sound preference')
                self.sound_enabled = values['sound_enabled']
            except (ValueError, OSError) as exc:
                self.settings_error = str(exc)

    def bind_buzzer(self, play, stop):
        with self.lock:
            self._play, self._stop = play, stop

    def _stop_locked(self):
        was_sounding = self.sounding
        try:
            if self._stop is not None:
                self._stop()
        finally:
            self.sounding = False
            if was_sounding:
                self.revision += 1

    def update_voltage(self, volts):
        if type(volts) not in (int, float) or not math.isfinite(volts) or volts <= 0:
            raise ValueError('유효한 전압 측정값이 필요합니다')
        with self.lock:
            self.voltage = float(volts)
            self.updated_at = self.clock()
            self.revision += 1
            if self.voltage >= self.threshold_volts:
                self.interrupt.set()
                if self.sounding:
                    self._stop_locked()

    def set_sound_enabled(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('경고음 켜기 또는 끄기를 선택하세요')
        with self.lock:
            if self.path is not None:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                pending = self.path.with_name(self.path.name+'.tmp')
                pending.write_text(json.dumps({'sound_enabled': enabled})+'\n', encoding='utf-8')
                pending.replace(self.path)
            self.sound_enabled = enabled
            self.revision += 1
            self.settings_error = None
            if not enabled:
                self.interrupt.set()
                self._stop_locked()
            # Enabling only changes the preference. The voltage monitor decides
            # whether a later warning cycle needs a note; there is no test beep.
            return self.snapshot()

    def play_note(self, note, duration):
        with self.lock:
            if (not self.sound_enabled or self.voltage is None or
                self.voltage >= self.threshold_volts or self._play is None):
                return False
            self.interrupt.clear()
            self._play(note)
            self.sounding = True
            self.revision += 1
        try:
            self.interrupt.wait(duration)
        finally:
            with self.lock:
                self._stop_locked()
        return True

    def snapshot(self):
        with self.lock:
            low = self.voltage is not None and self.voltage < self.threshold_volts
            # Same linear scale as Server/Voltage.py: warning threshold is 0 %, full charge 100 %.
            percent = None if self.voltage is None else round(min(100., max(0.,
                (self.voltage-self.threshold_volts)/(self.full_volts-self.threshold_volts)*100)))
            return dict(voltage_volts=None if self.voltage is None else round(self.voltage, 2),
                battery_percent=percent,
                threshold_volts=self.threshold_volts, low_voltage=low,
                sound_enabled=self.sound_enabled, muted=not self.sound_enabled,
                sounding=self.sounding, settings_error=self.settings_error,
                revision=self.revision, session_id=self.session_id,
                sample_age_seconds=None if self.updated_at is None else round(max(0., self.clock()-self.updated_at), 2))


alarm = BatteryAlarm(ROOT/'data'/'battery_alarm.json')
