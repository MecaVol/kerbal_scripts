import csv
import time
from math import sqrt, pow
from enum import Enum
import krpc


def magnitude(vector3d):
    v = vector3d
    return sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2))


class FlightPhase(Enum):
    LAUNCHPAD = 1
    INITIAL_CLIMB = 2
    INITIAL_TURN = 3
    GRAVITY_TURN = 4
    MECO = 5
    MAIN_STAGE_SEPARATION = 6
    SECOND_ENGINE_START = 7
    SUBORBITAL_ACCELERATION = 8
    SECO = 9
    IN_SPACE_TOWARDS_CIRCULARIZATION = 10
    CIRCULARIZATION = 11
    ORBIT = 12


class Timer:
    def __init__(self, duration):
        self.start_time = time.time()
        self.duration = duration

    def elapsed(self):
        if time.time() > self.start_time + self.duration:
            return True
        return False


class Telemetry:
    """Connects to kRPC to get KSP streams of values and interesting objects such as vessel, control, flight, etc."""
    def __init__(self):
        self.client = krpc.connect()

        self.do_record = False
        self.record = [['ut', 'altitude', 'drag', 'TAS', 'mach', 'density', 'Q', 'temperature']]

        self.sc = self.client.space_center
        self.kerbin = self.sc.bodies['Kerbin']
        self.vessel = self.client.space_center.active_vessel
        self.flight = self.vessel.flight()
        self.autopilot = self.vessel.auto_pilot
        self.control = self.vessel.control

        self.ut_stream = self.client.add_stream(getattr, self.sc, 'ut')
        self.altitude_stream = self.client.add_stream(getattr, self.flight, 'mean_altitude')
        self.apoapsis_stream = self.client.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        self.drag_stream = self.client.add_stream(getattr, self.flight, 'drag')
        self.tas_stream = self.client.add_stream(getattr, self.flight, 'true_air_speed')
        self.mach_stream = self.client.add_stream(getattr, self.flight, 'mach')
        self.density_stream = self.client.add_stream(getattr, self.flight, 'atmosphere_density')
        self.dynamic_pressure_stream = self.client.add_stream(getattr, self.flight, 'dynamic_pressure')
        self.temperature_stream = self.client.add_stream(getattr, self.flight, 'static_air_temperature')

        self.altitude = 0
        self.apoapsis = 0
        self.start_time = 0
        self.elapsed = 0

    def init_time(self):
        self.start_time = self.ut_stream()

    def update(self):
        # Required for flying
        self.altitude = self.altitude_stream()
        self.apoapsis = self.apoapsis_stream()

        if self.do_record:
            # Only required for telemetry
            self.elapsed = self.ut_stream() - self.start_time
            drag = magnitude(self.drag_stream())
            tas = self.tas_stream()
            mach = self.mach_stream()
            density = self.density_stream()
            q = self.dynamic_pressure_stream()
            temperature = self.temperature_stream()
            self.record.append([self.elapsed, self.altitude, drag, tas, mach, density, q, temperature])

    def stop_recording(self):
        self.do_record = False
        self._write_atmospheric_record()

    def _write_atmospheric_record(self):
        print('Writing atmospheric telemetry report...', sep=' ')
        with open('atmospheric.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.record)
        print('Done!')


class OrbitalLaunch:
    def __init__(self):
        self.telemetry = Telemetry()
        self.phase = FlightPhase.LAUNCHPAD
        self._change_flight_law()
        self.meco_timer_1 = None
        self.meco_timer_2 = None

    @property
    def phase_name(self):
        return str(self.phase).removeprefix('FlightPhase.')

    def run(self):
        self.telemetry.init_time()
        while True:
            start_time = time.time()
            self.telemetry.control.speed_mode = self.telemetry.sc.SpeedMode.surface
            self.telemetry.update()
            altitude = self.telemetry.altitude
            apoapsis = self.telemetry.apoapsis

            if self._check_phase(altitude, apoapsis):
                self._change_flight_law()

            if self.telemetry.do_record and altitude > 70000:
                self.telemetry.stop_recording()

            end_time = time.time()
            idle_time = 0.1 - (end_time - start_time)
            if idle_time > 0:
                time.sleep(idle_time)

    def _check_phase(self, altitude, apoapsis):
        """Handles flight phase against conditions

        :return False if no change, True if it changes
        """
        if self.phase == FlightPhase.LAUNCHPAD:
            self.phase = FlightPhase.INITIAL_CLIMB
            return True
        if self.phase == FlightPhase.INITIAL_CLIMB and altitude > 100:
            self.phase = FlightPhase.INITIAL_TURN
            return True
        if self.phase == FlightPhase.INITIAL_TURN and altitude > 500:
            self.phase = FlightPhase.GRAVITY_TURN
            return True
        if self.phase == FlightPhase.GRAVITY_TURN and self.telemetry.vessel.thrust < 1:
            self.phase = FlightPhase.MECO
            self.meco_timer_1 = Timer(0.5)
            return True
        if self.phase == FlightPhase.MECO and self.meco_timer_1.elapsed():
            self.phase = FlightPhase.MAIN_STAGE_SEPARATION
            self.meco_timer_2 = Timer(0.5)
            return True
        if self.phase == FlightPhase.MAIN_STAGE_SEPARATION and self.meco_timer_2.elapsed():
            self.phase = FlightPhase.SUBORBITAL_ACCELERATION
            return True
        if self.phase == FlightPhase.SUBORBITAL_ACCELERATION and apoapsis > 75000:
            self.phase = FlightPhase.SECO
            return True
        if self.phase == FlightPhase.SECO and altitude > 70000:
            self.phase = FlightPhase.IN_SPACE_TOWARDS_CIRCULARIZATION
            return True
        return False

    def _change_flight_law(self):
        control = self.telemetry.vessel.control
        autopilot = self.telemetry.autopilot
        full_elapsed_text = str(self.telemetry.elapsed)
        dot_index = full_elapsed_text.find('.')
        elapsed_text = full_elapsed_text[0:dot_index+3]

        print(elapsed_text, ' | ', self.phase_name)

        if self.phase == FlightPhase.LAUNCHPAD:
            control.sas = False
            control.rcs = False

        if self.phase == FlightPhase.INITIAL_CLIMB:
            autopilot.engage()
            autopilot.target_pitch_and_heading(90, 90)
            control.throttle = 1
            control.activate_next_stage()

        if self.phase == FlightPhase.INITIAL_TURN:
            autopilot.target_pitch_and_heading(87, 90)

        if self.phase == FlightPhase.GRAVITY_TURN:
            autopilot.disengage()
            control.sas = True
            time.sleep(0.1)
            control.sas_mode = self.telemetry.sc.SASMode.prograde

        if self.phase == FlightPhase.MECO:
            control.throttle = 0

        if self.phase == FlightPhase.MAIN_STAGE_SEPARATION:
            control.activate_next_stage()

        if self.phase == FlightPhase.SUBORBITAL_ACCELERATION:
            control.activate_next_stage()
            control.throttle = 1

        if self.phase == FlightPhase.SECO:
            control.throttle = 0

        if self.phase == FlightPhase.IN_SPACE_TOWARDS_CIRCULARIZATION:
            pass


if __name__ == '__main__':
    launch = OrbitalLaunch()
    launch.telemetry.do_record = True
    launch.run()
