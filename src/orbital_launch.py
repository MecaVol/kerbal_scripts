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
    CIRCULARIZATION = 10


class Timer:
    def __init__(self, duration):
        self.start_time = time.time()
        self.duration = duration

    def elapsed(self):
        if time.time() > self.start_time + self.duration:
            return True
        return False


class Telemetry:

    def __init__(self):
        self.client = krpc.connect()

        self.do_record = False
        self.record = [['ut', 'altitude', 'drag', 'TAS', 'mach', 'density']]

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
            self.record.append([self.elapsed, self.altitude, drag, tas, mach, density])

    def stop_recording(self):
        self.do_record = False
        self._write_record()

    def _write_record(self):
        print('Writing telemetry report...', sep=' ')
        with open('output.csv', 'w', newline='') as file:
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
        if self.phase == FlightPhase.SUBORBITAL_ACCELERATION and apoapsis > 80000:
            self.phase = FlightPhase.SECO
            return True
        return False

    def _change_flight_law(self):
        control = self.telemetry.vessel.control
        autopilot = self.telemetry.autopilot

        if self.phase == FlightPhase.LAUNCHPAD:
            control.sas = False
            control.rcs = False

        if self.phase == FlightPhase.INITIAL_CLIMB:
            print(self.telemetry.elapsed, 'Launch!')
            autopilot.engage()
            autopilot.target_pitch_and_heading(90, 90)
            control.throttle = 1
            control.activate_next_stage()

        if self.phase == FlightPhase.INITIAL_TURN:
            print(self.telemetry.elapsed, 'Initial turn!')
            autopilot.target_pitch_and_heading(87, 90)

        if self.phase == FlightPhase.GRAVITY_TURN:
            print(self.telemetry.elapsed, 'Gravity turn!')
            autopilot.disengage()
            control.sas = True
            time.sleep(0.1)
            control.sas_mode = self.telemetry.sc.SASMode.prograde

        if self.phase == FlightPhase.MECO:
            print(self.telemetry.elapsed, 'MECO!')
            control.throttle = 0

        if self.phase == FlightPhase.MAIN_STAGE_SEPARATION:
            print(self.telemetry.elapsed, 'Main stage separation!')
            control.activate_next_stage()

        if self.phase == FlightPhase.SUBORBITAL_ACCELERATION:
            print(self.telemetry.elapsed, 'Suborbital acceleration!')
            control.activate_next_stage()
            control.throttle = 1

        if self.phase == FlightPhase.SECO:
            print(self.telemetry.elapsed, 'SECO!')
            control.throttle = 0


if __name__ == '__main__':
    launch = OrbitalLaunch()
    launch.telemetry.do_record = True
    launch.run()
