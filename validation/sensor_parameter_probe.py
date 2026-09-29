#!/usr/bin/env python3
"""Measured raw/adapted sensor acceptance, never a perception truth source.

Run against a dedicated stationary sensor scene with AUTONOMY=external. Noise
mode requires 10000 paired acquisitions; rate/range modes require an overlapping
simulation-time interval. Range mode additionally requires measured returns past
--minimum-observed-range-m, with the evaluation scene supplying visible targets.
"""
import argparse
from collections import OrderedDict
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan, NavSatFix
from njord_sim.configuration import sensor_settings
from njord_sim.constants import (CAMERAS, GPS_RAW_TOPIC, GPS_TOPIC, IMU_RAW_TOPIC, IMU_TOPIC, LIDAR_SCAN_TOPIC,
                                 camera_topic)


def injected_errors(channel, raw, measured):
    if channel == 'gps':
        lat = math.radians(raw.latitude)
        e2, a = 6.69437999014e-3, 6378137.
        den = 1-e2*math.sin(lat)**2
        # north/east/up, matching the independently sampled metric channels.
        return [math.radians(measured.latitude-raw.latitude)*a*(1-e2)/den**1.5,
                math.radians(measured.longitude-raw.longitude)*a/math.sqrt(den)*math.cos(lat),
                measured.altitude-raw.altitude]
    a = np.array([raw.orientation.x,raw.orientation.y,raw.orientation.z,raw.orientation.w])
    b = np.array([measured.orientation.x,measured.orientation.y,measured.orientation.z,measured.orientation.w])
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    # q_measured * conjugate(q_raw): ENU rotation-vector error.
    vector = -b[3]*a[:3]+a[3]*b[:3]-np.cross(b[:3],a[:3])
    scalar = b[3]*a[3]+np.dot(b[:3],a[:3])
    if scalar < 0:
        vector, scalar = -vector, -scalar
    norm = np.linalg.norm(vector)
    rotation = vector*(2*math.atan2(norm,scalar)/norm) if norm else vector
    errors = list(rotation)
    for field in ('angular_velocity','linear_acceleration'):
        errors.extend(getattr(getattr(measured,field),axis)-getattr(getattr(raw,field),axis)
                      for axis in ('x','y','z'))
    return errors


def statistics(errors, stds, units):
    values = np.asarray(errors, dtype=float)
    if not len(values):
        return {'count': 0, 'passed': False, 'units': units}
    mean, std = values.mean(axis=0), values.std(axis=0)
    expected = np.asarray(stds)
    mean_ok = np.abs(mean) <= 5*expected/math.sqrt(len(values))+1e-10
    std_ok = np.abs(std-expected) <= .05*expected+1e-10
    return {'count':len(values),'mean':mean.tolist(),'stddev':std.tolist(),
            'expected_stddev':list(stds),'units':units,
            'passed':bool(np.isfinite(values).all() and mean_ok.all() and std_ok.all())}


class Probe(Node):
    def __init__(self, settings, samples, periods=None):
        super().__init__('sensor_parameter_probe', parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.settings, self.samples = settings, samples
        self.periods = periods or {}
        self.pending = {c: {'raw':OrderedDict(),'adapted':OrderedDict()} for c in ('gps','imu')}
        self.errors = {'gps':[],'imu':[]}
        self.stamps = {}
        self.invalid_headers = self.invalid_covariances = self.regressions = self.invalid_samples = 0
        self.ranges = {'count':0,'minimum_m':None,'maximum_m':None,'declared_max_m':None}
        self.dimensions = {}
        for channel, cls, topics in (
                ('gps',NavSatFix,{'raw':GPS_RAW_TOPIC,'adapted':GPS_TOPIC}),
                ('imu',Imu,{'raw':IMU_RAW_TOPIC,'adapted':IMU_TOPIC})):
            for source, topic in topics.items():
                self.create_subscription(cls,topic,lambda m,c=channel,s=source:self.pair(c,s,m),qos_profile_sensor_data)
        self.create_subscription(LaserScan,LIDAR_SCAN_TOPIC,self.scan,qos_profile_sensor_data)
        for camera, side in zip(CAMERAS, ('left','right')):
            self.create_subscription(Image,camera_topic(camera,'image_raw'),
                                     lambda m,s=side:self.camera(s,m),qos_profile_sensor_data)

    def stamp(self, stream, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        values = self.stamps.setdefault(stream,[])
        if values and stamp <= values[-1]:
            self.regressions += stamp < values[-1]
            return None
        values.append(stamp)
        return stamp

    def pair(self, channel, source, msg):
        stamp = self.stamp(channel+'_'+source,msg)
        if stamp is None:
            return
        pending = self.pending[channel]
        pending[source][stamp] = msg
        if len(pending[source]) > 20000:
            pending[source].popitem(last=False)
        if stamp not in pending['raw'] or stamp not in pending['adapted']:
            return
        raw, adapted = pending['raw'].pop(stamp), pending['adapted'].pop(stamp)
        if raw.header != adapted.header or not raw.header.frame_id:
            self.invalid_headers += 1
        if channel == 'gps':
            xy,z = self.settings['gps_horizontal_noise_m'],self.settings['gps_vertical_noise_m']
            covariances = [(adapted.position_covariance,(xy,xy,z))]
        else:
            covariances = [(getattr(adapted,field+'_covariance'),(self.settings[key],)*3)
                           for field,key in [('orientation','imu_orientation_noise_rad'),
                                             ('angular_velocity','imu_angular_velocity_noise_rad_s'),
                                             ('linear_acceleration','imu_linear_acceleration_noise_m_s2')]]
        for actual, stds in covariances:
            if not np.allclose(np.asarray(actual).reshape(3,3),np.diag(np.square(stds)),rtol=1e-10,atol=1e-14):
                self.invalid_covariances += 1
        if len(self.errors[channel]) < self.samples:
            errors = injected_errors(channel,raw,adapted)
            if np.isfinite(errors).all():
                self.errors[channel].append(errors)
            else:
                self.invalid_samples += 1

    def scan(self,msg):
        if self.stamp('lidar',msg) is None:
            return
        self.ranges['declared_max_m'] = float(msg.range_max)
        values = np.asarray(msg.ranges)
        values = values[np.isfinite(values)&(values >= msg.range_min)&(values < msg.range_max)]
        if len(values):
            self.ranges['count'] += len(values)
            low,high = float(values.min()),float(values.max())
            self.ranges['minimum_m'] = min(low,self.ranges['minimum_m']) if self.ranges['minimum_m'] is not None else low
            self.ranges['maximum_m'] = max(high,self.ranges['maximum_m']) if self.ranges['maximum_m'] is not None else high

    def camera(self,side,msg):
        self.stamp('camera_'+side,msg)
        self.dimensions[side] = [msg.width,msg.height]

    def duration(self):
        streams = ['gps_raw','gps_adapted','imu_raw','imu_adapted','lidar','camera_left','camera_right']
        if any(len(self.stamps.get(s,[])) < 2 for s in streams):
            return 0.
        return max(0.,(min(self.stamps[s][-1] for s in streams)-max(self.stamps[s][0] for s in streams))*1e-9)

    def result(self, args, complete):
        s = self.settings
        gps = statistics(self.errors['gps'],[s['gps_horizontal_noise_m']]*2+[s['gps_vertical_noise_m']],['m']*3)
        imu = statistics(self.errors['imu'],[s['imu_orientation_noise_rad']]*3+[s['imu_angular_velocity_noise_rad_s']]*3+
                         [s['imu_linear_acceleration_noise_m_s2']]*3,['rad']*3+['rad/s']*3+['m/s^2']*3)
        rates = {name:(len(t)-1)*1e9/(t[-1]-t[0]) for name,t in self.stamps.items() if len(t)>1}
        expected = {'gps_raw':s['gps_rate'],'imu_raw':s['imu_rate'],'lidar':s['lidar_rate'],
                    'camera_left':s['camera_rate'],'camera_right':s['camera_rate']}
        minimum_rates = {stream:1/self.periods[stream.split('_')[0]]
                         if stream.split('_')[0] in self.periods else requested
                         for stream,requested in expected.items()}
        # max-period is a conservative bound; simulator updates can alternate
        # quantized intervals and still average the requested frequency.
        tolerance = {stream:max(.02*requested,1/max(self.duration(),1.))
                     for stream,requested in expected.items()}
        checks = [{'name':'finite_paired_samples','passed':self.invalid_samples == 0},
                  {'name':'completed_measurement_window','passed':complete},
                  {'name':'headers_preserved','passed':self.invalid_headers == 0},
                  {'name':'covariances_match_configured_variance','passed':self.invalid_covariances == 0},
                  {'name':'acquisition_stamps_do_not_regress','passed':self.regressions == 0},
                  {'name':'configured_rates','passed':all(k in rates and minimum_rates[k]-tolerance[k] <= rates[k] <= v+tolerance[k] for k,v in expected.items())},
                  {'name':'camera_dimensions','passed':all(self.dimensions.get(side)==[s['camera_width'],s['camera_height']] for side in ('left','right'))},
                  {'name':'declared_lidar_range','passed':self.ranges['declared_max_m'] is not None and abs(self.ranges['declared_max_m']-s['lidar_range'])<1e-4}]
        if args.mode == 'noise':
            checks += [{'name':'gps_noise_statistics','passed':gps['passed'] and gps['count']>=10000},
                       {'name':'imu_noise_statistics','passed':imu['passed'] and imu['count']>=10000}]
        if args.mode == 'range':
            checks.append({'name':'observed_target_range','passed':self.ranges['maximum_m'] is not None and
                           self.ranges['maximum_m']>=args.minimum_observed_range_m})
        return {'complete':complete,'pass':all(c['passed'] for c in checks),'checks':checks,
                'evidence_level':'sensor_noise_plumbing_only','mode':args.mode,
                'metrics':{'sensor_noise_sample_count':min(gps['count'],imu['count']),
                           'gps_injected_noise':gps,'imu_injected_noise':imu,'rates_hz':rates,
                           'requested_rates_hz':expected,'minimum_rates_hz':minimum_rates,
                           'rate_tolerance_hz':tolerance,'lidar_returns':self.ranges,
                           'camera_dimensions_pixels':self.dimensions,'overlap_duration_s':self.duration()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--mode',choices=('noise','rates','range'),default='noise')
    parser.add_argument('--samples',type=int,default=10000)
    parser.add_argument('--duration-s',type=float,default=10.)
    parser.add_argument('--timeout-s',type=float,default=1800.)
    parser.add_argument('--minimum-observed-range-m',type=float)
    args = parser.parse_args()
    if args.samples < 1 or (args.mode == 'noise' and args.samples < 10000):
        parser.error('noise acceptance needs at least 10000 samples; all modes need positive samples')
    if args.mode == 'range' and args.minimum_observed_range_m is None:
        parser.error('range mode requires --minimum-observed-range-m')
    config_path = args.output_dir/'resolved_configuration.json'
    resolved = json.loads(config_path.read_text())
    rclpy.init()
    node = Probe(sensor_settings(resolved),args.samples,resolved.get('sensor_max_period_s'))
    end = time.monotonic()+args.timeout_s
    complete = False
    try:
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if (node.duration()>=args.duration_s and (args.mode != 'noise' or
                    all(len(v)>=args.samples for v in node.errors.values()))):
                complete = True
                break
        result = node.result(args,complete)
        result['resolved_configuration_sha256'] = hashlib.sha256(config_path.read_bytes()).hexdigest()
        manifest = args.output_dir/'run_manifest.json'
        result['run_manifest_sha256'] = hashlib.sha256(manifest.read_bytes()).hexdigest() if manifest.is_file() else None
        result['limitations'] = ['No marine fidelity or collision-free completion claim.',
                                'Range extent alone does not identify a target; use a controlled evaluation scene.']
        (args.output_dir/'sensor_parameter_metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(json.dumps(result,allow_nan=False))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
