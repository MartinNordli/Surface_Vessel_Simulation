"""Independent controlled-fixture momentum balance and hydrostatic equilibrium.

This oracle is intentionally restricted to upright, zero-COM, diagonal inertia,
pure-axis motion. It fails unsupported coupled runs instead of asserting a
universal monotonic relationship. All inputs are SI; forces are body-frame about
COM. Measured telemetry is required. No real-boat fidelity is implied.
"""
import bisect
import math
from pathlib import Path
import sys
from physical_acceptance import acceptance

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.constants import GRAVITY_MPS2


def failed(reason):
    return {'passed': False, 'reason': reason, 'evidence_level': 'controlled_simulator_correctness_only'}


def wind_force(vessel, environment, u, v, yaw):
    wind = vessel['wind']
    direction = math.radians(environment['wind_direction_to_deg_enu'])-yaw
    air = [environment['wind_speed_mps']*math.cos(direction)-u,
           environment['wind_speed_mps']*math.sin(direction)-v]
    angle = math.degrees(math.atan2(air[1], air[0])) % 360
    table = sorted(wind['coefficients'], key=lambda row: row['angle_deg'])
    upper = bisect.bisect_right([row['angle_deg'] for row in table], angle)
    a, b = table[upper-1], table[upper % len(table)]
    width = (b['angle_deg']-a['angle_deg']) % 360
    fraction = ((angle-a['angle_deg']) % 360)/width
    coefficients = [a[key]+fraction*(b[key]-a[key]) for key in ('cx','cy','cn')]
    pressure = .5*1.225*sum(value*value for value in air)
    ax, ay = wind['reference_area_m2']
    return [pressure*ax*coefficients[0], pressure*ay*coefficients[1],
            pressure*ay*wind['reference_length_m']*coefficients[2]]


def compare(report, vessel, environment, axis='surge'):
    rows, wrenches = report.get('samples', []), report.get('wrench_samples', [])
    if axis not in ('surge','sway','yaw'):
        return failed('unknown axis')
    if (any(abs(v)>1e-12 for v in vessel['center_of_mass_m'])
            or any(abs(vessel['inertia_kg_m2'][key])>1e-12 for key in ('ixy','ixz','iyz'))):
        return failed('controlled fixture requires zero COM and diagonal inertia')
    if report.get('complete') is not True or len(rows)<3 or len(wrenches)<3:
        return failed('missing complete trajectory or measured wrench')
    if any(len(row)<14 or not all(math.isfinite(value) for value in row) for row in rows):
        return failed('missing full finite surge/sway/yaw acquisition samples')
    if any(abs(row[6])>.002 or abs(row[7])>.002 for row in rows):
        return failed('unsupported roll/pitch in planar fixture')
    column = {'surge':8, 'sway':12, 'yaw':4}[axis]
    if any(abs(row[col])>.01 for row in rows for col in (8,12,4) if col != column):
        return failed('unsupported coupled motion in pure-axis fixture')
    if any(b[0]<=a[0] or b[0]-a[0]>.1 for a,b in zip(rows,rows[1:])):
        return failed('missing/duplicate acquisition timestamps')
    index = {'surge':0,'sway':1,'yaw':5}[axis]
    mass = vessel['mass_kg'] if axis!='yaw' else vessel['inertia_kg_m2']['izz']
    effective = mass+vessel['hydrodynamics']['added_mass'][index]
    if any(any(key not in sample or len(sample[key])!=3 or not all(isinstance(value,(int,float)) and math.isfinite(value) for value in sample[key])
               for key in ('force_n','moment_nm')) or not isinstance(sample.get('time_s'),(int,float)) or not math.isfinite(sample['time_s'])
           for sample in wrenches):
        return failed('incomplete or nonfinite wrench telemetry')
    times = [sample['time_s'] for sample in wrenches]
    if any(b<=a for a,b in zip(times,times[1:])):
        return failed('nonadvancing wrench timestamps')
    def load(row):
        stamp = row[0]
        hi = bisect.bisect_left(times, stamp)
        if hi == 0 or hi == len(times) or times[hi]-times[hi-1] > .1:
            raise ValueError('wrench does not bracket odometry acquisition time')
        a,b = wrenches[hi-1],wrenches[hi]
        weight = (stamp-a['time_s'])/(b['time_s']-a['time_s'])
        key, component = ('moment_nm',2) if axis=='yaw' else ('force_n',index)
        thrust = a[key][component]+weight*(b[key][component]-a[key][component])
        yaw = row[13]
        current_direction = math.radians(environment['current_direction_to_deg_enu'])-yaw
        current = (environment['current_speed_mps']*(math.cos(current_direction) if axis=='surge' else math.sin(current_direction))) if axis!='yaw' else 0.
        relative = row[column]-current
        damping = vessel['hydrodynamics']['linear_damping'][index]*relative + vessel['hydrodynamics']['quadratic_damping'][index]*abs(relative)*relative
        wind = wind_force(vessel, environment, row[8],row[12],yaw)[2 if axis=='yaw' else index]
        return thrust+wind-damping
    checks=[]
    # Independent interval impulse balances include startup, steady and decay.
    # Compare mean acceleration, retaining the absolute 0.01 / 0.001 floor.
    try:
        covered = [row for row in rows if times[0]<row[0]<times[-1]]
        if not covered or covered[-1][0]-covered[0][0]<10:
            return failed('less than ten seconds bracketed momentum measurements')
        start=0
        while start<len(covered)-1:
            end=next((j for j in range(start+1,len(covered)) if covered[j][0]-covered[start][0]>=.5), None)
            if end is None:
                break
            interval=covered[start:end+1]
            impulse=sum(.5*(load(a)+load(b))*(b[0]-a[0]) for a,b in zip(interval,interval[1:]))
            duration=interval[-1][0]-interval[0][0]
            actual=(interval[-1][column]-interval[0][column])/duration
            checks.append(acceptance(actual,impulse/effective/duration,'rad/s^2' if axis=='yaw' else 'm/s^2'))
            start=end
    except (ValueError,KeyError,IndexError,TypeError) as error:
        return failed(str(error))
    return {'passed':bool(checks) and all(c['passed'] for c in checks), 'axis':axis,
            'checks':checks, 'effective_mass':effective, 'evidence_level':'controlled_simulator_correctness_only'}


def hydrostatic_equilibrium(report, vessel, environment):
    shape=vessel['geometry']['buoyancy']
    if isinstance(shape,list) or shape.get('type')!='box' or any(abs(v)>1e-12 for v in shape['pose'][3:]):
        return failed('requires one axis-aligned buoyancy box')
    metrics=report.get('metrics',{})
    if report.get('complete') is not True or any(key not in metrics for key in ('mean_z_m','mean_roll_rad','mean_pitch_rad')):
        return failed('missing settled hydrostatic pose')
    if abs(metrics['mean_roll_rad'])>.002 or abs(metrics['mean_pitch_rad'])>.002:
        return failed('upright equilibrium not established')
    length,beam,height=shape['size_m']
    draft=vessel['mass_kg']/(environment['water_density_kg_m3']*length*beam)
    if not 0<draft<height:
        return failed('invalid floating draft')
    expected=environment['water_level_m']-shape['pose'][2]+height/2-draft
    result=acceptance(metrics['mean_z_m'],expected,'m')
    result.update(expected_draft_m=draft,evidence_level='controlled_hydrostatic_correctness_only')
    return result


def oscillator_parameters(vessel, environment, axis):
    """Independent small-displacement box-waterplane restoring coefficients."""
    if axis not in ('heave', 'roll', 'pitch'):
        raise ValueError('unknown restoring axis')
    shape = vessel['geometry']['buoyancy']
    if (isinstance(shape, list) or shape.get('type') != 'box'
            or any(abs(value) > 1e-12 for value in shape['pose'][:2]+shape['pose'][3:])
            or any(abs(value) > 1e-12 for value in vessel['center_of_mass_m'])
            or any(abs(vessel['inertia_kg_m2'][key]) > 1e-12 for key in ('ixy', 'ixz', 'iyz'))):
        raise ValueError('oscillator requires centered axis-aligned box, zero COM and diagonal inertia')
    if any(abs(environment.get(key, 0.)) > 1e-12 for key in ('wind_speed_mps','current_speed_mps','wave_gain')):
        raise ValueError('oscillator requires still water without wind or waves')
    length, beam, height = shape['size_m']
    mass, density = vessel['mass_kg'], environment['water_density_kg_m3']
    draft = mass/(density*length*beam)
    if not 0 < draft < height:
        raise ValueError('box is not partially immersed')
    equilibrium = environment['water_level_m']-shape['pose'][2]+height/2-draft
    buoyancy_z = shape['pose'][2]-height/2+draft/2
    index = {'heave':2,'roll':3,'pitch':4}[axis]
    rigid = mass if axis == 'heave' else vessel['inertia_kg_m2']['ixx' if axis == 'roll' else 'iyy']
    if axis == 'heave':
        stiffness = density*GRAVITY_MPS2*length*beam
    else:
        waterplane_moment = length*beam**3/12 if axis == 'roll' else beam*length**3/12
        stiffness = density*GRAVITY_MPS2*waterplane_moment + mass*GRAVITY_MPS2*buoyancy_z
    if stiffness <= 0:
        raise ValueError('unstable equilibrium has no damped restoring acceptance')
    amplitude = min(.005, min(draft,height-draft)/10)
    if axis != 'heave':
        amplitude = min(.005, min(draft,height-draft)/(10*max(length,beam)))
    return {'axis':axis, 'effective_mass':rigid+vessel['hydrodynamics']['added_mass'][index],
            'stiffness':stiffness, 'linear_damping':vessel['hydrodynamics']['linear_damping'][index],
            'quadratic_damping':vessel['hydrodynamics']['quadratic_damping'][index],
            'equilibrium_z_m':equilibrium, 'excitation':amplitude, 'draft_m':draft,
            'assumptions':['single small-displacement axis','centered partial-immersion box',
                           'zero COM and diagonal inertia','still water, zero applied force',
                           'linear waterplane restoring, configured linear/quadratic damping']}


def oscillator(report, vessel, environment, axis):
    """Compare measured restoring decay to integrated analytic momentum balance.

    The small angle is limited by draft/freeboard, keeping the waterline within
    the box. Hydrostatic linearization is explicit; unsupported large or coupled
    motion, missing startup excitation, missing force telemetry, and missing
    decay all fail. This is simulator correctness evidence, not calibration.
    """
    try:
        parameters = oscillator_parameters(vessel, environment, axis)
    except (ValueError, KeyError, TypeError) as error:
        return failed(str(error))
    rows = report.get('samples', [])
    applied = report.get('applied_samples', [])
    if report.get('complete') is not True or len(rows) < 3 or len(applied) < 3:
        return failed('missing complete oscillator trajectory or zero-force telemetry')
    if any(len(row) < 14 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in row) for row in rows):
        return failed('missing finite oscillator pose/twist samples')
    if (rows[-1][0]-rows[0][0] < 10 or any(b[0] <= a[0] or b[0]-a[0] > .1 for a,b in zip(rows,rows[1:]))):
        return failed('oscillator requires ten contiguous acquisition seconds')
    if any(len(sample.get('forces_n',[])) != len(vessel['thrusters'])
           or not all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<.02 for v in sample['forces_n'])
           for sample in applied):
        return failed('nonzero or incomplete applied-force telemetry')
    stamps = [sample.get('time_s') for sample in applied]
    if (any(not isinstance(t,(int,float)) or not math.isfinite(t) for t in stamps)
            or stamps[0] > rows[0][0]+.1 or stamps[-1] < rows[-1][0]-.1
            or any(b<=a or b-a>.1 for a,b in zip(stamps,stamps[1:]))):
        return failed('applied-force telemetry does not cover oscillator trajectory')
    index = {'heave':5,'roll':6,'pitch':7}[axis]
    origin = parameters['equilibrium_z_m'] if axis == 'heave' else 0.
    displacements = [row[index]-origin for row in rows]
    if abs(displacements[0]) < parameters['excitation']*.3:
        return failed('startup restoring excitation not observed')
    if max(abs(value) for value in displacements) > parameters['excitation']*1.1:
        return failed('displacement exceeds preregistered small-amplitude scope')
    if max(abs(value) for value in displacements[-max(2,len(rows)//5):]) > .5*abs(displacements[0]):
        return failed('restoring decay not established')
    for row in rows:
        if abs(row[8])>.005 or abs(row[12])>.005 or abs(row[4])>.005:
            return failed('unsupported planar motion in restoring fixture')
        for column in (5,6,7):
            if column != index and abs(row[column]-(parameters['equilibrium_z_m'] if column==5 else 0.)) > .001:
                return failed('unsupported coupled restoring motion')
    def rate(row):
        roll,pitch = row[6:8]
        if axis == 'heave':
            return -math.sin(pitch)*row[8]+math.sin(roll)*math.cos(pitch)*row[12]+math.cos(roll)*math.cos(pitch)*row[9]
        if axis == 'roll':
            return row[10]+math.sin(roll)*math.tan(pitch)*row[11]+math.cos(roll)*math.tan(pitch)*row[4]
        return math.cos(roll)*row[11]-math.sin(roll)*row[4]
    def load(row):
        speed = rate(row)
        return (-parameters['stiffness']*(row[index]-origin)
                -parameters['linear_damping']*speed-parameters['quadratic_damping']*abs(speed)*speed)
    checks=[]
    start=0
    while start < len(rows)-1:
        end=next((i for i in range(start+1,len(rows)) if rows[i][0]-rows[start][0] >= .1),None)
        if end is None:
            break
        interval=rows[start:end+1]
        duration=interval[-1][0]-interval[0][0]
        impulse=sum(.5*(load(a)+load(b))*(b[0]-a[0]) for a,b in zip(interval,interval[1:]))
        observed=(rate(interval[-1])-rate(interval[0]))/duration
        checks.append(acceptance(observed,impulse/parameters['effective_mass']/duration,
                                 'm/s^2' if axis=='heave' else 'rad/s^2'))
        start=end
    return {'passed':bool(checks) and all(check['passed'] for check in checks),
            'parameters':parameters,'checks':checks,'initial_displacement':displacements[0],
            'evidence_level':'controlled_small_displacement_restoring_only'}
