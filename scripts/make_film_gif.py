#!/usr/bin/env python3
"""Turn the frames of scripts/run_filmer.py into a sped-up animated GIF.

    python3 scripts/make_film_gif.py outputs/film-<time>/film --output docs/media/slalom.gif

Where to run: on the host with Python 3 and Pillow (the image has no Pillow);
``./scripts/njord film`` calls it after the race. Reads frames.jsonl and the
PNG frames, skips the wait before the vessel starts moving (keeping --lead-s
seconds), and samples frames by simulation time so the GIF plays --speed
times faster than simulated time. One shared palette without dithering keeps
the file small and stops colours flickering between frames.
"""
import argparse
import json
import math
from pathlib import Path


def select_frames(records, speed, fps, lead_s=1.0, moved_m=0.5):
    """Records to show, one per GIF frame, in order.

    ``records`` are frames.jsonl entries (``sim_time_s``, ``vessel_xy``).
    Playback starts ``lead_s`` of simulation time before the vessel is first
    ``moved_m`` away from where it was filmed first, and each GIF frame
    advances ``speed / fps`` seconds of simulation time.
    """
    records = sorted((r for r in records if r.get('vessel_xy')), key=lambda r: r['sim_time_s'])
    if not records:
        raise ValueError('No frames with a vessel position')
    x0, y0 = records[0]['vessel_xy']
    start = next((r['sim_time_s'] for r in records
                  if math.hypot(r['vessel_xy'][0] - x0, r['vessel_xy'][1] - y0) > moved_m),
                 records[0]['sim_time_s'])
    step = speed / fps
    selected, t, i = [], start - lead_s, 0
    while True:
        while i < len(records) and records[i]['sim_time_s'] < t:
            i += 1
        if i == len(records):
            return selected
        selected.append(records[i])
        t = records[i]['sim_time_s'] + step


def shared_palette(sample, colors, saturated=16):
    """One palette image for all frames, built from the ``sample`` frames.

    Water and sky fill almost every pixel, so a plain median cut merges the
    small red and green buoys into muddy shades. ``saturated`` entries are
    therefore reserved for the strongly coloured pixels alone.
    """
    import numpy as np
    from PIL import Image

    pixels = np.concatenate([np.asarray(frame).reshape(-1, 3) for frame in sample])
    vivid = pixels[pixels.max(axis=1).astype(int) - pixels.min(axis=1) > 90]
    entries = []
    for part, count in ((pixels, colors - saturated), (vivid, saturated)):
        if len(part):
            quantized = Image.fromarray(part.reshape(-1, 1, 3)).quantize(count, method=Image.Quantize.MEDIANCUT)
            entries += quantized.getpalette()[:3 * len(quantized.getcolors())]
    palette = Image.new('P', (1, 1))
    palette.putpalette(entries + [0] * (768 - len(entries)))
    return palette


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('film_dir', type=Path, help='the film/ directory of a run')
    parser.add_argument('--output', type=Path, help='GIF path (default <film_dir>/race.gif)')
    parser.add_argument('--speed', type=float, default=20.0, help='simulated seconds per playback second')
    parser.add_argument('--fps', type=float, default=12.0, help='GIF frames per second')
    parser.add_argument('--width', type=int, default=640, help='GIF width in pixels')
    parser.add_argument('--colors', type=int, default=64, help='size of the shared palette')
    parser.add_argument('--hold-s', type=float, default=1.5, help='how long the last frame stays')
    args = parser.parse_args()

    from PIL import Image

    records = [json.loads(line) for line in (args.film_dir / 'frames.jsonl').read_text().splitlines() if line]
    chosen = select_frames(records, args.speed, args.fps)
    frames = []
    for record in chosen:
        image = Image.open(args.film_dir / record['file']).convert('RGB')
        height = round(image.height * args.width / image.width)
        frames.append(image.resize((args.width, height), Image.LANCZOS))
    palette = shared_palette(frames[::max(1, len(frames) // 12)], args.colors)
    frames = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    frame_ms = round(1000 / args.fps)
    durations = [frame_ms] * (len(frames) - 1) + [round(args.hold_s * 1000)]
    output = args.output or args.film_dir / 'race.gif'
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=durations, loop=0,
                   optimize=True)
    span = chosen[-1]['sim_time_s'] - chosen[0]['sim_time_s']
    print(f'wrote {output}: {len(frames)} frames, {span:.1f} s simulated at {args.speed:g}x, '
          f'{output.stat().st_size / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
