#!/usr/bin/env python3
"""
frames.py [--details] RUN.log [MORE.log ...]: how evenly the frames of a run's gestures came, and what held up the slow ones.
(More logs: the live ones of the same run, XL_FOLLOW=N XL_FOLLOW_KEEP=1, which keep what the run's log loses.)

Reads the "tl:" lines a run with boot-arg touch_lat=1 writes (see tools/latency.py for what each
says): the samples and buttons of rwtouch (tl: p), the window server's frames (tl: g: began, ended),
their hand-over (tl: s), the display driver's GPU wait and copy (tl: v), and every wait of a program
for the GPU of a millisecond or more (tl: f). Every gesture -- from a touch going down, or a button,
to the next one or 2.5 s later -- is a segment; for each:

  frames a second while frames came; the share of frame-to-frame times (as the frames reached the
  panel's memory) over 16.7 ms (counted from 18.0: a 60 Hz frame missed) and over 33 ms (from 34.5);
  with boot-arg cpuload=MS (tl: u), how busy each processor core was meanwhile, in percent;
  and for the frames over 16.7 ms, where the time between the previous frame and this one went:
    draw      the window server drawing it (its first draw to its glFlush done: CPU, Mesa included)
    gpu       the GPU after the hand-over (SwapEnd to done)
    idle      the window server not drawing at all before it began (waiting for a commit, a fence, ...)
    copy      the display driver's copy into the panel's memory

The thresholds are as the coordinator asked for them; 18.0 rather than 16.7 because a frame of a steady
60 comes 16.6 to 16.8 ms after the one before. Beside them, the share over 8.3 ms (from 9.0: a 120 Hz frame
missed), what a run at fb_hz=120 is measured by.

For Metal, include the output of `uibench catrace BACKBOARDD_PID SECONDS` with the run log. QuartzCore's
timer, vsync and render markers bound the time through hand-off. A Mach command-stream receipt before the
timer splits the preceding gap into time before an observed commit and time after it. That message can
belong to an unrelated layer tree. With no receipt in that window, the gap is reported as pre-timer.
`--details` lists each slow interval (including 120 Hz misses), its largest stage,
and overlapping client main-run-loop samples from waitstack=sb. More than one stage can contribute to a miss.
Include the `uibench run ... timeline` output to use its named motion windows and omit idle wallpaper tails.
"""
from bisect import bisect_left, bisect_right
import os
import sys

OVER_ONE, OVER_TWO, SEGMENT_US = 18000, 34500, 2500000
OVER_120 = 9000
# the first cpu-id of a small core: the loader lists the boot core, then the big ones, then the small (xnuloader/loader.c);
# on the phone with cpus=7 the Cortex-A55s are 4, 5 and 6 (FRAMES_SMALL_FROM= for another layout)
SMALL_FROM = int(os.environ.get('FRAMES_SMALL_FROM', '4'))


def metal_frames(swaps, events):
    """Use each render marker at most once, including for a swap that never made the panel."""
    events = sorted(e for e in events if e[1] in ('timer', 'vsync', 'render', 'lost'))
    timers, ticks, result = {}, {}, {}
    at = 0
    for sid, swap in sorted(swaps.items(), key=lambda item: item[1]):
        frame = None
        while at < len(events) and events[at][0] <= swap:
            ts, stage, thread, _ = events[at]
            at += 1
            if stage == 'lost':
                timers.clear()
                ticks.clear()
                frame = None
            elif stage == 'timer':
                timers[thread] = ts
            elif stage == 'vsync':
                ticks[thread] = timers.pop(thread, ts)
            else:
                frame = (ticks.pop(thread, None), ts)
        if frame is not None:
            result[sid] = frame
        timers.clear()
        ticks.clear()
    return result


def metal_stages(start, frame, commits, commit_times, loss_times):
    """Partition the interval to panel completion. Clip overlapping work at the previous present."""
    swap, ready, shown = (max(start, frame[k]) for k in ('swap', 'ready', 'shown'))
    stages = {}
    commit = None
    if frame['metal'] is None:
        stages['untraced before swap'] = swap - start
    else:
        timer, render = frame['metal']
        render = max(start, render)
        if timer is None:
            stages['untraced pre-render'] = render - start
        else:
            timer = max(start, timer)
            i = bisect_left(commit_times, start)
            if bisect_left(loss_times, start) < bisect_right(loss_times, timer):
                stages['untraced pre-timer'] = timer - start
            elif i < len(commits) and commit_times[i] <= timer:
                ts, message = commits[i]
                stages['before commit'] = ts - start
                stages['commit to timer'] = timer - ts
                commit = message
            else:
                stages['pre-timer'] = timer - start
            stages['timer to render'] = render - timer
        stages['draw to swap'] = swap - render
    stages.update(gpu=ready - swap, **{'display wait': shown - ready}, copy=frame['done'] - shown)
    return stages, commit


def show_metal_details(start, end, frames, turns, commits, commit_times, loss_times):
    began = start
    for frame in frames:
        elapsed = frame['done'] - began
        if elapsed > OVER_120:
            stages, commit = metal_stages(began, frame, commits, commit_times, loss_times)
            parts = ', '.join(f'{stage} {us / 1000:.1f}' for stage, us in stages.items() if us)
            message = '' if commit is None else f', Mach message {commit}'
            print(f'    {frame["done"] / 1e6:.6f} s swap {frame["sid"]}: {elapsed / 1000:.1f} ms, '
                  f'largest {max(stages, key=stages.get)} ({parts} ms{message})')
        began = frame['done']
    for woke, last, name, pid, cpu in turns:
        if woke < end and last >= start:
            print(f'    {name}[{pid}] {woke / 1e6:.6f}–{last / 1e6:.6f} s, {cpu / 1000:.1f} ms CPU sampled')


def lines_of(paths):
    seen, out = set(), []
    for path in paths:
      with open(path, 'rb') as f:
        for raw in f:
            text = raw.decode('utf-8', 'replace')
            at = text.find('tl: ')
            if at < 0:
                continue
            line = text[at:].strip()
            if line not in seen:
                seen.add(line)
                out.append(line)
    return out


def main():
    details = '--details' in sys.argv[1:]
    paths = [p for p in sys.argv[1:] if p != '--details']
    if not paths:
        sys.exit(__doc__)
    gestures, frames, swaps, shows, waits, loads = [], [], {}, {}, [], []
    events, turns, windows = [], [], []
    for line in lines_of(paths):
        w = line.split()
        if len(w) < 2:
            continue
        try:
            if w[1] == 'p' and len(w) >= 8 and int(w[2]) in (1, 4):
                gestures.append((int(w[7]), int(w[2]), int(w[3]), int(w[4]), int(w[5])))
            elif w[1] == 'g' and len(w) >= 8:
                # (and the core it ended on, "cpu N", where eagl.c says it: -1 where it does not)
                frames.append((int(w[4]), int(w[6]), int(w[7]), int(w[9]) if len(w) >= 10 and w[8] == 'cpu' else -1))
            elif w[1] == 's' and len(w) >= 5:
                swaps[int(w[2])] = int(w[4])
            elif w[1] == 'v' and len(w) >= 9:
                shows[int(w[2])] = (int(w[4]), int(w[6]), int(w[8]))
            elif w[1] == 'f' and len(w) >= 6:
                waits.append((int(w[5]), w[2], int(w[3])))
            elif w[1] == 'u' and len(w) >= 4:
                loads.append((int(w[3]), {int(a): int(b) for a, b in (x.split(':') for x in w[4:])}))
            elif w[1] == 'A' and len(w) == 8:
                events.append((int(w[5]), w[2], int(w[3]), int(w[7])))
            elif w[1] == 'T' and len(w) == 12:
                turns.append((int(w[5]), int(w[7]), w[2], int(w[3]), int(w[11])))
            elif w[1] == 'B' and len(w) >= 5:
                windows.append((int(w[2]), int(w[3]), ' '.join(w[4:])))
        except ValueError:
            pass
    gestures.sort()
    frames.sort(key=lambda f: f[1])
    frame_ends = [f[1] for f in frames]
    ca_frames = metal_frames(swaps, events)
    commits = sorted((t, arg) for t, stage, _, arg in events if stage == 'receive')
    commit_times = [t for t, _ in commits]
    loss_times = sorted(t for t, stage, _, _ in events if stage == 'lost')
    turns.sort()
    windows.sort()
    window_starts = [w[0] for w in windows]
    # every frame on the panel, with the drawing it came from
    timeline = []
    for sid in sorted(shows):
        if sid not in swaps:
            continue
        swap = swaps[sid]
        drawn = bisect_right(frame_ends, swap)
        if drawn and not events:
            began, ended, _, cpu = frames[drawn - 1]
        elif events:
            began = ended = swap
            cpu = -1
        elif not frames:
            began = ended = swap            # the Metal road: no "tl: g", the swap is where its frame ended
            cpu = -1
        else:
            continue
        ready, shown, done = shows[sid]
        timeline.append(dict(sid=sid, began=began, ended=ended, swap=swap, ready=ready, shown=shown, done=done,
                             cpu=cpu, metal=ca_frames.get(sid)))
    timeline.sort(key=lambda f: f['done'])
    # A swap held in SwapEnd (fb_queue=2): two before it not yet in the panel's memory, so the window server waits for
    # the GPU there. The slot comes back just before tl: v's "done"; counted to it, this is an upper bound.
    holds = []
    for sid, at in swaps.items():
        if sid - 2 in shows and sid - 1 in shows and shows[sid - 1][2] > at and shows[sid - 2][2] > at:
            holds.append((at, shows[sid - 2][2] - at))
    all_counts = [0, 0, 0]
    kinds, previous = {}, ''          # the same gesture done more than once: its frames together
    print(f'frames.py: {len(gestures)} gestures, {len(timeline)} frames on the panel, {len(waits)} waits for the GPU '
          f'of 1 ms or more{" (the Metal completion thread)" if events else ""}; '
          f'{len(holds)} swaps held in SwapEnd, {sum(h for _, h in holds) / 1000:.0f} ms at most')
    if events:
        print(f'QuartzCore: {sum(f["metal"] is not None for f in timeline)}/{len(timeline)} frames with render markers, '
              f'{len(commits)} command-stream receipts, {len(turns)} sampled client turns, '
              f'{len(loss_times)} trace loss markers')
    for i, (start, kind, button, x, y) in enumerate(gestures):
        next_gesture = gestures[i + 1][0] if i + 1 < len(gestures) else sys.maxsize
        end = min(next_gesture, start + SEGMENT_US)
        what = f'button {button}' if kind == 4 else f'touch at {x / 655.35:.0f}%,{y / 655.35:.0f}%'
        window = bisect_right(window_starts, start) - 1
        named = window >= 0 and start <= windows[window][1]
        if named:
            _, active_end, name = windows[window]
            end = min(next_gesture, active_end + 1)
            key = name
            what = f'{name} ({what})'
        else:
            key = f'{what} after {previous}' if kind == 4 else what
        chosen = [f for f in timeline if start <= f['done'] < end]
        previous = what
        if len(chosen) < 3:
            print(f'{i + 1:2}. {start / 1e6:9.3f} s {what}: {len(chosen)} frames')
            if details and events:
                show_metal_details(start, end, chosen, turns, commits, commit_times, loss_times)
            continue
        gaps = [(b['done'] - a['done'], a, b) for a, b in zip(chosen, chosen[1:])]
        moving = gaps if named else [g for g in gaps if g[0] < 100000]
        span = sum(g[0] for g in moving)
        over1 = [g for g in moving if g[0] > OVER_ONE]
        over2 = [g for g in moving if g[0] > OVER_TWO]
        over120 = [g for g in moving if g[0] > OVER_120]
        all_counts[0] += len(moving)
        all_counts[1] += len(over1)
        all_counts[2] += len(over2)
        k = kinds.setdefault(key, [0, 0, 0, 0, 0])
        k[0] += 1
        k[1] += len(moving)
        k[2] += span
        k[3] += len(over1)
        k[4] += len(over2)
        fps = len(moving) * 1e6 / span if span else 0
        line = (f'{i + 1:2}. {start / 1e6:9.3f} s {what}: {len(moving)} frames at {fps:4.1f} a second; '
                f'over 8.3 ms {100 * len(over120) / max(1, len(moving)):3.0f}%, '
                f'over 16.7 ms {100 * len(over1) / max(1, len(moving)):3.0f}%, over 33 ms {100 * len(over2) / max(1, len(moving)):3.0f}%')
        if over1 and events:
            totals = {}
            for _, a, b in over1:
                stages, _ = metal_stages(a['done'], b, commits, commit_times, loss_times)
                for stage, us in stages.items():
                    totals[stage] = totals.get(stage, 0) + us
            line += '; slow frame means: ' + ', '.join(f'{stage} {us / len(over1) / 1000:.1f} ms' for stage, us in totals.items())
        elif over1:
            # where the slow frames' time went (means over them)
            def mean(values):
                return sum(values) / len(values) / 1000 if values else 0
            gpu = mean([b['ready'] - b['swap'] for _, a, b in over1])
            copy = mean([b['done'] - b['shown'] for _, a, b in over1])
            if frames:
                draw = mean([b['ended'] - b['began'] for _, a, b in over1])
                idle = mean([b['began'] - a['ready'] for _, a, b in over1])
                line += f'; the slow ones: draw {draw:4.1f}, gpu {gpu:4.1f}, idle before {idle:5.1f}, copy {copy:3.1f} ms'
            else:
                before = mean([b['swap'] - a['ready'] for _, a, b in over1])
                line += f'; the slow ones: untraced before swap {before:4.1f}, gpu {gpu:4.1f}, copy {copy:3.1f} ms'
            known = [b for _, a, b in over1 if b['cpu'] >= 0]
            if known:
                line += f', {100 * sum(b["cpu"] >= SMALL_FROM for b in known) / len(known):.0f}% drawn on small cores'
        if over1:
            stalls = [w for w in waits if start <= w[0] < end]
            if stalls:
                # (with Metal, every tl: f is CommandQueue::waiter's, the plug-in's completion thread: nobody's drawing
                # waits there. What holds the window server's drawing back is SwapEnd, below)
                what_waits = 'completion-thread fence waits' if events else 'waits for the GPU'
                line += f'; {len(stalls)} {what_waits} ({sum(w[2] for w in stalls) / 1000:.0f} ms)'
            held = [h for t, h in holds if start <= t < end]
            line += f'; SwapEnd held {len(held)} ({sum(held) / 1000:.0f} ms at most)'
        # the load samples that end within the movement (each covers the cpuload=MS before it)
        within = [l for t, l in loads if chosen[0]['done'] < t <= chosen[-1]['done']]
        if within:
            cores = sorted({c for l in within for c in l}, reverse=True)
            line += '; cores busy ' + ' '.join(f'{c}:{sum(l.get(c, 0) for l in within) / len(within):.0f}%' for c in cores)
        print(line)
        if details and events:
            show_metal_details(start, end, chosen, turns, commits, commit_times, loss_times)
    if any(k[0] > 1 for k in kinds.values()):
        print('the same gestures together:')
        for key, (times, steps, span, over1, over2) in kinds.items():
            print(f'  {key:40} {times} times, {steps:4} frames at {steps * 1e6 / span if span else 0:4.1f} a second; '
                  f'over 16.7 ms {100 * over1 / max(1, steps):3.0f}%, over 33 ms {100 * over2 / max(1, steps):3.0f}%')
    if all_counts[0]:
        print(f'all gestures: {all_counts[0]} frame steps, over 16.7 ms {100 * all_counts[1] / all_counts[0]:.0f}%, '
              f'over 33 ms {100 * all_counts[2] / all_counts[0]:.0f}%')
    # the window server's drawing by the kind of core it ended on (eagl.c's "cpu N")
    known = [f for f in timeline if f['cpu'] >= 0]
    if known:
        small = [f['ended'] - f['began'] for f in known if f['cpu'] >= SMALL_FROM]
        big = [f['ended'] - f['began'] for f in known if f['cpu'] < SMALL_FROM]
        def avg(values):
            return sum(values) / len(values) / 1000 if values else 0
        print(f'drawing: {len(big)} frames on big cores ({avg(big):.1f} ms each), {len(small)} on small ones '
              f'({avg(small):.1f} ms each)')


if __name__ == '__main__':
    main()
