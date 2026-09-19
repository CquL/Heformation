#!/usr/bin/env python3
"""Replay one recorded mission as an animation: allocation -> dispatch -> flight.

The left panel is the scene: the seven members, their flown trails, the formation
centre each task asked for and the slot layout around it, plus the declared
obstacle.  The right column scrolls the slot error and the altitude with the
replay, so the control and domain behaviour can be read off at the same instant.

The replay is driven purely by the recorded bag and metrics.json; the animation
is a viewer, not a simulation, and never writes to the experiment directory
except the file it is asked to produce.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.gridspec import GridSpec  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_mission_overview as overview  # noqa: E402

AGENTS = overview.AGENTS
COLOURS = overview.COLOURS
TRAIL_RATE_HZ = 25.0
TAIL_SECONDS = 3.0


def resample(rows, times):
    """Linearly interpolate one agent's rows onto ``times``; NaN outside its span."""
    stamps = np.array([row[0] for row in rows])
    out = []
    for column in range(1, 4):
        values = np.array([row[column] for row in rows])
        out.append(np.interp(times, stamps, values, left=np.nan, right=np.nan))
    return np.vstack(out)


def phase_at(executions, elapsed):
    for index, execution in enumerate(executions):
        if execution["actual_start"] <= elapsed <= execution["actual_finish"]:
            holding = elapsed >= execution["actual_finish"] - float(
                execution.get("model_time_hold_seconds") or 5.0)
            return "{} {}".format(execution["task_id"], "HOLD" if holding else "TRANSIT")
    if executions and elapsed > executions[-1]["actual_finish"]:
        return "MISSION COMPLETE"
    if executions and elapsed < executions[0]["actual_start"]:
        return "QUALIFICATION / READY_IDLE"
    return "IDLE"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_dir")
    parser.add_argument("--output", default=None)
    parser.add_argument("--title", default=None)
    parser.add_argument("--fps", type=float, default=8.0,
                        help="GIF frame rate; frame count is duration*speed/fps")
    parser.add_argument("--speed", type=float, default=3.0,
                        help="fast-forward factor: mission seconds per played second")
    parser.add_argument("--lead-in", type=float, default=1.5)
    parser.add_argument("--tail", type=float, default=2.5)
    parser.add_argument("--colours", type=int, default=64,
                        help="palette size for the GIF; smaller is lighter")
    parser.add_argument("--scale", type=float, default=0.52,
                        help="render scale, smaller is a lighter file")
    arguments = parser.parse_args()

    directory = Path(arguments.experiment_dir)
    output = Path(arguments.output) if arguments.output else directory / "mission_replay.gif"
    title = arguments.title or directory.name

    metrics = overview.load_json(directory / "metrics.json")
    config = overview.load_json(directory / "config.json")
    executions = metrics.get("executions", [])
    data = overview.read_bag(directory / "execution.bag")
    monitor = config["monitor"]
    monitor["start_center"] = config["runner"]["centers"][
        config["runner"].get("initial_target_ref", "start")]
    mission_epoch = metrics["mission_epoch"]

    start = mission_epoch - arguments.lead_in
    finish = mission_epoch + max(float(e["actual_finish"]) for e in executions) + arguments.tail
    times = np.arange(start, finish, arguments.speed / arguments.fps)
    elapsed = times - mission_epoch

    tail_frames = max(int(round(TAIL_SECONDS * arguments.fps / arguments.speed)), 1)
    positions = {agent: resample(data["odometry"][agent], times) for agent in AGENTS}
    centres = data["centres"]
    goal_names = [row[1] for row in data["action_goals"]]

    errors = overview.slot_errors(data, monitor, mission_epoch)
    error_arrays = {agent: (np.array([row[0] for row in errors[agent]]),
                            np.array([row[1] for row in errors[agent]]))
                    for agent in AGENTS}
    epsilon_p = float(monitor["epsilon_p"])
    obstacle = bool(metrics.get("obstacle_scenario"))

    figure = plt.figure(figsize=(13.5, 7.5))
    grid = GridSpec(2, 3, figure=figure, width_ratios=[2.05, 1.0, 1.0], hspace=0.35,
                    wspace=0.32, top=0.87, bottom=0.09, left=0.06, right=0.98)
    scene = figure.add_subplot(grid[:, 0:2])
    slot_axis = figure.add_subplot(grid[0, 2])
    altitude_axis = figure.add_subplot(grid[1, 2])

    for agent in AGENTS:
        series = errors[agent]
        slot_axis.plot([row[0] for row in series], [row[1] for row in series],
                       linewidth=0.8, alpha=0.8, color=COLOURS(agent % 10))
        rows = data["diagnostics"][agent]
        altitude_axis.plot([row["stamp"] - mission_epoch for row in rows],
                           [row["position"][2] for row in rows], linewidth=0.7,
                           alpha=0.8, color=COLOURS(agent % 10))
    slot_axis.axhline(epsilon_p, color="#c0392b", linestyle="--", linewidth=1.1)
    slot_axis.set_ylabel("slot error [m]")
    slot_axis.set_title("control: distance to slot   epsilon_p {:.2f} m".format(epsilon_p))
    slot_axis.grid(alpha=0.3)
    altitude_axis.axhline(overview.AIR_FLOOR_M, color="#c0392b", linestyle="--", linewidth=1.1,
                          label="AIR floor {:.3f} m".format(overview.AIR_FLOOR_M))
    altitude_axis.set_ylabel("altitude [m]")
    altitude_axis.set_xlabel("time since mission epoch [s]")
    altitude_axis.set_title("safety: altitude")
    altitude_axis.grid(alpha=0.3)
    altitude_axis.legend(fontsize=8, loc="lower right")
    for axis in (slot_axis, altitude_axis):
        axis.set_xlim(elapsed[0], elapsed[-1])
    slot_axis.set_ylim(0.0, max(0.6, float(slot_axis.get_ylim()[1])))

    trails = {}
    dots = {}
    for agent in AGENTS:
        series = positions[agent]
        visible = ~np.isnan(series[0])
        scene.plot(series[0][visible], series[1][visible], linewidth=1.1, alpha=0.55,
                   color=COLOURS(agent % 10), label="drone_{}".format(agent), zorder=1)
        # A bounded tail keeps the moving region small: the whole flown path is
        # already drawn statically above, so only the last few seconds repaint.
        line, = scene.plot([], [], linewidth=1.8, alpha=0.95, color=COLOURS(agent % 10),
                           zorder=4)
        trails[agent] = line
        dots[agent] = scene.scatter([], [], s=110, color=COLOURS(agent % 10),
                                    edgecolor="k", linewidth=0.8, zorder=5)
    all_x = np.concatenate([positions[a][0] for a in AGENTS])
    all_y = np.concatenate([positions[a][1] for a in AGENTS])
    margin = 2.5
    scene.set_xlim(np.nanmin(all_x) - margin, np.nanmax(all_x) + margin)
    scene.set_ylim(np.nanmin(all_y) - margin, np.nanmax(all_y) + margin)
    if obstacle:
        centre = metrics["obstacle_center"]
        size = metrics["obstacle_size"]
        scene.add_patch(Rectangle((centre[0] - size[0] / 2.0, centre[1] - size[1] / 2.0),
                                  size[0], size[1], facecolor="#c0392b", alpha=0.3,
                                  edgecolor="#c0392b", linewidth=1.5,
                                  label="declared obstacle"))
    hidden_goal = scene.scatter([], [], marker="*", s=260, color="k", zorder=6)
    slot_lines, = scene.plot([], [], linestyle="--", linewidth=1.3, color="0.35", zorder=4)
    scene.set_aspect("equal", adjustable="box")
    scene.set_xlabel("x [m]")
    scene.set_ylabel("y [m]")
    scene.grid(alpha=0.3)
    scene.legend(fontsize=7, ncol=4, loc="upper center", framealpha=0.9)

    stamp = figure.text(0.06, 0.915, "", fontsize=12, family="monospace", ha="left")
    figure.suptitle("{}: task allocation -> dispatch -> formation control".format(title),
                    fontsize=12.5, y=0.975)

    def draw(index):
        moment = times[index]
        now = elapsed[index]
        first = max(index - tail_frames, 0)
        for agent in AGENTS:
            series = positions[agent]
            window = slice(first, index + 1)
            visible = ~np.isnan(series[0][window])
            trails[agent].set_data(series[0][window][visible],
                                   series[1][window][visible])
            if visible.any():
                last = index if not np.isnan(series[0][index]) else None
                if last is None:
                    found = np.flatnonzero(~np.isnan(series[0][:index + 1]))
                    last = int(found[-1]) if found.size else None
                if last is None:
                    dots[agent].set_offsets(np.empty((0, 2)))
                else:
                    dots[agent].set_offsets([[series[0][last], series[1][last]]])
            else:
                dots[agent].set_offsets(np.empty((0, 2)))
        order = int(np.searchsorted([row[0] for row in centres], moment, side="right")) - 1
        order = min(max(order, 0), len(centres) - 1) if centres else -1
        if order >= 0:
            centre = centres[order][1:4]
            hidden_goal.set_offsets([[centre[0], centre[1]]])
            targets = [overview.slot_target(monitor, centre, agent) for agent in AGENTS]
            loop = list(range(len(AGENTS))) + [0]
            slot_lines.set_data([targets[a][0] for a in loop], [targets[a][1] for a in loop])
        for axis in (slot_axis, altitude_axis):
            for line in axis.lines:
                if line.get_label() == "__cursor":
                    line.set_xdata([now, now])
        current = []
        for agent in AGENTS:
            stamps, values = error_arrays[agent]
            index_here = int(np.searchsorted(stamps, now, side="right")) - 1
            current.append(float(values[index_here]) if index_here >= 0 else 0.0)
        goal_name = goal_names[order] if order >= 0 else "-"
        stamp.set_text("t = {:6.2f} s   {:<26} centre {}   max slot error {:.2f} m".format(
            now, phase_at(executions, now), goal_name, max(current)))
        return []

    for axis in (slot_axis, altitude_axis):
        axis.axvline(elapsed[0], color="#c0392b", linewidth=1.0, alpha=0.7,
                     label="__cursor")

    output.parent.mkdir(parents=True, exist_ok=True)

    def render(index):
        draw(index)
        figure.canvas.draw()
        raw = np.asarray(figure.canvas.buffer_rgba())[:, :, :3]
        frame = Image.fromarray(raw)
        if arguments.scale != 1.0:
            size = (max(int(frame.width * arguments.scale), 2),
                    max(int(frame.height * arguments.scale), 2))
            frame = frame.resize(size, Image.LANCZOS)
        return frame

    # One shared palette for every frame: a per-frame palette would force a local
    # colour table on each frame and defeat inter-frame differencing.
    stride = max(len(times) // 12, 1)
    samples = [render(index) for index in range(0, len(times), stride)]
    # A deliberately large swatch: PIL builds the shared palette by frequency, so
    # a thin strip of the drone hues would be squeezed out by the grey plot area.
    swatch = Image.new("RGB", (samples[0].width, 120), (255, 255, 255))
    pen = ImageDraw.Draw(swatch)
    width = samples[0].width / float(len(AGENTS))
    for position, agent in enumerate(AGENTS):
        colour = tuple(int(255 * channel) for channel in COLOURS(agent % 10)[:3])
        pen.rectangle([int(position * width), 0, int((position + 1) * width) - 1, 119],
                      fill=colour)
    montage = Image.new("RGB", (samples[0].width,
                                samples[0].height * len(samples) + swatch.height))
    for row, sample in enumerate(samples):
        montage.paste(sample, (0, row * samples[0].height))
    montage.paste(swatch, (0, samples[0].height * len(samples)))
    palette = montage.convert("P", palette=Image.ADAPTIVE, colors=arguments.colours)

    frames = []
    for index in range(len(times)):
        # No dithering: dithered noise differs per frame, which both bloats the
        # GIF and hides the inter-frame differences the encoder relies on.
        frames.append(render(index).quantize(palette=palette, dither=Image.NONE))
    plt.close(figure)
    frames[0].save(str(output), save_all=True, append_images=frames[1:], optimize=True,
                   disposal=1, duration=1000.0 / arguments.fps, loop=0)
    print("wrote {} ({} frames, {:.0f} fps, {:.1f}x speed, {} colours, scale {})".format(
        output, len(times), arguments.fps, arguments.speed, arguments.colours,
        arguments.scale))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
