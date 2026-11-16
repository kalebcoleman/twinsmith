"use strict";
// The step viewer: panels over one run. The run is window.RUN, written by scripts/puzzle_viewer.py.
//
// A panel is {title, mount(body, run, app), update(state)}. Viewer.panel(id, panel) adds a kind of panel; run.panels
// lists which to show, in order, so a page for another purpose shows other panels over the same run. To drive the
// page live, call Viewer.load(run) again whenever the run has grown: every panel is mounted afresh.
//
// state: {t, active, selected, tryName, tryT}: the film's time, the step running then, the step picked in the list,
// and the lookahead try on show with its own time.

const PALETTE = ["#2563eb", "#d97706", "#059669", "#dc2626", "#7c3aed", "#0891b2", "#be185d", "#65a30d"];
const SHARE = {physics: "#2563eb", rendering: "#d97706", bridge: "#7c3aed", sam3: "#059669", scan: "#0891b2", planning: "#be185d", other: "#9aa3b0"};

function el(tag, attrs, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs || {})) {
    if (key === "class") node.className = value;
    else if (key.startsWith("on")) node.addEventListener(key.slice(2), value);
    else if (value !== null && value !== undefined) node.setAttribute(key, value);
  }
  for (const child of children.flat()) if (child !== null && child !== undefined) node.append(child);
  return node;
}
const fix = (value, digits = 1) => (value === null || value === undefined ? "" : Number(value).toFixed(digits));
const ink = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

function outcome(step) {
  const result = step.result || {};
  if (step.error) return "failed: " + step.error;
  if (step.kind === "checkpoint") return step.passed ? "passed" : "not passed";
  if (step.kind === "lookahead") return `${result.verdict} after ${result.tries.length} ${result.tries.length === 1 ? "try" : "tries"}`;
  if (step.kind === "try") {
    if (result.go) return `go: seated ${fix(result.position_error_mm, 2)} mm from the slot`;
    return "no: " + (result.error ? `${result.error} (${result.during})` : `${fix(result.position_error_mm, 1)} mm off, others moved ${fix(result.others_moved_mm, 1)} mm`);
  }
  if (step.kind === "survey") return `found ${result.found}, missing ${result.missing.length}`;
  if (step.kind === "close" || step.kind === "grip") return `pads ${fix(result.gap_m * 1000, 1)} mm apart`;
  if (step.kind === "lift") return `rose ${fix(result.rise_m * 1000, 0)} mm`;
  if (step.kind === "lower" || step.kind === "insert") return result.blocked ? "blocked: " + result.blocked : "reached";
  if (step.kind === "rescan" || step.kind === "place_base") return [result.result, result.position_error_mm !== undefined && result.position_error_mm !== null ? fix(result.position_error_mm, 2) + " mm" : ""].join(" ");
  return "";
}

function chart(canvas, series, options) {
  // Lines over time with a cursor: series = [{label, color, points: [[t, value]], step}], options = {t, band, height}.
  const ratio = window.devicePixelRatio || 1, width = canvas.clientWidth, height = options.height || 150;
  canvas.width = width * ratio; canvas.height = height * ratio; canvas.style.height = height + "px";
  const pen = canvas.getContext("2d");
  pen.scale(ratio, ratio);
  const values = series.flatMap((one) => one.points.map((point) => point[1]));
  const times = series.flatMap((one) => one.points.map((point) => point[0]));
  if (!values.length) return;
  let low = Math.min(...values), high = Math.max(...values);
  if (high - low < 1e-6) { low -= 1; high += 1; }
  const t0 = Math.min(...times), t1 = Math.max(...times), left = 44, bottom = height - 16;
  const x = (t) => left + (width - left - 6) * (t - t0) / Math.max(1e-6, t1 - t0);
  const y = (value) => 6 + (bottom - 6) * (1 - (value - low) / (high - low));
  if (options.band) { pen.fillStyle = ink("--band"); pen.fillRect(x(options.band[0]), 0, Math.max(1, x(options.band[1]) - x(options.band[0])), bottom); }
  pen.strokeStyle = ink("--line"); pen.fillStyle = ink("--soft"); pen.font = "11px system-ui"; pen.lineWidth = 1;
  for (const value of [low, (low + high) / 2, high]) {
    pen.beginPath(); pen.moveTo(left, y(value)); pen.lineTo(width - 6, y(value)); pen.stroke();
    pen.fillText(fix(value, Math.abs(high - low) < 10 ? 1 : 0), 2, y(value) + 4);
  }
  pen.fillText(fix(t0, 0) + " s", left, height - 3); pen.fillText(fix(t1, 0) + " s", width - 40, height - 3);
  for (const one of series) {
    pen.strokeStyle = one.color; pen.lineWidth = 1.5; pen.beginPath();
    one.points.forEach((point, index) => {
      if (index && one.step) pen.lineTo(x(point[0]), y(one.points[index - 1][1]));
      if (index) pen.lineTo(x(point[0]), y(point[1])); else pen.moveTo(x(point[0]), y(point[1]));
    });
    if (one.step && one.points.length) pen.lineTo(x(t1), y(one.points[one.points.length - 1][1]));
    pen.stroke();
  }
  pen.strokeStyle = ink("--ink"); pen.lineWidth = 1; pen.beginPath(); pen.moveTo(x(options.t), 0); pen.lineTo(x(options.t), bottom); pen.stroke();
}

const legend = (items) => el("div", {class: "legend"}, items.map(([label, color]) => el("span", {}, el("i", {style: `background:${color}`}), label)));

const Viewer = {
  kinds: {},
  panel(id, panel) { this.kinds[id] = panel; },

  load(run) {
    const host = document.getElementById("panels");
    host.replaceChildren();
    const real = run.steps.filter((step) => step.try === null);
    const checks = run.checkpoints.map((check, index) => ({id: "c" + index, kind: "checkpoint", piece: check.name, depth: 1, try: null,
      t0: check.t, t1: check.t, passed: check.passed, plan: {}, spent: {}, subgraph: check.subgraph, after: check.after}));
    const app = {
      run, real, checks, videos: [], panels: [], last: 0, queue: [], wanted: null,
      state: {t: 0, active: null, selected: null, tryName: null, tryT: 0},
      step(id) { return typeof id === "string" ? checks.find((check) => check.id === id) : run.steps[id]; },
      seek(t) {
        app.wanted = Math.max(0, Math.min(run.duration, t));             // kept until the film is there: it may not have loaded yet
        for (const video of app.videos) video.currentTime = app.wanted;
        for (const step of real) if (step.kind === "lookahead" && step.t0 > t) step.shown = false;
        app.last = t;
      },
      playing() { return app.videos.length && !app.videos[0].paused; },
      play(on) { for (const video of app.videos) on ? video.play() : video.pause(); },
      select(id) {
        app.state.selected = id;
        const step = app.step(id);
        if (step.try) {                                     // a step of a lookahead try: the build waits where that lookahead was made
          let ahead = step;
          while (ahead.kind !== "lookahead") ahead = run.steps[ahead.parent];
          app.seek(ahead.t0);
          app.showTry(step.try, step.kind === "try" ? 0 : step.t0);
        } else { app.endTry(); app.seek(step.t0); }
        app.refresh();
      },
      showTry(name, from, then) { app.play(false); app.future.show(name, from || 0, then); app.state.tryName = name; },
      endTry() { app.queue = []; app.state.tryName = null; if (app.future) app.future.stop(); },
      refresh() { for (const panel of app.panels) panel.update(app.state); },
    };
    document.getElementById("title").textContent = `Puzzle build, ${run.title}`;
    const badge = document.getElementById("badge");
    badge.textContent = `${run.summary.completed ? "completed" : "not completed"} · ${fix(run.summary.simulated_seconds, 0)} s simulated · ${fix(run.summary.wall_seconds / 60, 0)} min wall · perception: ${run.summary.perception}`;
    badge.className = run.summary.completed ? "ok" : "bad";
    for (const id of run.panels) {
      const kind = this.kinds[id];
      const body = el("div", {});
      host.append(el("section", {class: "panel", id: "panel-" + id}, el("h2", {}, kind.title), body));
      const panel = Object.create(kind);
      panel.mount(body, run, app);
      app.panels.push(panel);
    }
    const scrub = document.getElementById("scrub"), speed = document.getElementById("speed"), ahead = document.getElementById("ahead");
    scrub.max = run.duration;
    scrub.oninput = () => { app.endTry(); app.seek(Number(scrub.value)); };
    const rate = () => { for (const video of document.querySelectorAll("video")) video.playbackRate = Number(speed.value); };
    speed.onchange = rate;
    document.getElementById("play").onclick = () => { if (app.state.tryName) app.endTry(); app.play(!app.playing()); };
    const tick = () => {
      const state = app.state, lead = app.videos[0];
      if (app.wanted !== null && lead && lead.readyState > 0 && Math.abs(lead.currentTime - app.wanted) < 0.5) app.wanted = null;
      const t = app.wanted !== null ? app.wanted : lead ? lead.currentTime : 0;
      for (const video of app.videos.slice(1)) if (video.readyState > 0 && Math.abs(video.currentTime - t) > 0.25) video.currentTime = t;
      if (app.playing() && ahead.checked && !state.tryName) {                   // the future first: play a lookahead's tries before its step
        const due = real.find((step) => step.kind === "lookahead" && !step.shown && app.last <= step.t0 + 1e-6 && t >= step.t0 && t - step.t0 < 2);
        if (due) {
          due.shown = true;
          app.queue = run.steps.filter((step) => step.kind === "try" && step.parent === due.id).map((step) => step.plan.clip);
          const next = () => { const name = app.queue.shift(); if (name) app.showTry(name, 0, next); else { app.endTry(); app.play(true); } };
          state.selected = due.id;
          next();
        }
      }
      state.tryT = app.future ? app.future.time() : 0;
      state.t = t; app.last = t;
      const begun = real.filter((step) => step.t0 <= t + 1e-6), within = begun.filter((step) => t < step.t1);
      state.active = within.length ? within[within.length - 1] : begun.length ? begun[begun.length - 1] : null;      // the innermost step running, else the last one begun
      const now = [t, state.tryName, fix(state.tryT, 1), state.selected, state.active && state.active.id, app.playing()].join("|");
      if (app.shown !== now) {
        app.shown = now;
        scrub.value = t;
        document.getElementById("clock").textContent = `${fix(t, 1)} / ${fix(run.duration, 1)} s`;
        document.getElementById("play").textContent = app.playing() ? "Pause" : "Play";
        rate();
        app.refresh();
      }
      requestAnimationFrame(tick);
    };
    const want = new URLSearchParams(location.hash.slice(1));                    // index.html#step=12 or #t=95.5 opens there
    if (want.has("step")) app.select(/^c/.test(want.get("step")) ? want.get("step") : Number(want.get("step")));
    else if (want.has("t")) app.seek(Number(want.get("t")));
    app.refresh();
    requestAnimationFrame(tick);
    return app;
  },
};

Viewer.panel("video", {
  title: "The build: table view and wrist D415",
  mount(body, run, app) {
    this.caption = el("div", {class: "caption"});
    const views = run.views.map((view) => {
      const video = el("video", {src: view.src, muted: "", playsinline: "", preload: "auto"});
      video.muted = true;
      video.addEventListener("loadedmetadata", () => { if (app.wanted !== null) video.currentTime = app.wanted; });
      app.videos.push(video);
      return el("figure", {}, video, el("figcaption", {}, view.name === "wrist" ? "wrist D415 (what the robot sees)" : view.name + " camera (for the film only)"));
    });
    body.append(el("div", {class: "views"}, views), this.caption);
    this.run = run;
  },
  update(state) { this.caption.textContent = this.run.captions[Math.min(this.run.captions.length - 1, Math.floor(state.t * this.run.fps))] || ""; },
});

Viewer.panel("future", {
  title: "Future: the Newton lookahead",
  mount(body, run, app) {
    const first = run.steps.find((one) => one.kind === "try");
    this.video = el("video", {muted: "", playsinline: "", preload: "metadata", src: first ? run.tries[first.plan.clip].src : null});
    this.video.muted = true;
    this.label = el("div", {class: "caption"}, "Each try is simulated ahead before the real pick; the clip plays here first.");
    this.chips = el("div", {class: "chips"});
    this.app = app; this.run = run;
    for (const step of run.steps.filter((one) => one.kind === "try")) {
      const chip = el("span", {class: "chip pick " + (step.result && step.result.go ? "ok" : "bad"), "data-clip": step.plan.clip,
        onclick: () => app.select(step.id)}, `${step.piece} ${step.plan.clip.split("_").pop()}`);
      this.chips.append(chip);
    }
    body.append(this.video, this.label, this.chips, el("div", {class: "note"}, "Green: the try said go. Red: it did not, and the next try (or an abort) followed."));
    app.future = this;
  },
  show(name, from, then) {
    const step = this.run.steps.find((one) => one.kind === "try" && one.plan.clip === name);
    this.video.src = this.run.tries[name].src;
    this.video.currentTime = from;
    this.video.onended = () => { if (then) then(); };
    this.video.play();
    this.label.textContent = `${step.piece}, try ${name.split("_").pop()}: ${outcome(step)}`;
    this.label.className = "caption " + (step.result && step.result.go ? "ok" : "bad");
  },
  stop() { this.video.pause(); this.video.onended = null; },
  time() { return this.video.currentTime || 0; },
  update(state) { for (const chip of this.chips.children) chip.classList.toggle("on", chip.dataset.clip === state.tryName); },
});

Viewer.panel("steps", {
  title: "Steps",
  mount(body, run, app) {
    this.app = app;
    this.rows = [...run.steps];                             // the log's own order, with each checkpoint after the step that ended its subgraph
    for (const check of app.checks) {
      let at = this.rows.findIndex((step) => step.id === check.after) + 1;
      while (at < this.rows.length && (this.rows[at].depth > 0 || this.rows[at].try !== null) && this.rows[at].kind !== "checkpoint") at++;
      this.rows.splice(at, 0, check);
    }
    this.body = el("tbody", {id: "steps"});
    for (const step of this.rows) {
      const row = el("tr", {"data-id": step.id, class: (step.error || step.passed === false || (step.kind === "try" && !step.result.go) ? "fail " : "") + (step.try ? "tried" : ""),
        onclick: () => app.select(step.id)},
        el("td", {style: `padding-left:${6 + 14 * step.depth}px`}, step.kind.replace("_", " ") + (step.piece ? " · " + step.piece : "")),
        el("td", {class: "n"}, step.try && step.kind !== "try" ? "+" + fix(step.t0, 1) : fix(step.t0, 1)),
        el("td", {class: "n"}, step.kind === "checkpoint" ? "" : fix(step.sim_s, 1)), el("td", {class: "n"}, step.kind === "checkpoint" ? "" : fix(step.wall_s, 1)),
        el("td", {}, outcome(step)));
      this.body.append(row);
    }
    body.append(el("div", {class: "scroll"}, el("table", {}, el("thead", {}, el("tr", {}, el("th", {}, "Step"), el("th", {class: "n"}, "Start, s"),
      el("th", {class: "n"}, "Sim, s"), el("th", {class: "n"}, "Wall, s"), el("th", {}, "Result"))), this.body)),
      el("div", {class: "note"}, "Start is the film's time; inside a lookahead try it counts from the try's own start. A lookahead takes wall time and no simulated time of the build."));
  },
  update(state) {
    const chosen = state.selected !== null ? this.app.step(state.selected) : null;
    const open = state.tryName || (chosen && chosen.try);
    let now = null;
    for (const row of this.body.children) {
      const step = this.app.step(/^c/.test(row.dataset.id) ? row.dataset.id : Number(row.dataset.id));
      row.hidden = Boolean(step.try && step.kind !== "try" && step.try !== open);
      const live = state.tryName ? step.try === state.tryName && step.kind !== "try" && step.t0 <= state.tryT && state.tryT < step.t1
        : state.active && step.id === state.active.id;
      row.classList.toggle("now", Boolean(live));
      const picked = String(state.selected) === row.dataset.id;
      row.classList.toggle("sel", picked);
      if (live || (picked && !now)) now = row;
    }
    if (now && this.shown !== now) { this.shown = now; this.body.parentElement.parentElement.scrollTop = Math.max(0, now.offsetTop - 120); }
  },
});

Viewer.panel("graph", {
  title: "The Graph-as-Policy workflow",
  mount(body, run, app) {
    this.run = run; this.marks = [];
    const strip = el("div", {class: "graph"});
    run.graph.nodes.forEach((node, index) => {
      const card = el("div", {class: "node", "data-node": node.name}, el("h3", {}, node.name), el("small", {}, node.skill ? "skill: " + node.skill : node.type + (node.status ? ": " + node.status : "")));
      this.marks.push([card, node.name]);
      if (node.children) {
        const list = el("ol", {});
        for (const child of node.children) {
          const item = el("li", {}, el("span", {class: "dot"}), el("span", {}, child.name, " ", el("small", {}, child.does)));
          this.marks.push([item, node.name + "." + child.name]);
          list.append(item);
        }
        this.exits = this.exits || {};
        this.exits[node.name] = el("div", {class: "chips"});
        card.append(list, this.exits[node.name]);
      }
      const span = () => (run.spans[node.name] || [])[0];
      card.onclick = () => { if (span()) { app.endTry(); app.seek(span()[0]); } };
      if (index) strip.append(el("span", {class: "arrow"}, "→"));
      strip.append(card);
    });
    body.append(el("div", {class: "note"}, run.graph.description), strip,
      el("div", {class: "note"}, `Border and dot: grey not reached, amber running, green finished, red failed. Result: ${run.result.exit_status}` +
        (run.result.error ? ", " + run.result.error : "") + ". The trace with every node's inputs and outputs is in the run folder, trace/node_data."));
  },
  status(name, t) {
    let status = "";
    for (const [start, end, how] of this.run.spans[name] || []) {
      if (start > t + 1e-6) break;
      status = end === null || end > t + 1e-6 || (end === start && Math.abs(t - start) < 0.05 && how === "running") ? "active" : how === "ok" ? "done" : "failed";
    }
    return status;
  },
  update(state) {
    for (const [node, name] of this.marks) {
      const status = this.status(name, state.t);
      node.classList.toggle("active", status === "active"); node.classList.toggle("done", status === "done"); node.classList.toggle("failed", status === "failed");
    }
    for (const [name, box] of Object.entries(this.exits || {})) {
      const visits = this.run.spans[name] || [], done = visits.filter((visit) => visit[1] !== null && visit[1] <= state.t + 1e-6).length;
      const taken = this.run.graph.edges.filter((edge) => edge.from === name && edge.label).filter((edge) =>
        (this.run.spans[edge.to] || []).some((visit) => visits.some((mine) => mine[1] !== null && visit[0] >= mine[1] - 1e-6 && mine[1] <= state.t + 1e-6)));
      box.replaceChildren(...(done ? [el("span", {class: "chip"}, visits.length > 1 ? `${done} visits` : "exit: " + (taken.length ? taken[0].label : "?"))] : []));
    }
  },
});

Viewer.panel("plan", {
  title: "The step: plan and result",
  mount(body, run, app) { this.body = body; this.app = app; this.run = run; },
  update(state) {
    const step = state.selected !== null ? this.app.step(state.selected) : state.active;
    if (!step || this.shown === step.id) return;
    this.shown = step.id;
    const plan = step.plan || {}, parts = [];
    parts.push(el("div", {class: "caption"}, `${step.kind.replace("_", " ")}${step.piece ? " · " + step.piece : ""}${step.try ? " (lookahead try " + step.try + ")" : ""}`));
    parts.push(el("div", {}, step.kind === "checkpoint" ? `Checkpoint of ${step.subgraph} against the simulator's truth: ${outcome(step)}.`
      : `Starts at ${fix(step.t0, 1)} s, ${fix(step.sim_s, 1)} s simulated, ${fix(step.wall_s, 1)} s wall. ${outcome(step)}`));
    if (plan.grasp) {
      const grasp = plan.grasp;
      parts.push(el("div", {}, el("b", {}, "Grasp on the CAD: "), `${grasp.what} Point ${grasp.point_m.map((v) => fix(v * 1000, 1)).join(", ")} mm in the piece's frame, closing along [${grasp.closing_axis}], approaching along [${grasp.approach_axis}], ${fix(grasp.width_m * 1000, 1)} mm wide at ${grasp.force_n} N; way ${grasp.way} of holding it` +
        (plan.adjusted ? ", corrected by how the piece sat in the hand in the lookahead." : ".")));
    }
    if (plan.waypoints) {
      parts.push(el("table", {}, el("thead", {}, el("tr", {}, ["Waypoint", "x, mm", "y, mm", "z, mm", "gripper points along"].map((name, index) => el("th", {class: index && index < 4 ? "n" : ""}, name)))),
        el("tbody", {}, plan.waypoints.map((point) => el("tr", {}, el("td", {}, point.name), ...point.xyz.map((v) => el("td", {class: "n"}, fix(v * 1000, 1))),
          el("td", {}, "[" + point.approach.map((v) => fix(v, 2)).join(", ") + "]"))))));
    }
    const spent = step.spent || {};
    if (step.wall_s) {
      const shares = {physics: spent.physics_s || 0, rendering: spent.render_s || 0, sam3: spent.sam3_s || 0, scan: spent.scan_s || 0, planning: spent.plan_s || 0};
      shares.bridge = Math.max(0, (spent.bridge_s || 0) - shares.physics - shares.rendering);
      shares.other = Math.max(0, step.wall_s - Object.values(shares).reduce((a, b) => a + b, 0));
      parts.push(el("div", {}, el("b", {}, "Wall time: "), Object.entries(shares).filter(([, seconds]) => seconds >= 0.05).map(([name, seconds]) => `${name} ${fix(seconds, 1)} s`).join(", ") +
        (spent.bridge_calls ? `; ${spent.bridge_calls} bridge calls` : "")),
        el("div", {class: "split"}, Object.entries(shares).map(([name, seconds]) => el("span", {style: `width:${100 * seconds / step.wall_s}%;background:${SHARE[name]}`, title: name}))));
    }
    if (step.clip) parts.push(el("div", {}, el("a", {href: step.clip}, "This step's clip")));
    if (step.result) parts.push(el("pre", {}, JSON.stringify(step.result, null, 1).slice(0, 1500)));
    this.body.replaceChildren(...parts);
  },
});

Viewer.panel("plots", {
  title: "Joints and gripper over time",
  mount(body, run, app) {
    this.run = run; this.app = app;
    this.arm = el("canvas", {}); this.rail = el("canvas", {}); this.grip = el("canvas", {});
    this.what = el("div", {class: "note"});
    body.append(this.what, legend(run.columns.slice(2, 8).map((name, index) => [name, PALETTE[index]])), this.arm,
      legend([[run.columns[1], PALETTE[6]]]), this.rail,
      legend([[run.columns[8], PALETTE[0]], ["grip force commanded, N (the drive's limit while closed; not a measured force)", PALETTE[3]]]), this.grip);
  },
  update(state) {
    const source = state.tryName ? this.run.tries[state.tryName] : this.run, t = state.tryName ? state.tryT : state.t;
    const chosen = state.selected !== null ? this.app.step(state.selected) : state.active;
    const band = chosen && (chosen.try || null) === state.tryName && chosen.kind !== "try" && chosen.t1 > chosen.t0 ? [chosen.t0, chosen.t1] : null;
    const column = (index) => source.samples.map((row) => [row[0], row[index]]);
    this.what.textContent = state.tryName ? `Lookahead try ${state.tryName} (its own clock)` : "The build";
    chart(this.arm, [2, 3, 4, 5, 6, 7].map((index, order) => ({color: PALETTE[order], points: column(index)})), {t, band, height: 170});
    chart(this.rail, [{color: PALETTE[6], points: column(1)}], {t, band, height: 70});
    const end = source.samples.length ? source.samples[source.samples.length - 1][0] : 0;
    const gap = column(8), top = Math.max(1, ...gap.map((point) => point[1]));
    const force = [[source.samples.length ? source.samples[0][0] : 0, 0], ...source.grips, [end, source.grips.length ? source.grips[source.grips.length - 1][1] : 0]];
    chart(this.grip, [{color: PALETTE[0], points: gap}, {color: PALETTE[3], step: true, points: force.map(([time, newtons]) => [time, newtons * top / 40])}], {t, band, height: 110});
  },
});

Viewer.panel("perception", {
  title: "Perception: where the wrist camera puts each piece",
  mount(body, run, app) {
    this.run = run; this.app = app; this.pick = null; this.masks = true;
    this.chips = el("div", {class: "chips"}, run.looks.map((look) => el("span", {class: "chip pick", "data-look": look.index,
      onclick: () => { this.pick = look.index; this.drawn = null; this.update(app.state); }}, `${look.index} ${look.label}`)),
      el("span", {class: "chip pick", onclick: () => { this.masks = !this.masks; this.drawn = null; this.update(app.state); }}, "masks / picture"));
    this.image = el("img", {alt: "The wrist camera's picture of this look"});
    this.map = el("canvas", {});
    this.table = el("div", {});
    body.append(this.chips, el("div", {class: "two"}, el("figure", {}, this.image, el("figcaption", {}, "Wrist D415 with the SAM 3.1 masks tinted")),
      el("figure", {}, this.map, el("figcaption", {}, "From above. Solid: the scan's estimate. Dashed: the simulator's truth, shown for scoring only; the robot never reads it."))), this.table);
  },
  update(state) {
    const looks = this.run.looks;
    if (!looks.length) return;
    const look = looks.find((one) => one.index === this.pick) || [...looks].reverse().find((one) => one.t <= state.t + 1e-6) || looks[0];
    const chosen = state.selected !== null ? this.app.step(state.selected) : state.active;
    const path = chosen && chosen.plan && chosen.plan.waypoints ? chosen.plan.waypoints : [];
    const key = look.index + "|" + this.masks + "|" + (chosen ? chosen.id : "");
    if (this.drawn === key) return;
    this.drawn = key;
    if (!state.tryName && this.pick !== null && look.t > state.t) this.pick = null;
    for (const chip of this.chips.children) chip.classList.toggle("on", Number(chip.dataset.look) === look.index);
    this.image.hidden = !look.masks;
    if (look.masks) this.image.src = this.masks ? look.masks : look.picture;
    const names = Object.keys(look.truth), all = [...Object.values(look.truth), ...Object.values(look.estimate)].flatMap((piece) => piece.outline);
    const xs = all.map((p) => p[0]), ys = all.map((p) => p[1]), pad = 0.05;
    const x0 = Math.min(...xs) - pad, x1 = Math.max(...xs) + pad, y0 = Math.min(...ys) - pad, y1 = Math.max(...ys) + pad;
    const width = this.map.clientWidth || 320, height = Math.round(width * (y1 - y0) / (x1 - x0)), ratio = window.devicePixelRatio || 1;
    this.map.width = width * ratio; this.map.height = height * ratio; this.map.style.height = height + "px";
    const pen = this.map.getContext("2d");
    pen.scale(ratio, ratio);
    const px = (p) => [width * (x1 - p[0]) / (x1 - x0), height * (p[1] - y0) / (y1 - y0)];       // as the wrist camera sees the table: +x to the left, the robot at the top
    const outline = (points, color, dashed) => {
      pen.setLineDash(dashed ? [4, 3] : []); pen.strokeStyle = color; pen.lineWidth = dashed ? 1 : 2; pen.beginPath();
      points.forEach((p, index) => (index ? pen.lineTo(...px(p)) : pen.moveTo(...px(p)))); pen.closePath(); pen.stroke();
    };
    names.forEach((name, index) => {
      outline(look.truth[name].outline, ink("--soft"), true);
      if (look.estimate[name]) {
        outline(look.estimate[name].outline, PALETTE[index % PALETTE.length], false);
        pen.fillStyle = PALETTE[index % PALETTE.length]; pen.font = "11px system-ui";
        pen.fillText(name, ...px([look.estimate[name].xyz[0], look.estimate[name].xyz[1]]));
      }
    });
    if (path.length) {
      pen.setLineDash([]); pen.strokeStyle = ink("--ink"); pen.fillStyle = ink("--ink"); pen.lineWidth = 1; pen.beginPath();
      path.forEach((point, index) => (index ? pen.lineTo(...px(point.xyz)) : pen.moveTo(...px(point.xyz)))); pen.stroke();
      for (const point of path) { const [u, v] = px(point.xyz); pen.fillRect(u - 2, v - 2, 5, 5); pen.fillText(point.name, u + 5, v - 4); }
    }
    this.table.replaceChildren(look.scores ? el("table", {}, el("thead", {}, el("tr", {}, ["Piece", "Named", "Position error, mm", "Rotation error, deg"].map((name, index) => el("th", {class: index > 1 ? "n" : ""}, name)))),
      el("tbody", {}, look.scores.map((row) => el("tr", {}, el("td", {}, row.name), el("td", {class: row.result === "correct" ? "ok" : "bad"}, row.result + (row.as && row.as !== row.name ? " as " + row.as : "")),
        el("td", {class: "n"}, fix(row.position_error_mm, 2)), el("td", {class: "n"}, fix(row.rotation_error_deg, 2))))))
      : el("div", {class: "note"}, "This run read the simulator's own poses (perception: oracle); there is no camera estimate to score."));
  },
});

Viewer.panel("timing", {
  title: "Where the wall time went",
  mount(body, run) {
    const timing = run.timing, names = timing.categories;
    const row = (group, strong) => el("tr", {}, el(strong ? "th" : "td", {}, group.kind.replace("_", " ")), el("td", {class: "n"}, group.count), el("td", {class: "n"}, fix(group.wall_s, 0)),
      el("td", {class: "n"}, fix(group.sim_s, 0)), el("td", {style: "min-width:160px"}, el("div", {class: "split", style: `width:${Math.max(2, 100 * group.wall_s / timing.total.wall_s)}%`},
        names.map((name) => el("span", {style: `width:${group.wall_s ? 100 * group[name] / group.wall_s : 0}%;background:${SHARE[name]}`, title: `${name} ${group[name]} s`})))),
      names.map((name) => el("td", {class: "n"}, fix(group[name], 0))), el("td", {class: "n"}, group.bridge_calls));
    body.append(
      el("div", {class: "caption"}, `${fix(timing.wall_s / 60, 1)} min of wall time for ${fix(timing.simulated_s, 0)} s of simulated time: ${timing.slower_than_real_time} times slower than real time.`),
      legend(names.map((name) => [name, SHARE[name]])),
      el("div", {class: "scroll"}, el("table", {}, el("thead", {}, el("tr", {}, ["Step", "Count", "Wall, s", "Sim, s", "Share of all steps", ...names.map((name) => name + ", s"), "Bridge calls"].map((name, index) =>
        el("th", {class: index && index !== 4 ? "n" : ""}, name)))), el("tbody", {}, timing.rows.map((group) => row(group, false)), row(timing.total, true)))),
      el("div", {class: "note"}, `Outside the steps: ${timing.outside_steps.vlm_s} s waiting for the vision-language model, ${timing.outside_steps.graph_and_other_s} s in the graph's own work and everything else. ` +
        "Physics is the time inside the solver; rendering is drawing the film's and the looks' frames; bridge is what the calls to the simulator cost beyond those two; planning is choosing grasps on the CAD. A lookahead's physics and rendering are in its row."));
  },
  update() {},
});

if (window.RUN) window.addEventListener("DOMContentLoaded", () => { window.app = Viewer.load(window.RUN); });
