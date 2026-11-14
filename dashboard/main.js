import { FlightScene } from './scene.js';

const $ = (id) => document.getElementById(id);

const FIELD_SECTIONS = [
  {
    title: 'Flight profile', open: true, columns: 2, fields: [
      ['controller', 'H', 'Hover height', 'm', 0.1],
      ['controller', 'V', 'Vertical speed', 'm/s', 0.1],
      ['controller', 'Ti', 'Hover starts', 's', 0.1],
      ['controller', 'Th', 'Hover duration', 's', 0.1],
      ['controller', 'Tf', 'Landing time', 's', 0.1],
      ['controller', 'Z', 'Initial accel.', 'm/s²', 0.0000000001],
      ['controller', 'control_frequency', 'Control rate', 'Hz', 1],
      ['rocket', 't_final', 'Simulation time', 's', 1],
      ['root', 'seed', 'Random seed', '', 1],
    ],
  },
  {
    title: 'Rocket', open: true, columns: 2, fields: [
      ['rocket', 'T', 'Max thrust', 'N', .01],
      ['rocket', 'P_max', 'Max throttle cut', '0–1', 0.01],
      ['rocket', 'm0', 'Controller mass', 'kg', 0.01],
      ['rocket', 'm_dot', 'Mass flow', 'kg/s', 0.01],
      ['rocket', 'g', 'Gravity', 'm/s²', 0.01],
      ['rocket', 'dNoz', 'CoM to nozzle', 'm', 0.01],
      ['rocket', 'l', 'Effective length', 'm', 0.01],
      ['rocket', 'r', 'Effective radius', 'm', 0.01],
      ['rocket', 'Tau0', 'RCS torque', 'N·m', 0.01],
    ],
  },
  {
    title: 'Aerodynamics', open: false, columns: 2, fields: [
      ['rocket', 'Cd0', 'Drag coefficient', '', 0.01],
      ['rocket', 'Cl0', 'Lift coefficient', '', 0.01],
      ['rocket', 'S0', 'Reference area', 'm²', 0.0001],
      ['rocket', 'dCoP', 'CoM to CoP', 'm', 0.01],
      ['rocket', 'rho', 'Air density', 'kg/m³', 0.001],
    ],
  },
];

const STATE_FIELDS = [
  ['x', 'x', 'm'], ['y', 'y', 'm'], ['z', 'z', 'm'],
  ['dx', 'ẋ', 'm/s'], ['dy', 'ẏ', 'm/s'], ['dz', 'ż', 'm/s'],
  ['phi', 'φ', 'rad'], ['theta', 'θ', 'rad'], ['psi', 'ψ', 'rad'],
  ['dphi', 'φ̇', 'rad/s'], ['dtheta', 'θ̇', 'rad/s'], ['dpsi', 'ψ̇', 'rad/s'],
  ['mass', 'mass', 'kg'],
];

const WIND_FIELDS = [
  ['speed', 'Wind magnitude', 'm/s', 0, 20, 0.1],
  ['direction_deg', 'Wind direction', 'deg', -180, 180, 1],
  ['gust_variance', 'Gust variance', '(m/s)²', 0, 100, 0.1],
];

const MATRIX_FIELDS = [
  { group: 'controller', key: 'Q', title: 'LQR Q diagonal', names: ['x', 'z', 'ẋ', 'ż', 'angle', 'rate'] },
  { group: 'controller', key: 'R', title: 'LQR R diagonal', names: ['gimbal', 'throttle'] },
  { group: 'rocket', key: 'Q_kalman', title: 'Kalman disturbance diagonal', names: ['position x', 'position z', 'velocity x', 'velocity z', 'angle', 'angle rate'] },
];

const METRICS = [
  ['altitude', 'Altitude', 'm'],
  ['reference', 'Reference', 'm'],
  ['verticalSpeed', 'Vertical speed', 'm/s'],
  ['drift', 'Lateral drift', 'm'],
  ['attitude', 'Roll / pitch / yaw', 'deg'],
  ['throttle', 'Effective throttle', '%'],
  ['gimbal', 'Gimbal x / y', 'deg'],
  ['wind', 'Wind x / y', 'm/s'],
  ['mass', 'Mass', 'kg'],
];

const scene = new FlightScene($('scene'));
let defaults = null;
let currentConfig = null;
let flight = null;
let playing = false;
let playbackTime = 0;
let lastWallTime = null;
let shownFrame = -1;
let followCamera = true;

function deepClone(value) {
  return JSON.parse(JSON.stringify(value));
}

function numberInput(value, step, path, index = null) {
  const input = document.createElement('input');
  input.type = 'number';
  input.required = true;
  input.value = value;
  input.step = step;
  input.dataset.path = path;
  if (index !== null) input.dataset.index = String(index);
  return input;
}

function sliderInput(value, path, minimum, maximum, step, unit) {
  const wrapper = document.createElement('div');
  wrapper.className = 'slider-control';
  const input = document.createElement('input');
  input.type = 'range';
  input.min = minimum;
  input.max = maximum;
  input.step = step;
  input.value = value;
  input.dataset.path = path;
  const output = document.createElement('output');
  const update = () => { output.textContent = `${Number(input.value).toFixed(step < 1 ? 1 : 0)} ${unit}`; };
  input.addEventListener('input', update);
  update();
  wrapper.append(input, output);
  return wrapper;
}

function field(label, code, unit, input) {
  const wrapper = document.createElement('label');
  wrapper.className = 'field';
  const title = document.createElement('span');
  title.className = 'field-label';
  const text = document.createElement('span');
  text.textContent = label;
  const symbols = document.createElement('span');
  const codeNode = document.createElement('code');
  codeNode.textContent = code;
  symbols.append(codeNode);
  if (unit) {
    const unitNode = document.createElement('span');
    unitNode.className = 'unit';
    unitNode.textContent = ` ${unit}`;
    symbols.append(unitNode);
  }
  title.append(text, symbols);
  wrapper.append(title, input);
  return wrapper;
}

function addSection(form, title, open, columns = 2) {
  const details = document.createElement('details');
  details.open = open;
  const summary = document.createElement('summary');
  summary.textContent = title;
  const fields = document.createElement('div');
  fields.className = `fields${columns === 3 ? ' three' : ''}`;
  details.append(summary, fields);
  form.append(details);
  return fields;
}

function renderForm(config) {
  currentConfig = deepClone(config);
  const form = $('parameterForm');
  form.replaceChildren();
  for (const section of FIELD_SECTIONS) {
    const container = addSection(form, section.title, section.open, section.columns);
    for (const [group, key, label, unit, step] of section.fields) {
      const value = group === 'root' ? config[key] : config[group][key];
      container.append(field(label, key, unit, numberInput(value, step, `${group}.${key}`)));
    }
    if (section.title === 'Flight profile') {
      const wind = addSection(form, 'Wind & gusts', true, 2);
      for (const [key, label, unit, minimum, maximum, step] of WIND_FIELDS) {
        const control = sliderInput(config.wind[key], `wind.${key}`, minimum, maximum, step, unit);
        const item = field(label, key, '', control);
        item.classList.add('wide');
        wind.append(item);
      }
      const note = document.createElement('div');
      note.className = 'matrix-note';
      note.textContent = 'Direction is measured from +x toward +y. Gust variance applies independently to x and y.';
      wind.append(note);
    }
  }

  const state = addSection(form, 'Initial state', false, 3);
  STATE_FIELDS.forEach(([key, symbol, unit], index) => {
    state.append(field(key, symbol, unit, numberInput(config.state[index], 0.01, 'state', index)));
  });

  const matrices = addSection(form, 'Control matrices', false, 3);
  for (const matrix of MATRIX_FIELDS) {
    const note = document.createElement('div');
    note.className = 'matrix-note';
    note.textContent = matrix.title;
    matrices.append(note);
    config[matrix.group][matrix.key].forEach((value, index) => {
      matrices.append(field(matrix.names[index], `${matrix.key}${index + 1}`, '', numberInput(value, 'any', `${matrix.group}.${matrix.key}`, index)));
    });
  }

  // The default state mass follows m0 until the user deliberately changes it.
  const m0Input = form.querySelector('[data-path="rocket.m0"]');
  const stateMass = form.querySelector('[data-path="state"][data-index="12"]');
  let lastMass = Number(m0Input.value);
  m0Input.addEventListener('input', () => {
    if (Number(stateMass.value) === lastMass) stateMass.value = m0Input.value;
    lastMass = Number(m0Input.value);
  });
}

function readForm() {
  if (!$('parameterForm').reportValidity()) throw new Error('Please correct the highlighted parameter.');
  const result = deepClone(currentConfig);
  for (const input of $('parameterForm').querySelectorAll('input[data-path]')) {
    const value = Number(input.value);
    if (!Number.isFinite(value)) throw new Error('Every parameter must be a finite number.');
    const [group, key] = input.dataset.path.split('.');
    if (group === 'root') result[key] = value;
    else if (group === 'state') result.state[Number(input.dataset.index)] = value;
    else if (input.dataset.index !== undefined) result[group][key][Number(input.dataset.index)] = value;
    else result[group][key] = value;
  }
  result.seed = Math.trunc(result.seed);
  return result;
}

function buildTelemetry() {
  const grid = $('telemetryGrid');
  for (const [id, label, unit] of METRICS) {
    const card = document.createElement('div');
    card.className = 'metric';
    const title = document.createElement('span');
    title.textContent = label;
    const value = document.createElement('strong');
    value.id = `metric-${id}`;
    value.textContent = '—';
    const unitNode = document.createElement('em');
    unitNode.textContent = unit;
    value.append(unitNode);
    card.append(title, value);
    grid.append(card);
  }
}

function metric(id, value, unit) {
  const node = $(`metric-${id}`);
  node.replaceChildren(document.createTextNode(value));
  const suffix = document.createElement('em');
  suffix.textContent = unit;
  node.append(suffix);
}

function formatClock(seconds) {
  const minutes = Math.floor(seconds / 60);
  const remainder = seconds - 60 * minutes;
  return `T+ ${String(minutes).padStart(2, '0')}:${remainder.toFixed(1).padStart(4, '0')}`;
}

function showTelemetry(frame) {
  const [x, y, z] = frame.position;
  const [dx, dy, dz] = frame.velocity;
  metric('altitude', z.toFixed(2), 'm');
  metric('reference', frame.reference_altitude.toFixed(2), 'm');
  metric('verticalSpeed', dz.toFixed(2), 'm/s');
  metric('drift', Math.hypot(x, y).toFixed(2), 'm');
  metric('attitude', frame.euler_deg.map((value) => value.toFixed(1)).join(' / '), 'deg');
  metric('throttle', frame.throttle_pct.toFixed(1), '%');
  metric('gimbal', frame.gimbal_deg.map((value) => value.toFixed(2)).join(' / '), 'deg');
  metric('wind', frame.wind.slice(0, 2).map((value) => value.toFixed(2)).join(' / '), 'm/s');
  metric('mass', frame.mass.toFixed(2), 'kg');
  $('phase').textContent = frame.phase;
  $('phase').classList.remove('ready');
  $('clock').textContent = formatClock(frame.t);
  void dx; void dy;
}

function resizeCanvas(canvas) {
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
  const height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, ratio };
}

function palette(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function chartBase(canvas) {
  const size = resizeCanvas(canvas);
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, size.width, size.height);
  context.lineWidth = size.ratio;
  context.strokeStyle = palette('--line');
  context.beginPath();
  for (let row = 1; row < 4; row += 1) {
    const y = row * size.height / 4;
    context.moveTo(0, y);
    context.lineTo(size.width, y);
  }
  context.stroke();
  return { context, ...size };
}

function plotLine(context, points, xScale, yScale, color, ratio) {
  if (!points.length) return;
  context.strokeStyle = color;
  context.lineWidth = 1.3 * ratio;
  context.beginPath();
  points.forEach((point, index) => {
    const x = xScale(point[0]);
    const y = yScale(point[1]);
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  });
  context.stroke();
}

function drawCharts(cursor = 0) {
  if (!flight?.frames?.length) return;
  const frames = flight.frames;
  const lastT = Math.max(frames.at(-1).t, 1e-9);

  const altitude = chartBase($('altitudeChart'));
  const values = frames.flatMap((frame) => [frame.position[2], frame.reference_altitude]);
  const minAltitude = Math.min(0, ...values);
  const maxAltitude = Math.max(1, ...values);
  const altitudeSpan = Math.max(maxAltitude - minAltitude, 1e-9);
  const tx = (t) => t / lastT * altitude.width;
  const ay = (value) => altitude.height - (value - minAltitude) / altitudeSpan * altitude.height;
  const stride = Math.max(1, Math.floor(frames.length / Math.max(altitude.width, 1)));
  const actual = [], reference = [];
  for (let index = 0; index < frames.length; index += stride) {
    actual.push([frames[index].t, frames[index].position[2]]);
    reference.push([frames[index].t, frames[index].reference_altitude]);
  }
  plotLine(altitude.context, reference, tx, ay, palette('--aqua'), altitude.ratio);
  plotLine(altitude.context, actual, tx, ay, palette('--blue'), altitude.ratio);
  altitude.context.strokeStyle = palette('--accent');
  altitude.context.lineWidth = altitude.ratio;
  altitude.context.beginPath();
  altitude.context.moveTo(tx(frames[cursor].t), 0);
  altitude.context.lineTo(tx(frames[cursor].t), altitude.height);
  altitude.context.stroke();

  const track = chartBase($('trackChart'));
  const extent = Math.max(1, ...frames.flatMap((frame) => [Math.abs(frame.position[0]), Math.abs(frame.position[1])])) * 1.1;
  const px = (value) => track.width / 2 + value / (2 * extent) * track.width;
  const py = (value) => track.height / 2 - value / (2 * extent) * track.height;
  const points = [];
  for (let index = 0; index < frames.length; index += stride) points.push([frames[index].position[0], frames[index].position[1]]);
  plotLine(track.context, points, px, py, palette('--blue'), track.ratio);
  const current = frames[cursor].position;
  track.context.fillStyle = palette('--accent');
  track.context.beginPath();
  track.context.arc(px(current[0]), py(current[1]), 3 * track.ratio, 0, 2 * Math.PI);
  track.context.fill();
}

function showFrame(index, forceTrail = false) {
  if (!flight) return;
  index = Math.max(0, Math.min(index, flight.frames.length - 1));
  const jumped = forceTrail || index <= shownFrame || index > shownFrame + 2;
  if (jumped) scene.rebuildTrail(flight.frames, index);
  scene.update(flight.frames[index], !jumped);
  shownFrame = index;
  showTelemetry(flight.frames[index]);
  drawCharts(index);
}

function indexForTime(seconds) {
  if (!flight || flight.result.duration <= 0) return 0;
  return Math.min(flight.frames.length - 1, Math.floor(seconds / flight.result.duration * (flight.frames.length - 1)));
}

function updatePlaybackUi() {
  const duration = flight?.result.duration || 0;
  $('timeline').value = duration ? String(playbackTime / duration) : '0';
  $('playbackTime').textContent = `${playbackTime.toFixed(1)} / ${duration.toFixed(1)} s`;
  $('playButton').textContent = playing ? '❚❚' : '▶';
}

function setPlaying(value) {
  if (!flight) return;
  if (value && playbackTime >= flight.result.duration) {
    playbackTime = 0;
    shownFrame = -1;
    scene.reset();
  }
  playing = value;
  lastWallTime = null;
  updatePlaybackUi();
}

function revealResult() {
  const result = flight.result;
  const panel = $('result');
  panel.replaceChildren();
  const title = document.createElement('strong');
  title.textContent = result.message;
  const text = document.createTextNode(
    `Peak ${result.max_altitude.toFixed(2)} m · final drift ${result.horizontal_distance.toFixed(2)} m · final speed ${result.final_speed.toFixed(2)} m/s · ${result.raw_samples} solver samples`,
  );
  panel.append(title, text);
  panel.classList.remove('hidden');
}

function animationLoop(wallTime) {
  if (playing && flight) {
    if (lastWallTime !== null) playbackTime += (wallTime - lastWallTime) / 1000 * Number($('speed').value);
    lastWallTime = wallTime;
    if (playbackTime >= flight.result.duration) {
      playbackTime = flight.result.duration;
      playing = false;
      revealResult();
    }
    const index = indexForTime(playbackTime);
    if (index !== shownFrame) showFrame(index);
    updatePlaybackUi();
  }
  requestAnimationFrame(animationLoop);
}

async function launch() {
  $('formError').textContent = '';
  let config;
  try {
    config = readForm();
  } catch (error) {
    $('formError').textContent = error.message;
    return;
  }
  $('launchButton').disabled = true;
  $('solving').classList.remove('hidden');
  $('serverStatus').className = 'status busy';
  $('serverStatus').lastChild.textContent = ' Solver running';
  $('result').classList.add('hidden');
  setPlaying(false);
  try {
    const response = await fetch('/api/simulate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    });
    const answer = await response.json();
    if (!response.ok) throw new Error(answer.error || `HTTP ${response.status}`);
    flight = answer;
    scene.setup(answer);
    playbackTime = 0;
    shownFrame = -1;
    showFrame(0, true);
    $('timeline').disabled = false;
    $('playButton').disabled = false;
    $('replayButton').disabled = false;
    $('sampleRate').textContent = `${Math.round((answer.frames.length - 1) / Math.max(answer.result.duration, 1e-9))} Hz`;
    setPlaying(true);
    $('serverStatus').className = 'status';
    $('serverStatus').lastChild.textContent = ' Trajectory ready';
  } catch (error) {
    $('formError').textContent = error.message;
    $('serverStatus').className = 'status error';
    $('serverStatus').lastChild.textContent = ' Simulation error';
  } finally {
    $('launchButton').disabled = false;
    $('solving').classList.add('hidden');
  }
}

async function initialize() {
  buildTelemetry();
  try {
    const response = await fetch('/api/defaults', { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    defaults = await response.json();
    renderForm(defaults.config);
    scene.setup(defaults);
  } catch (error) {
    $('serverStatus').className = 'status error';
    $('serverStatus').lastChild.textContent = ` Server unavailable: ${error.message}`;
    $('launchButton').disabled = true;
    return;
  }

  $('launchButton').addEventListener('click', launch);
  $('resetButton').addEventListener('click', () => {
    renderForm(defaults.config);
    flight = null;
    playing = false;
    playbackTime = 0;
    shownFrame = -1;
    scene.setup(defaults);
    $('result').classList.add('hidden');
    $('timeline').disabled = true;
    $('playButton').disabled = true;
    $('replayButton').disabled = true;
    $('phase').textContent = 'READY';
    $('phase').classList.add('ready');
    $('clock').textContent = 'T+ 00:00.0';
    for (const [id, , unit] of METRICS) metric(id, '—', unit);
    $('serverStatus').className = 'status';
    $('serverStatus').lastChild.textContent = ' Simulator ready';
    updatePlaybackUi();
  });
  $('playButton').addEventListener('click', () => setPlaying(!playing));
  $('replayButton').addEventListener('click', () => {
    playbackTime = 0;
    shownFrame = -1;
    scene.reset();
    showFrame(0, true);
    setPlaying(true);
  });
  $('timeline').addEventListener('input', () => {
    if (!flight) return;
    playing = false;
    playbackTime = Number($('timeline').value) * flight.result.duration;
    showFrame(indexForTime(playbackTime), true);
    updatePlaybackUi();
  });
  $('cameraButton').addEventListener('click', () => {
    followCamera = !followCamera;
    scene.setFollow(followCamera);
    $('cameraButton').textContent = followCamera ? 'Overview' : 'Follow vehicle';
  });
  window.addEventListener('resize', () => drawCharts(Math.max(0, shownFrame)));
  requestAnimationFrame(animationLoop);
}

initialize();
