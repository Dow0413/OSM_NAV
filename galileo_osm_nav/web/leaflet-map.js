/* Leaflet satellite base map with OSM navigation overlays. */
const statusBox = document.getElementById('status');
const navigate = document.getElementById('navigate');
const reset = document.getElementById('reset');
const hoverCoordinate = document.getElementById('hover-coordinate');

if (!window.L) {
  statusBox.textContent = 'Leaflet 地图库加载失败。请确认浏览器可以访问 unpkg.com。';
  throw new Error('Leaflet is unavailable');
}

const map = L.map('map', {
  zoomControl: true,
  preferCanvas: true,
  maxBoundsViscosity: 1.0,
});
const vectorRenderer = L.canvas({padding: 0.5});
let satelliteLayer;

function setSatelliteBase(enabled) {
  if (satelliteLayer) {
    map.removeLayer(satelliteLayer);
    satelliteLayer = undefined;
  }
  if (enabled) {
    satelliteLayer = L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      {
        maxZoom: 19,
        attribution: 'Tiles © Esri — Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community',
      },
    ).addTo(map);
  }
  document.querySelector('.map-toolbar span').textContent = enabled
    ? '卫星底图 · 滚轮缩放 · 拖拽平移 · 任意点击设置起终点'
    : 'OSM 矢量底图 · 滚轮缩放 · 拖拽平移 · 任意点击设置起终点';
}

const groupNames = ['features', 'roads', 'arrows', 'nodes', 'signals', 'labels', 'route', 'endpoints', 'robot'];
const layers = Object.fromEntries(groupNames.map(name => [name, L.layerGroup().addTo(map)]));
const roadStyle = {
  motorway: ['#f0cf6e', 7], trunk: ['#f1dc85', 6], primary: ['#f4e99e', 5],
  secondary: ['#f4f59f', 4.5], tertiary: ['#fff8bd', 3.5], unclassified: ['#fff', 3],
  residential: ['#fff', 2.6], service: ['#fff', 2], footway: ['#e9ddec', 2.2],
  path: ['#e9ddec', 1.6], pedestrian: ['#e9ddec', 2.2], cycleway: ['#c6e5ed', 2.2],
};
const areaStyle = {
  commercial: ['#efd3d4', '#d5989c'], industrial: ['#ded2c9', '#9e9289'],
  residential: ['#e4d9eb', '#b79cbf'], retail: ['#efd8cd', '#d6a18c'],
  grass: ['#d9ebd4', '#9ab98f'], forest: ['#cfe4cc', '#85ad85'], water: ['#cfeaf0', '#80b9c7'],
};
const displaySettings = {road: .65, arrow: 1, label: .65, node: 1, feature: .65, signal: 1, route: .65, endpoint: 1};
const layerSettings = {roads: true, arrows: true, labels: false, nodes: false, features: false, signals: false, route: true, endpoints: true};
const uiConfig = {default_display_mode: 'compact', use_leaflet: true, mode: 0};
let mapData;
let osmBounds;
let initialBoundsSet = false;
let start;
let goal;
let referencePoint;
let lastRoute;
let displayMode = 'compact';
let hoverPoint;
let hoverFrame = 0;
let robotPose;

function updateStatus(text) {
  statusBox.textContent = text;
}

function escapeHtml(value) {
  return String(value).replace(/[&<>'"]/g, character => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;'}[character]));
}

function scaled(value, key) {
  return Math.max(0, value * displaySettings[key]);
}

function centerOf(coordinates) {
  const sum = coordinates.reduce((result, point) => [result[0] + point[0], result[1] + point[1]], [0, 0]);
  return [sum[0] / coordinates.length, sum[1] / coordinates.length];
}

function addLabel(text, coordinate, roadLabel = false) {
  if (!text || !layerSettings.labels) return;
  const className = `leaflet-label${roadLabel ? ' leaflet-road-label' : ''}`;
  L.marker(coordinate, {
    icon: L.divIcon({className, html: escapeHtml(text), iconAnchor: [0, roadLabel ? 8 : 6]}),
    interactive: false,
    keyboard: false,
  }).addTo(layers.labels);
}

function addDirectionArrow(coordinates) {
  if (coordinates.length < 2 || !layerSettings.arrows) return;
  const index = Math.floor((coordinates.length - 1) / 2);
  const first = coordinates[index];
  const second = coordinates[index + 1];
  const angle = Math.atan2(-(second[0] - first[0]), second[1] - first[1]) * 180 / Math.PI;
  const size = Math.max(1, Math.round(15 * displaySettings.arrow));
  L.marker(first, {
    icon: L.divIcon({
      className: 'direction-arrow',
      html: `<span style="display:block;font-size:${size}px;transform:rotate(${angle}deg)">➤</span>`,
      iconSize: [size + 4, size + 4],
      iconAnchor: [(size + 4) / 2, (size + 4) / 2],
    }),
    interactive: false,
    keyboard: false,
  }).addTo(layers.arrows);
}

function drawFeatures() {
  if (!layerSettings.features) return;
  for (const area of mapData.areas || []) {
    const [fillColor, color] = areaStyle[area.kind] || ['#e6dde3', '#bcaeb9'];
    L.polygon(area.coordinates, {renderer: vectorRenderer, color, fillColor, fillOpacity: .42, weight: scaled(.8, 'feature'), interactive: false}).addTo(layers.features);
    addLabel(area.name, centerOf(area.coordinates));
  }
  for (const building of mapData.buildings || []) {
    L.polygon(building.coordinates, {renderer: vectorRenderer, color: '#9a8f87', fillColor: '#d9cec5', fillOpacity: .72, weight: scaled(.8, 'feature'), interactive: false}).addTo(layers.features);
    addLabel(building.name, centerOf(building.coordinates));
  }
}

function drawRoad(road) {
  if (!layerSettings.roads || road.coordinates.length < 2) return;
  const [color, width] = roadStyle[road.kind] || ['#fff', 2];
  const dashArray = ['footway', 'path', 'pedestrian', 'cycleway'].includes(road.kind) ? '4 3' : undefined;
  L.polyline(road.coordinates, {renderer: vectorRenderer, color: '#aa9f93', weight: scaled(width + 1.25, 'road'), dashArray, interactive: false}).addTo(layers.roads);
  const directionCoordinates = road.oneway === '-1' ? [...road.coordinates].reverse() : road.coordinates;
  L.polyline(directionCoordinates, {renderer: vectorRenderer, color, weight: scaled(width, 'road'), dashArray, interactive: false}).addTo(layers.roads);
  if (['yes', '-1'].includes(road.oneway)) addDirectionArrow(directionCoordinates);
  if (road.name && road.kind !== 'footway') addLabel(road.name, directionCoordinates[Math.floor(directionCoordinates.length / 2)], true);
}

function drawPoints() {
  if (layerSettings.nodes) {
    for (const [, latitude, longitude] of mapData.nodes || []) {
      L.circleMarker([latitude, longitude], {renderer: vectorRenderer, radius: scaled(1.35, 'node'), color: '#3f5260', fillColor: '#3f5260', fillOpacity: .6, weight: 0, interactive: false}).addTo(layers.nodes);
    }
  }
  if (layerSettings.signals) {
    for (const [, latitude, longitude] of mapData.traffic_signals || []) {
      L.circleMarker([latitude, longitude], {renderer: vectorRenderer, radius: scaled(2.4, 'signal'), color: '#514011', fillColor: '#f5cc45', fillOpacity: 1, weight: scaled(1, 'signal'), interactive: false}).addTo(layers.signals);
    }
  }
}

function drawRoute() {
  if (!layerSettings.route || !lastRoute) return;
  L.polyline(lastRoute.path, {renderer: vectorRenderer, color: '#d94b42', weight: scaled(4.2, 'route'), lineCap: 'round', lineJoin: 'round', interactive: false}).addTo(layers.route);
  for (const [selected, snapped] of [[lastRoute.selectedStart || start, lastRoute.startSnap], [lastRoute.selectedGoal || goal, lastRoute.goalSnap]]) {
    if (selected && snapped && L.latLng(selected).distanceTo(snapped) > .1) {
      L.polyline([selected, snapped], {renderer: vectorRenderer, color: '#65747b', weight: scaled(1.4, 'route'), dashArray: '4 4', interactive: false}).addTo(layers.route);
    }
  }
}

function drawEndpoints() {
  if (!layerSettings.endpoints) return;
  if (referencePoint) L.circleMarker(referencePoint, {renderer: vectorRenderer, radius: scaled(4, 'endpoint'), color: '#fff', fillColor: '#9653b6', fillOpacity: 1, weight: scaled(1.1, 'endpoint'), interactive: false}).addTo(layers.endpoints);
  if (start && Number(uiConfig.mode) !== 1) L.circleMarker(start, {renderer: vectorRenderer, radius: scaled(5, 'endpoint'), color: '#183243', fillColor: '#269c53', fillOpacity: 1, weight: scaled(1.1, 'endpoint'), interactive: false}).addTo(layers.endpoints);
  if (goal) L.circleMarker(goal, {renderer: vectorRenderer, radius: scaled(5, 'endpoint'), color: '#183243', fillColor: '#1c73bc', fillOpacity: 1, weight: scaled(1.1, 'endpoint'), interactive: false}).addTo(layers.endpoints);
  if (Number(uiConfig.mode) === 1 && robotPose) {
    L.circleMarker([robotPose.latitude, robotPose.longitude], {renderer: vectorRenderer, radius: scaled(6, 'endpoint'), color: '#183243', fillColor: '#f07d32', fillOpacity: 1, weight: scaled(1.3, 'endpoint'), interactive: false}).addTo(layers.robot);
  }
}

function renderMap() {
  if (!mapData) return;
  Object.values(layers).forEach(layer => layer.clearLayers());
  drawFeatures();
  (mapData.roads || []).forEach(drawRoad);
  drawPoints();
  drawRoute();
  drawEndpoints();
}

function fitToOsmBounds() {
  if (!osmBounds) return;
  map.fitBounds(osmBounds, {animate: false, padding: [0, 0]});
}

function setMapBounds() {
  osmBounds = L.latLngBounds(
    [mapData.bounds.min_lat, mapData.bounds.min_lon],
    [mapData.bounds.max_lat, mapData.bounds.max_lon],
  );
  map.setMaxBounds(osmBounds.pad(.00001));
  if (!initialBoundsSet) {
    fitToOsmBounds();
    map.setMinZoom(map.getZoom());
    initialBoundsSet = true;
  }
}

function inBounds(point) {
  return osmBounds && osmBounds.contains(point);
}

function updateHover(point) {
  hoverCoordinate.innerHTML = `纬度：${point[0].toFixed(7)}<br>经度：${point[1].toFixed(7)}`;
}

function scheduleHover(point) {
  hoverPoint = point;
  if (hoverFrame) return;
  hoverFrame = requestAnimationFrame(() => {
    hoverFrame = 0;
    if (hoverPoint) updateHover(hoverPoint);
  });
}

function clearRoute() {
  lastRoute = null;
}

function resetSelection() {
  start = null;
  goal = null;
  clearRoute();
  navigate.disabled = true;
  renderMap();
  updateStatus('点击地图任意位置选择起点。');
}

function choosePoint(latlng) {
  const point = [latlng.lat, latlng.lng];
  if (Number(uiConfig.mode) === 1) {
    goal = point;
    lastRoute = null;
    navigate.disabled = !robotPose;
    renderMap();
    updateStatus(robotPose ? '已选择终点。点击“开启导航”将发布 /global_path。' : '已选择终点，正在等待 /Odometry 定位消息。');
    return;
  }
  if (!start) {
    start = point;
    updateStatus('已选择起点。请点击地图选择终点。');
  } else if (!goal) {
    goal = point;
    navigate.disabled = false;
    updateStatus('已选择终点。点击“开启导航”。');
  } else {
    start = point;
    goal = null;
    navigate.disabled = true;
    clearRoute();
    updateStatus('已重新选择起点。请点击地图选择终点。');
  }
  renderMap();
}

function showCoordinate() {
  const latitude = Number(document.getElementById('input-latitude').value);
  const longitude = Number(document.getElementById('input-longitude').value);
  if (!Number.isFinite(latitude) || !Number.isFinite(longitude)) {
    updateStatus('请输入有效的纬度和经度。');
    return;
  }
  referencePoint = [latitude, longitude];
  if (!inBounds(referencePoint)) {
    updateStatus('该坐标不在当前 OSM 地图边界内，无法在此地图中定位。');
    return;
  }
  map.panTo(referencePoint, {animate: false});
  renderMap();
  updateStatus(`已显示坐标点：${latitude.toFixed(7)}, ${longitude.toFixed(7)}`);
}

function updateDisplayMode() {
  const compact = displayMode === 'compact';
  document.getElementById('compact-mode').classList.toggle('active', compact);
  document.getElementById('full-mode').classList.toggle('active', !compact);
  document.getElementById('map-detail-value').textContent = compact ? '简略显示' : '全部显示';
}

async function loadMap() {
  updateStatus(displayMode === 'compact' ? '正在加载简略道路图…' : '正在加载全部地图要素，较大地图可能需要较长时间…');
  try {
    const response = await fetch(`/api/map?detail=${displayMode}`);
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || response.statusText);
    mapData = data;
    setMapBounds();
    start = null;
    goal = null;
    lastRoute = null;
    navigate.disabled = true;
    renderMap();
    updateStatus(displayMode === 'compact' ? '卫星底图与简略道路图已加载。点击地图选择起点。' : '卫星底图与全部地图要素已加载。点击地图选择起点。');
  } catch (error) {
    updateStatus(`地图加载失败：${error}`);
  }
}

async function pollRobotPose() {
  if (Number(uiConfig.mode) !== 1) return;
  try {
    const response = await fetch('/api/robot_pose');
    const pose = await response.json();
    if (!response.ok || !pose.available) return;
    robotPose = pose;
    start = [pose.latitude, pose.longitude];
    if (goal) navigate.disabled = false;
    renderMap();
  } catch (error) {
    console.warn('Unable to read robot position:', error);
  }
}

map.on('mousemove', event => scheduleHover([event.latlng.lat, event.latlng.lng]));
map.on('click', event => choosePoint(event.latlng));
window.addEventListener('resize', () => map.invalidateSize());
document.getElementById('zoom-reset').addEventListener('click', fitToOsmBounds);
document.getElementById('show-coordinate').addEventListener('click', showCoordinate);
document.getElementById('compact-mode').addEventListener('click', () => {
  if (displayMode !== 'compact') {
    displayMode = 'compact';
    updateDisplayMode();
    loadMap();
  }
});
document.getElementById('full-mode').addEventListener('click', () => {
  if (displayMode !== 'full') {
    displayMode = 'full';
    updateDisplayMode();
    loadMap();
  }
});
document.querySelectorAll('[data-scale]').forEach(input => input.addEventListener('input', () => {
  const key = input.dataset.scale;
  const value = Number(input.value);
  displaySettings[key] = value / 100;
  const output = document.getElementById(`${key}-scale-value`);
  output.value = `${value}%`;
  output.textContent = `${value}%`;
  if (key === 'label' && !layerSettings.labels) {
    layerSettings.labels = true;
    document.querySelector('[data-layer="labels"]').checked = true;
    if (displayMode !== 'full') {
      displayMode = 'full';
      updateDisplayMode();
      loadMap();
      return;
    }
  }
  renderMap();
}));
document.getElementById('display-reset').addEventListener('click', () => {
  const defaults = {road: 65, arrow: 100, label: 65, node: 100, feature: 65, signal: 100, route: 65, endpoint: 100};
  Object.entries(defaults).forEach(([key, value]) => {
    displaySettings[key] = value / 100;
    const input = document.querySelector(`[data-scale="${key}"]`);
    const output = document.getElementById(`${key}-scale-value`);
    input.value = value;
    output.value = `${value}%`;
    output.textContent = `${value}%`;
  });
  renderMap();
});
document.querySelectorAll('[data-layer]').forEach(input => input.addEventListener('change', () => {
  const key = input.dataset.layer;
  layerSettings[key] = input.checked;
  if (input.checked && ['labels', 'nodes', 'features', 'signals'].includes(key) && displayMode !== 'full') {
    displayMode = 'full';
    updateDisplayMode();
    loadMap();
  } else {
    renderMap();
  }
}));
navigate.addEventListener('click', async () => {
  if (!start || !goal) return;
  updateStatus('正在将选点投影到道路并规划…');
  const query = new URLSearchParams({start_lat: start[0], start_lon: start[1], goal_lat: goal[0], goal_lon: goal[1]});
  const response = await fetch(`/api/route?${query}`);
  const data = await response.json();
  if (!response.ok) {
    updateStatus(`规划失败：${data.error}`);
    return;
  }
  lastRoute = {
    path: data.path,
    startSnap: data.start.coordinate,
    goalSnap: data.goal.coordinate,
    selectedStart: [...start],
    selectedGoal: [...goal],
  };
  renderMap();
  const roadText = place => `${place.road.name || place.road.kind}，偏移 ${place.distance_m.toFixed(1)} m`;
  const signalText = data.traffic_signal_count ? `交通信号点：${data.traffic_signal_count} 处（执行时等待绿灯）` : '交通信号点：路线未经过已标注的信号点';
  const costText = data.planning_cost_s === undefined ? '' : `\n规则代价：${data.planning_cost_s.toFixed(1)} s 等效`;
  const publishText = data.global_path_topic ? `\n已发布 ${data.slam_path.length} 个 SLAM 路径点至：${data.global_path_topic}` : '';
  updateStatus(`规划完成\n路线长度：${data.distance_m.toFixed(1)} m${costText}\n${signalText}\n起点投影至：${roadText(data.start)}\n终点投影至：${roadText(data.goal)}${publishText}`);
});
reset.addEventListener('click', resetSelection);

async function initialize() {
  try {
    const response = await fetch('/api/config');
    if (response.ok) Object.assign(uiConfig, await response.json());
    if (['compact', 'full'].includes(uiConfig.default_display_mode)) displayMode = uiConfig.default_display_mode;
    if (uiConfig.map_name) document.getElementById('map-name').textContent = uiConfig.map_name;
  } catch (error) {
    console.warn('Using built-in UI defaults:', error);
  }
  setSatelliteBase(Boolean(uiConfig.use_leaflet));
  if (Number(uiConfig.mode) === 1) {
    document.querySelector('.steps').innerHTML = '<li>等待 <code>/Odometry</code> 定位消息</li><li>在地图点击设置终点（经纬度）</li><li>点击“开启导航”发布 <code>/global_path</code></li>';
    navigate.disabled = true;
    pollRobotPose();
    window.setInterval(pollRobotPose, 250);
  }
  updateDisplayMode();
  loadMap();
}

initialize();
