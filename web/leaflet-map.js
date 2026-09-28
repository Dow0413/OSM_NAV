/* Leaflet base tiles with local OSM navigation overlays. */
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
map.createPane('pcdPane');
map.getPane('pcdPane').style.zIndex = 350;
map.getPane('pcdPane').style.pointerEvents = 'none';
let baseLayer;

function setBaseMap(kind) {
  if (baseLayer) {
    map.removeLayer(baseLayer);
    baseLayer = undefined;
  }
  if (kind === 'openstreetmap') {
    baseLayer = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 24,
      maxNativeZoom: 19,
      attribution: '<a href="https://www.openstreetmap.org/copyright">© OpenStreetMap contributors</a>',
    }).addTo(map);
  } else if (kind === 'esri_satellite') {
    baseLayer = L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      {
        maxZoom: 24,
        maxNativeZoom: 19,
        attribution: 'Tiles © Esri — Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community',
      },
    ).addTo(map);
  }
  const name = {openstreetmap: 'OpenStreetMap 街道底图', esri_satellite: 'Esri 卫星底图', none: '本地 OSM 线路'}[kind] || kind;
  document.querySelector('.map-toolbar span').textContent = `${name} · 滚轮缩放 · 拖拽平移 · 任意点击设置起终点`;
}

const groupNames = ['features', 'roads', 'arrows', 'nodes', 'signals', 'labels', 'route', 'endpoints', 'robot', 'pcd_buildings', 'highlights'];
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
const layerSettings = {roads: true, arrows: true, labels: false, nodes: false, features: false, signals: true, route: true, endpoints: true};
const uiConfig = {default_display_mode: 'compact', use_base_map: 'esri_satellite', mode: 0};
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
let signalData = [];
let signalNavigation = null;
let pcdMetadata = null;
let pcdOverlayLayer = null;
let pcdBuildingData = [];
let designCategory = null;
let ruleDesignActive = false;
let selectRuleRoad = null;

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
  const selectable = ruleDesignActive && typeof selectRuleRoad === 'function';
  const selectRoad = event => { if (!selectable) return; L.DomEvent.stopPropagation(event.originalEvent); selectRuleRoad(String(road.id)); };
  const outline = L.polyline(road.coordinates, {renderer: vectorRenderer, color: '#aa9f93', weight: scaled(width + 1.25, 'road'), dashArray, interactive: selectable}).addTo(layers.roads);
  const directionCoordinates = road.oneway === '-1' ? [...road.coordinates].reverse() : road.coordinates;
  const line = L.polyline(directionCoordinates, {renderer: vectorRenderer, color, weight: scaled(width, 'road'), dashArray, interactive: selectable}).addTo(layers.roads);
  if (selectable) { outline.on('click', selectRoad); line.on('click', selectRoad); }
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
    drawSignalMarkers();
  }
}

function drawSignalMarkers() {
  layers.signals.clearLayers();
  if (!layerSettings.signals) return;
  for (const signal of signalData) {
    L.circleMarker([signal.latitude, signal.longitude], {
      renderer: vectorRenderer,
      radius: scaled(signal.controls_active_profile ? 5 : 3, 'signal'),
      color: '#fff', fillColor: signal.state === 1 ? '#ce4d47' : '#28965e',
      fillOpacity: 1, weight: scaled(1.3, 'signal'), interactive: false,
    }).addTo(layers.signals);
  }
}

function pcdBounds() {
  if (!pcdMetadata?.available) return null;
  return L.latLngBounds(pcdMetadata.bounds[document.getElementById('pcd-frame').value]);
}

function drawPcdBuildings() {
  layers.pcd_buildings.clearLayers();
  if (!document.getElementById('pcd-visible').checked || !document.getElementById('pcd-buildings').checked) return;
  const bounds = pcdBounds();
  if (!bounds) return;
  for (const building of pcdBuildingData) {
    if (building.coordinates.length < 3 || !L.latLngBounds(building.coordinates).intersects(bounds)) continue;
    L.polygon(building.coordinates, {renderer: vectorRenderer, color: '#f28b28', weight: 2.4,
      fill: false, interactive: false}).addTo(layers.pcd_buildings);
  }
}

function updatePcdInfo(message) {
  document.getElementById('pcd-info').textContent = message;
}

async function loadPcdMetadata() {
  try {
    const response = await fetch('/api/pcd');
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || response.statusText);
    pcdMetadata = data;
    if (!data.available) {
      updatePcdInfo('服务端未启用 PCD 叠加；请设置 pcd_overlay: true。');
      document.getElementById('pcd-visible').disabled = true;
      return;
    }
    updatePcdInfo(`${data.file}：${data.point_count.toLocaleString()} 点\nENU 原点：${data.enu_origin_lla[0].toFixed(7)}, ${data.enu_origin_lla[1].toFixed(7)}\n勾选后加载俯视图，并缩放到点云范围。`);
  } catch (error) {
    updatePcdInfo(`点云元数据加载失败：${error}`);
  }
}

async function refreshPcdOverlay() {
  if (pcdOverlayLayer) {
    map.removeLayer(pcdOverlayLayer);
    pcdOverlayLayer = null;
  }
  if (!document.getElementById('pcd-visible').checked) {
    drawPcdBuildings();
    return;
  }
  if (!pcdMetadata?.available) {
    await loadPcdMetadata();
    if (!pcdMetadata?.available) return;
  }
  const frame = document.getElementById('pcd-frame').value;
  const height = document.getElementById('pcd-height').value;
  const opacity = Number(document.getElementById('pcd-opacity').value) / 100;
  const url = `/api/pcd/overlay?${new URLSearchParams({frame, height})}`;
  pcdOverlayLayer = L.imageOverlay(url, pcdBounds(), {pane: 'pcdPane', opacity, interactive: false}).addTo(map);
  pcdOverlayLayer.on('error', () => updatePcdInfo('点云图像加载失败；请检查服务端日志和 PCD 格式。'));
  if (!pcdBuildingData.length) {
    try {
      const response = await fetch('/api/buildings');
      if (!response.ok) throw new Error(response.statusText);
      pcdBuildingData = (await response.json()).buildings;
    } catch (error) {
      updatePcdInfo(`OSM 建筑轮廓加载失败：${error}`);
    }
  }
  drawPcdBuildings();
}

document.getElementById('pcd-visible').addEventListener('change', refreshPcdOverlay);
document.getElementById('pcd-buildings').addEventListener('change', drawPcdBuildings);
document.getElementById('pcd-frame').addEventListener('change', refreshPcdOverlay);
document.getElementById('pcd-height').addEventListener('change', refreshPcdOverlay);
document.getElementById('pcd-opacity').addEventListener('input', event => {
  const value = Number(event.target.value);
  document.getElementById('pcd-opacity-value').textContent = `${value}%`;
  if (pcdOverlayLayer) pcdOverlayLayer.setOpacity(value / 100);
});
document.getElementById('pcd-fit').addEventListener('click', async () => {
  if (!pcdMetadata?.available) await loadPcdMetadata();
  const bounds = pcdBounds();
  if (bounds) map.fitBounds(bounds, {animate: false, padding: [20, 20]});
});

function updateSignalPanel() {
  const list = document.getElementById('signal-list');
  list.innerHTML = signalData.length ? signalData.map(signal => `
    <div class="signal-item">
      <div class="signal-name"><b>${escapeHtml(signal.id)}</b><small>${signal.controls_active_profile ? '参与当前规则族的受控信号' : '仅地图显示，不参与当前规划'}</small></div>
      <button type="button" class="${signal.state === 1 ? 'red' : 'green'}" data-signal-id="${escapeHtml(signal.id)}">${signal.state === 1 ? '1 红灯' : '0 绿灯'}</button>
    </div>`).join('') : '<p class="signal-note">当前 OSM 没有交通信号灯。</p>';
  const display = document.getElementById('signal-runtime-status');
  if (Number(uiConfig.mode) !== 1) {
    display.textContent = '当前是自由规划模式：可演示切换灯色；机器狗自动停车需 mode=1。';
  } else if (signalNavigation?.holding_signal_id) {
    display.textContent = `红灯等待中：${signalNavigation.holding_signal_id}，已通过 /stop 和零速度覆盖暂停。`;
  } else {
    display.textContent = '导航正常；接近受控过街路段且对应灯为红灯时自动等待。';
  }
}

async function pollSignals() {
  try {
    const response = await fetch('/api/signals');
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || response.statusText);
    // Topology-setting selection changes only the planning graph. Every OSM
    // traffic signal remains visible in the map and category-display panels.
    const visibleSignals = data.signals;
    const changed = JSON.stringify(signalData) !== JSON.stringify(visibleSignals) ||
      signalNavigation?.holding_signal_id !== data.navigation?.holding_signal_id;
    signalData = visibleSignals;
    signalNavigation = data.navigation;
    if (changed) {
      updateSignalPanel();
      drawSignalMarkers();
      renderDesignPanel();
    }
  } catch (error) {
    document.getElementById('signal-runtime-status').textContent = `读取信号灯失败：${error}`;
  }
}

document.getElementById('signal-list').addEventListener('click', async event => {
  const button = event.target.closest('button[data-signal-id]');
  if (!button) return;
  const signal = signalData.find(item => item.id === button.dataset.signalId);
  if (!signal) return;
  button.disabled = true;
  try {
    const response = await fetch(`/api/signals/${encodeURIComponent(signal.id)}`, {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({state: signal.state === 1 ? 0 : 1}),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || response.statusText);
    signalData = data.signals;
    signalNavigation = data.navigation;
    updateSignalPanel();
    drawSignalMarkers();
    renderDesignPanel();
  } catch (error) {
    document.getElementById('signal-runtime-status').textContent = `切换信号灯失败：${error}`;
  } finally {
    button.disabled = false;
  }
});

function roadCategoryLabel(kind) {
  return {footway: '人行道', path: '小径', pedestrian: '步行街', steps: '台阶', living_street: '生活街道', service: '服务道路', residential: '居民区道路', unclassified: '未分类道路', track: '便道', tertiary: '次干道', secondary: '二级道路', primary: '主干道'}[kind] || kind;
}

function designCategories() {
  if (!mapData) return [];
  const categories = [{id: null, label: '清除高亮'}];
  for (const kind of [...new Set((mapData.roads || []).map(road => road.kind))].sort()) categories.push({id: `road:${kind}`, label: roadCategoryLabel(kind)});
  categories.push({id: 'building', label: '建筑物'});
  categories.push({id: 'area:all', label: '全部区域'});
  if (displayMode === 'full') {
    for (const kind of [...new Set((mapData.areas || []).map(area => area.kind))].sort()) categories.push({id: `area:${kind}`, label: `区域：${kind}`});
  }
  categories.push({id: 'signal', label: '交通信号点'});
  return categories;
}

function renderDesignPanel() {
  const roads = document.getElementById('design-road-list');
  const features = document.getElementById('design-feature-list');
  const clear = document.getElementById('design-clear');
  if (!roads || !features || !clear) return;
  roads.replaceChildren(); features.replaceChildren();
  for (const category of designCategories()) {
    if (category.id === null) continue;
    const button = document.createElement('button');
    button.type = 'button'; button.textContent = category.label;
    button.classList.toggle('active', category.id === designCategory);
    button.addEventListener('click', () => selectDesignCategory(category.id));
    (category.id.startsWith('road:') ? roads : features).append(button);
  }
  clear.classList.toggle('active', designCategory === null);
  clear.onclick = () => selectDesignCategory(null);
}

function drawDesignHighlight() {
  if (!designCategory) return;
  const [type, kind] = designCategory.split(':');
  if (type === 'road') {
    for (const road of mapData.roads || []) if (road.kind === kind) L.polyline(road.coordinates, {renderer: vectorRenderer, color: '#d94b42', weight: 6, opacity: .95, lineCap: 'round', interactive: false}).addTo(layers.highlights);
  } else if (type === 'building') {
    for (const building of mapData.buildings || []) L.polygon(building.coordinates, {renderer: vectorRenderer, color: '#d94b42', fillColor: '#ff7b72', fillOpacity: .58, weight: 2, interactive: false}).addTo(layers.highlights);
  } else if (type === 'area') {
    for (const area of mapData.areas || []) if (kind === 'all' || area.kind === kind) L.polygon(area.coordinates, {renderer: vectorRenderer, color: '#d94b42', fillColor: '#ffb347', fillOpacity: .45, weight: 2, interactive: false}).addTo(layers.highlights);
  } else if (type === 'signal') {
    for (const signal of signalData) L.circleMarker([signal.latitude, signal.longitude], {renderer: vectorRenderer, radius: 9, color: '#fff', fillColor: '#d94b42', fillOpacity: 1, weight: 3, interactive: false}).addTo(layers.highlights);
  }
}

function selectDesignCategory(category) {
  designCategory = category;
  const needsFull = category && !category.startsWith('road:');
  if (needsFull && displayMode !== 'full') {
    displayMode = 'full'; updateDisplayMode(); loadMap(); return;
  }
  renderMap(); renderDesignPanel();
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
  drawDesignHighlight();
  window.redrawRuleHighlights?.();
  drawRoute();
  drawEndpoints();
  drawPcdBuildings();
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
  let readout = `纬度：${point[0].toFixed(7)}<br>经度：${point[1].toFixed(7)}`;
  if (Number(uiConfig.mode) === 1) {
    const affine = uiConfig.transform?.gps_to_slam;
    if (affine) {
      const latitudeDelta = point[0] - affine.reference_latitude;
      const longitudeDelta = point[1] - affine.reference_longitude;
      const x = affine.offset[0] + affine.matrix[0][0] * latitudeDelta + affine.matrix[0][1] * longitudeDelta;
      const y = affine.offset[1] + affine.matrix[1][0] * latitudeDelta + affine.matrix[1][1] * longitudeDelta;
      readout += `<br>/Odometry x：${x.toFixed(2)} m<br>/Odometry y：${y.toFixed(2)} m`;
    } else {
      readout += '<br>/Odometry x、y：坐标转换不可用';
    }
  }
  hoverCoordinate.innerHTML = readout;
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
  if (ruleDesignActive) { updateStatus('规则设计模式：请直接点击道路选择，不会设置起点或终点。'); return; }
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
    renderDesignPanel();
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
  if (input.checked && ['labels', 'nodes', 'features'].includes(key) && displayMode !== 'full') {
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
  setBaseMap(uiConfig.use_base_map);
  loadPcdMetadata();
  document.querySelector('[data-layer="signals"]').checked = true;
  pollSignals();
  window.setInterval(pollSignals, 1000);
  if (Number(uiConfig.mode) === 1) {
    hoverCoordinate.innerHTML = '纬度：--<br>经度：--<br>/Odometry x：-- m<br>/Odometry y：-- m';
    document.querySelector('.steps').innerHTML = '<li>等待 <code>/Odometry</code> 定位消息</li><li>在地图点击设置终点（经纬度）</li><li>点击“开启导航”发布 <code>/global_path</code></li>';
    navigate.disabled = true;
    pollRobotPose();
    window.setInterval(pollRobotPose, 250);
  }
  updateDisplayMode();
  loadMap();
}

initialize();

// OSM per-way regulatory editor. In this mode map clicks select roads, never endpoints.
(() => {
  const select = document.getElementById('rule-road-select');
  if (!select) return;
  const maxspeed = document.getElementById('rule-maxspeed'), oneway = document.getElementById('rule-oneway'), signal = document.getElementById('rule-signal');
  const via = document.getElementById('rule-via'), to = document.getElementById('rule-to'), turn = document.getElementById('rule-turn'), status = document.getElementById('rule-design-status');
  let roads = [], ownHighlights = [];
  const label = road => `${road.name || '未命名道路'} · ${road.kind} · #${road.id}`;
  const selected = () => roads.find(road => road.id === select.value);
  const show = message => { status.textContent = message; };
  const removeHighlights = () => { ownHighlights.forEach(layer => layers.highlights.removeLayer(layer)); ownHighlights = []; };
  const displayRoad = id => (mapData?.roads || []).find(road => String(road.id) === String(id));
  const addRoadHighlight = (id, color, weight) => { const road = displayRoad(id); if (road?.coordinates) ownHighlights.push(L.polyline(road.coordinates, {renderer:vectorRenderer, color, weight, opacity:.95, interactive:false}).addTo(layers.highlights)); };
  function allowedEndpoints(road) { return road.oneway === 'yes' ? [road.nodes.at(-1)] : road.oneway === '-1' ? [road.nodes[0]] : [road.nodes[0], road.nodes.at(-1)]; }
  function canLeaveAt(road, nodeId) { return road.oneway === 'yes' ? road.nodes[0] === nodeId : road.oneway === '-1' ? road.nodes.at(-1) === nodeId : road.nodes[0] === nodeId || road.nodes.at(-1) === nodeId; }
  function redrawHighlights() { removeHighlights(); if (!ruleDesignActive) return; const road=selected(); if (!road) return; addRoadHighlight(road.id,'#d94b42',7); if(to.value) addRoadHighlight(to.value,'#f0a62d',6); for(const item of signalData.filter(item=>road.nodes.includes(String(item.id)))) ownHighlights.push(L.circleMarker([item.latitude,item.longitude],{renderer:vectorRenderer,radius:8,color:'#fff',fillColor:'#34c77b',fillOpacity:1,weight:3,interactive:false}).addTo(layers.highlights)); }
  function updateTargets() { const road=selected(), nodeId=via.value; if(!road||!nodeId)return; const candidates=roads.filter(item=>item.id!==road.id&&item.nodes.includes(nodeId)&&canLeaveAt(item,nodeId)); to.innerHTML=candidates.length?candidates.map(item=>`<option value="${escapeHtml(item.id)}">${escapeHtml(label(item))}</option>`).join(''):'<option value="">该路口没有可前往的道路</option>'; redrawHighlights(); }
  function applyRoad(fit=true) { const road=selected(); if(!road)return; maxspeed.value=road.maxspeed||''; oneway.value=['yes','-1'].includes(road.oneway)?road.oneway:'no'; signal.checked=road.traffic_signals; via.innerHTML=allowedEndpoints(road).map(id=>`<option value="${escapeHtml(id)}">路口节点 ${escapeHtml(id)}</option>`).join(''); updateTargets(); if(fit){const display=displayRoad(road.id);if(display?.coordinates)map.fitBounds(L.latLngBounds(display.coordinates),{padding:[35,35],maxZoom:20});} }
  selectRuleRoad = id => { if(!ruleDesignActive)return; select.value=String(id); applyRoad(true); show(`已从地图选中：${label(selected())}。红色为当前道路，橙色为可前往的目标道路，绿色为关联信号灯。`); };
  window.redrawRuleHighlights = redrawHighlights;
  async function loadRules() { try { const response=await fetch('/api/rules/roads'); const data=await response.json(); if(!response.ok)throw new Error(data.error||response.statusText); roads=data.roads; select.innerHTML=roads.map(road=>`<option value="${escapeHtml(road.id)}">${escapeHtml(label(road))}</option>`).join(''); applyRoad(false); show(`已加载 ${roads.length} 条道路；进入规则设计后可直接在地图点击道路。`); } catch(error) { show(`读取规则失败：${error.message}`); } }
  select.addEventListener('change',()=>applyRoad(true)); via.addEventListener('change',updateTargets); to.addEventListener('change',redrawHighlights);
  document.getElementById('rule-save-road').addEventListener('click',async()=>{const road=selected();if(!road)return;try{const response=await fetch('/api/rules/save-road',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id:road.id,maxspeed:maxspeed.value,oneway:oneway.value,traffic_signals:signal.checked})});const data=await response.json();if(!response.ok)throw new Error(data.error||response.statusText);show(`道路规则已保存；备份：${data.backup}。请重启服务使规划生效。`);await loadRules();}catch(error){show(`保存失败：${error.message}`);}});
  document.getElementById('rule-save-turn').addEventListener('click',async()=>{const road=selected();if(!road||!to.value)return;try{const response=await fetch('/api/rules/add-turn-restriction',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({from:road.id,via:via.value,to:to.value,restriction:turn.value})});const data=await response.json();if(!response.ok)throw new Error(data.error||response.statusText);show(`转向限制已新增；备份：${data.backup}。请重启服务使规划生效。`);}catch(error){show(`新增失败：${error.message}`);}});
  window.addEventListener('osm-nav-menu-change',event=>{ruleDesignActive=event.detail.target==='rule-design'; if(ruleDesignActive){layerSettings.roads=true;document.querySelector('[data-layer="roads"]').checked=true;start=null;goal=null;lastRoute=null;navigate.disabled=true;renderMap();applyRoad(false);updateStatus('规则设计模式：请直接点击道路选择，不会设置起点或终点。');}else{removeHighlights();renderMap();}});
  loadRules();
})();
