const statusIndicator = document.getElementById('status-indicator');
const mappingStatus = document.getElementById('mapping-status');
const mappingProcesses = document.getElementById('mapping-processes');
const navigationStatus = document.getElementById('navigation-status');
const navigationProcesses = document.getElementById('navigation-processes');
const sensorStatus = document.getElementById('sensor-status');
const robotInfo = document.getElementById('robot-info');
const waypointIndicator = document.getElementById('local-waypoints-indicator');
const executionIndicator = document.getElementById('route-execution-indicator');
const stopExecutionBtn = document.getElementById('btn-stop-execution');
const mapCanvas = document.getElementById('map-canvas');
const routeList = document.getElementById('route-list');
const costmapSelect = document.getElementById('select-costmap-kind');

const ctx = mapCanvas.getContext('2d');
const costmapCanvas = document.createElement('canvas');
const costmapCtx = costmapCanvas.getContext('2d');

const buttons = {
  refreshMap: document.getElementById('btn-refresh-map'),
  refreshCostmap: document.getElementById('btn-refresh-costmap'),
  refreshMaps: document.getElementById('btn-refresh-maps'),
  deleteMap: document.getElementById('btn-delete-map'),
  startMapping: document.getElementById('btn-start-mapping'),
  stopMapping: document.getElementById('btn-stop-mapping'),
  clearCache: document.getElementById('btn-clear-cache'),
  startNavigation: document.getElementById('btn-start-navigation'),
  stopNavigation: document.getElementById('btn-stop-navigation'),
  cancelNavGoal: document.getElementById('btn-cancel-nav-goal'),
  stopNavSystem: document.getElementById('btn-stop-nav'),
  setInitialPose: document.getElementById('btn-set-initial-pose'),
  triggerGlobalReloc: document.getElementById('btn-trigger-global-relocalization'),
  setNavGoal: document.getElementById('btn-set-nav-goal'),
  stand: document.getElementById('btn-stand'),
  walk: document.getElementById('btn-walk'),
  sit: document.getElementById('btn-sit'),
  recover: document.getElementById('btn-recover'),
  stairOn: document.getElementById('btn-stair-on'),
  stairOff: document.getElementById('btn-stair-off'),
  heightUp: document.getElementById('btn-height-up'),
  heightDown: document.getElementById('btn-height-down'),
  emgStop: document.getElementById('btn-emg-stop'),
  saveRoute: document.getElementById('btn-save-route'),
  clearWaypoints: document.getElementById('btn-clear-waypoints'),
  recordWaypoint: document.getElementById('btn-record-waypoint'),
  startTrajectory: document.getElementById('btn-start-trajectory'),
  stopTrajectory: document.getElementById('btn-stop-trajectory'),
};

const inputs = {
  newMapName: document.getElementById('input-new-map-name'),
  routeName: document.getElementById('input-route-name'),
};

// Localizer log auto-scroll control
let localizerAutoScrollEnabled = false;

const selects = {
  mapName: document.getElementById('select-map-name'),
  costmapKind: costmapSelect,
};

const state = {
  waypoints: [],
  robot: null,
  mapMeta: null,
  mapImage: null,
  costmapReady: false,
  selectedCostmap: (selects.costmapKind?.value) || 'local',
  executingRoute: null,
  lastNavStatus: 'IDLE',
  robotPosition: null,  // {x, y, yaw}
  trajectoryRecording: false,
  settingInitialPose: false,  // Whether we're in initial pose setting mode
  initialPose: null,  // {x, y, yaw} - the set initial pose for visualization
  settingNavGoal: false,  // Whether we're in navigation goal setting mode
  navGoal: null,  // {x, y, yaw} - the navigation goal for visualization
  showPointCloud: false,  // Whether to show 2D point cloud overlay
  pointCloudImage: null,  // Point cloud visualization data
  settingPoseStart: null,  // {x, y, canvasX, canvasY} - first click position when setting pose
  tempOrientation: null,  // {x, y, yaw} - temporary orientation for preview
};

const KEY_BINDINGS = {
  w: { linear: { x: 1 } },
  s: { linear: { x: -1 } },
  q: { linear: { y: 1 } },
  e: { linear: { y: -1 } },
  a: { angular: { z: 1 } },
  d: { angular: { z: -1 } },
};

const activeKeys = new Set();
let lastTwist = { x: 0, y: 0, z: 0 };
let sendingTwist = false;

// Notification System
function showNotification(message, type = 'info', duration = 5000) {
  if (!message) return;
  
  let container = document.getElementById('notification-container');
  if (!container) {
    console.error('Notification container not found! Creating it...');
    // Fallback: try to create it
    container = document.createElement('div');
    container.id = 'notification-container';
    container.className = 'notification-container';
    document.body.insertBefore(container, document.body.firstChild);
  }
  
  // Auto-detect type from message content
  if (!type || type === 'info') {
    const msgLower = message.toLowerCase();
    if (msgLower.includes('错误') || msgLower.includes('失败') || msgLower.includes('error') || msgLower.includes('failed')) {
      type = 'error';
    } else if (msgLower.includes('成功') || msgLower.includes('完成') || msgLower.includes('success') || msgLower.includes('完成')) {
      type = 'success';
    } else if (msgLower.includes('警告') || msgLower.includes('注意') || msgLower.includes('warning') || 
               msgLower.includes('请先') || msgLower.includes('需要') || msgLower.includes('必须')) {
      type = 'warning';
    }
  }
  
  // Create notification element
  const notification = document.createElement('div');
  notification.className = `notification ${type}`;
  
  const textSpan = document.createElement('span');
  textSpan.textContent = message;
  notification.appendChild(textSpan);
  
  // Add close button
  const closeBtn = document.createElement('button');
  closeBtn.textContent = '×';
  closeBtn.style.cssText = 'background: transparent; border: none; color: inherit; font-size: 20px; cursor: pointer; padding: 0; margin-left: 15px; line-height: 1; width: 24px; height: 24px; display: flex; align-items: center; justify-content: center;';
  closeBtn.addEventListener('click', () => {
    notification.style.animation = 'fadeOut 0.3s ease-out forwards';
    setTimeout(() => notification.remove(), 300);
  });
  notification.appendChild(closeBtn);
  
  container.appendChild(notification);
  
  // Debug: log notification creation
  console.log(`[Notification] Showing ${type} notification: "${message}"`);
  
  // Auto-remove after duration (longer for warnings/errors)
  const autoRemoveDuration = (type === 'error' || type === 'warning') ? duration * 2 : duration;
  setTimeout(() => {
    if (notification.parentNode) {
      notification.style.animation = 'fadeOut 0.3s ease-out forwards';
      setTimeout(() => notification.remove(), 300);
    }
  }, autoRemoveDuration);
}

function notify(message, type) {
  if (message) {
    // Update status bar (existing behavior)
    statusIndicator.textContent = message;
    
    // Show top notification (new behavior)
    showNotification(message, type);
  }
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function updateWaypointIndicator() {
  if (waypointIndicator) {
    waypointIndicator.textContent = `当前未保存航点：${state.waypoints.length}`;
  }
}

function drawPlaceholder() {
  ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  ctx.fillStyle = 'rgba(148, 163, 184, 0.15)';
  ctx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);
  ctx.fillStyle = 'rgba(226, 232, 240, 0.85)';
  ctx.font = '20px "Segoe UI"';
  ctx.fillText('地图加载中...', 40, 60);
}

function drawWaypoints() {
  if (!state.waypoints.length) {
    return;
  }
  ctx.save();
  ctx.fillStyle = 'rgba(56, 189, 248, 0.9)';
  ctx.strokeStyle = 'rgba(2, 132, 199, 0.9)';
  state.waypoints.forEach((wp, index) => {
    ctx.beginPath();
    ctx.arc(wp.pixelX, wp.pixelY, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = 'rgba(226, 232, 240, 0.9)';
    ctx.font = '12px "Segoe UI"';
    ctx.fillText(`${index + 1}`, wp.pixelX + 6, wp.pixelY + 4);
    ctx.fillStyle = 'rgba(56, 189, 248, 0.9)';
  });
  ctx.restore();
}

function renderOccupancyGrid(rawData, width, height) {
  const image = ctx.createImageData(width, height);
  
  // Backend already converts all data to top-to-bottom format (Canvas standard)
  // ROS2 data is flipped in ros_extended_node, PGM data is already top-to-bottom
  // So we can render directly without flipping
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const idx = y * width + x;
      const pixelIdx = idx * 4;
      
      const value = rawData[idx];
      if (value === 100) {  // Obstacle
        image.data[pixelIdx] = 220;
        image.data[pixelIdx + 1] = 38;
        image.data[pixelIdx + 2] = 38;
        image.data[pixelIdx + 3] = 220;
      } else if (value === 0) {  // Free space
        image.data[pixelIdx] = 30;
        image.data[pixelIdx + 1] = 41;
        image.data[pixelIdx + 2] = 59;
        image.data[pixelIdx + 3] = 180;
      } else {  // Unknown
        image.data[pixelIdx] = 148;
        image.data[pixelIdx + 1] = 163;
        image.data[pixelIdx + 2] = 184;
        image.data[pixelIdx + 3] = 90;
      }
    }
  }
  return image;
}

function drawRobot() {
  if (!state.robotPosition || !state.mapMeta) {
    console.log('drawRobot: no position or meta', {pos: state.robotPosition, meta: state.mapMeta});
    return;
  }
  
  const pos = state.robotPosition;
  const meta = state.mapMeta;
  
  // Convert world coordinates (ROS2: origin at bottom-left, Y up) 
  // to pixel coordinates (Canvas: origin at top-left, Y down)
  const originX = meta.origin_x !== undefined ? meta.origin_x : (meta.origin ? meta.origin[0] : 0);
  const originY = meta.origin_y !== undefined ? meta.origin_y : (meta.origin ? meta.origin[1] : 0);
  const pixelX = (pos.x - originX) / meta.resolution;
  // Flip Y: ROS2 Y increases upward, Canvas Y increases downward
  const pixelY = (meta.height - (pos.y - originY) / meta.resolution);
  
  // Debug: log every draw for comparison with manual_reloc
  console.log('[INDEX] drawRobot:', {
    world: {x: pos.x.toFixed(3), y: pos.y.toFixed(3)},
    origin: {x: originX.toFixed(3), y: originY.toFixed(3)},
    mapSize: {w: meta.width, h: meta.height},
    resolution: meta.resolution.toFixed(4),
    pixel: {x: pixelX.toFixed(1), y: pixelY.toFixed(1)},
    canvas: {w: mapCanvas.width, h: mapCanvas.height}
  });
  
  ctx.save();
  ctx.translate(pixelX, pixelY);
  ctx.rotate(-pos.yaw);  // Rotate to robot heading
  
  // Draw robot as a triangle
  ctx.fillStyle = 'rgba(239, 68, 68, 0.9)';  // Red
  ctx.strokeStyle = 'rgba(185, 28, 28, 1)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(12, 0);   // Front
  ctx.lineTo(-8, -8);  // Back left
  ctx.lineTo(-8, 8);   // Back right
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  
  ctx.restore();
}

function drawInitialPose() {
  if (!state.initialPose || !state.mapMeta) return;
  
  const pose = state.initialPose;
  const meta = state.mapMeta;
  
  // Convert world coordinates to pixel coordinates (with Y flip)
  const originX = meta.origin_x !== undefined ? meta.origin_x : (meta.origin ? meta.origin[0] : 0);
  const originY = meta.origin_y !== undefined ? meta.origin_y : (meta.origin ? meta.origin[1] : 0);
  const pixelX = (pose.x - originX) / meta.resolution;
  const pixelY = (meta.height - (pose.y - originY) / meta.resolution);
  
  ctx.save();
  ctx.translate(pixelX, pixelY);
  ctx.rotate(-pose.yaw);  // Rotate to pose heading
  
  // Draw initial pose as a green arrow
  ctx.fillStyle = 'rgba(34, 197, 94, 0.8)';  // Green
  ctx.strokeStyle = 'rgba(22, 163, 74, 1)';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(15, 0);   // Arrow head
  ctx.lineTo(-10, -10);  // Back left
  ctx.lineTo(-5, 0);   // Middle back
  ctx.lineTo(-10, 10);   // Back right
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  
  // Draw a circle at the base
  ctx.beginPath();
  ctx.arc(0, 0, 5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.stroke();
  
  ctx.restore();
}

function drawNavGoal() {
  if (!state.navGoal || !state.mapMeta) return;
  
  const goal = state.navGoal;
  const meta = state.mapMeta;
  
  // Convert world coordinates to pixel coordinates (with Y flip)
  const originX = meta.origin_x !== undefined ? meta.origin_x : (meta.origin ? meta.origin[0] : 0);
  const originY = meta.origin_y !== undefined ? meta.origin_y : (meta.origin ? meta.origin[1] : 0);
  const pixelX = (goal.x - originX) / meta.resolution;
  const pixelY = (meta.height - (goal.y - originY) / meta.resolution);
  
  console.log('🏁 Drawing Nav Goal (Blue Flag):', {
    worldCoords: { x: goal.x.toFixed(3), y: goal.y.toFixed(3) },
    pixelCoords: { pixelX: pixelX.toFixed(1), pixelY: pixelY.toFixed(1) },
    canvasSize: { w: mapCanvas.width, h: mapCanvas.height }
  });
  
  ctx.save();
  ctx.translate(pixelX, pixelY);
  
  // Draw navigation goal as a blue flag
  ctx.fillStyle = 'rgba(59, 130, 246, 0.9)';  // Blue
  ctx.strokeStyle = 'rgba(37, 99, 235, 1)';
  ctx.lineWidth = 3;
  
  // Flag pole
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0, -25);
  ctx.stroke();
  
  // Flag
  ctx.beginPath();
  ctx.moveTo(0, -25);
  ctx.lineTo(15, -20);
  ctx.lineTo(0, -15);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  
  // Base circle
  ctx.beginPath();
  ctx.arc(0, 0, 6, 0, 2 * Math.PI);
  ctx.fill();
  ctx.stroke();
  
  ctx.restore();
}

function renderScene() {
  if (!state.mapImage) {
    drawPlaceholder();
    return;
  }
  const { image, width, height } = state.mapImage;
  mapCanvas.width = width;
  mapCanvas.height = height;
  ctx.putImageData(image, 0, 0);

  // Draw point cloud overlay if enabled
  if (state.showPointCloud && state.pointCloudImage) {
    ctx.globalAlpha = 0.6;
    ctx.putImageData(state.pointCloudImage, 0, 0);
    ctx.globalAlpha = 1.0;
  }

  if (state.costmapReady) {
    ctx.globalAlpha = 0.35;
    ctx.drawImage(costmapCanvas, 0, 0);
    ctx.globalAlpha = 1.0;
  }

  drawWaypoints();
  drawInitialPose();  // Draw initial pose (green arrow)
  drawNavGoal();  // Draw navigation goal (blue flag)
  drawTempOrientation();  // Draw temporary orientation preview
  drawRobot();  // Draw robot (red triangle) on top
}

// Draw temporary orientation preview (when setting pose/goal)
function drawTempOrientation() {
  if (!state.tempOrientation || !state.mapMeta) return;
  
  const pose = state.tempOrientation;
  const meta = state.mapMeta;
  
  const originX = meta.origin_x !== undefined ? meta.origin_x : (meta.origin ? meta.origin[0] : 0);
  const originY = meta.origin_y !== undefined ? meta.origin_y : (meta.origin ? meta.origin[1] : 0);
  const pixelX = (pose.x - originX) / meta.resolution;
  const pixelY = (meta.height - (pose.y - originY) / meta.resolution);
  
  ctx.save();
  ctx.translate(pixelX, pixelY);
  ctx.rotate(-pose.yaw);
  
  // Draw semi-transparent yellow arrow
  ctx.fillStyle = 'rgba(255, 193, 7, 0.7)';  // Yellow
  ctx.strokeStyle = 'rgba(245, 127, 23, 0.9)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(20, 0);   // Arrow head
  ctx.lineTo(-12, -12);  // Back left
  ctx.lineTo(-6, 0);   // Middle back
  ctx.lineTo(-12, 12);   // Back right
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  
  // Draw direction line
  ctx.strokeStyle = 'rgba(255, 193, 7, 0.6)';
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 5]);
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(40, 0);
  ctx.stroke();
  ctx.setLineDash([]);
  
  ctx.restore();
}

async function fetchMap() {
  try {
    const response = await fetch('/api/robot/map');
    if (!response.ok) {
      // In navigation mode, /api/robot/map is not available
      // This is normal - user should load map via map selector
      console.log('Real-time map not available (use map selector in navigation mode)');
      return;
    }
    const payload = await response.json();
    // Check if map data is available and valid
    if (payload.Result !== 0 || !payload.data || typeof payload.data !== 'string') {
      // Map not received yet or error occurred
      if (payload.Error) {
        console.log('Map not available:', payload.Error);
      }
      return;
    }
    const data = Uint8Array.from(atob(payload.data), c => c.charCodeAt(0));
    const image = renderOccupancyGrid(data, payload.width, payload.height);
    state.mapImage = { image, width: payload.width, height: payload.height };
    state.mapMeta = {
      width: payload.width,
      height: payload.height,
      resolution: payload.resolution,
      origin_x: payload.origin_x,
      origin_y: payload.origin_y,
    };
    state.costmapReady = false;
    renderScene();
    fetchCostmap(state.selectedCostmap);
  } catch (error) {
    // Silently ignore - expected when not in mapping mode
    console.log('Map fetch skipped:', error.message);
  }
}

async function fetchCostmap(kind = 'local') {
  try {
    const response = await fetch(`/api/costmap/${kind}`);
    if (!response.ok) {
      // Costmap not available yet (navigation not started or no map loaded)
      console.log(`Costmap (${kind}) not available yet`);
      return;
    }
    const payload = await response.json();
    // Check if costmap data is available and valid
    if (!payload.data || typeof payload.data !== 'string') {
      console.log(`Costmap (${kind}) data not available`);
      return;
    }
    const data = Uint8Array.from(atob(payload.data), c => c.charCodeAt(0));
    const image = renderOccupancyGrid(data, payload.width, payload.height);
    costmapCanvas.width = payload.width;
    costmapCanvas.height = payload.height;
    costmapCtx.putImageData(image, 0, 0);
    state.costmapReady = true;
    renderScene();
  } catch (error) {
    // Silently ignore - expected when navigation not active
    console.log('Costmap fetch skipped:', error.message);
  }
}

function scheduleCostmapRefresh() {
  setInterval(() => {
    fetchCostmap(state.selectedCostmap);
  }, 7000);
}

async function probeBackend() {
  try {
    const response = await fetch('/healthz');
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    statusIndicator.textContent = `后端状态：${payload.status}`;
  } catch (error) {
    statusIndicator.textContent = `后端不可用：${error}`;
    console.error('Health probe failed', error);
  }
}

async function fetchRobotStatus() {
  try {
    const response = await fetch('/api/robot/status');
    if (!response.ok) return;
    const payload = await response.json();
    state.robot = payload;
    const robotMode = payload.robot_info?.status ?? '未知';
    const navStatus = payload.navigation?.status ?? '未知';
    const battery = payload.battery?.power ?? '?';
    
    // 检测摔倒状态
    let statusDisplay = `模式：${robotMode} ｜ 电量：${battery}`;
    if (robotMode === 'ERROR_FALLOVER') {
      statusDisplay = `⚠️ 机器人摔倒了！ ｜ 电量：${battery}`;
      robotInfo.style.background = '#f8d7da';
      robotInfo.style.color = '#721c24';
      robotInfo.style.fontWeight = 'bold';
    } else if (robotMode === 'RECOVER') {
      statusDisplay = `🔄 恢复中... ｜ 电量：${battery}`;
      robotInfo.style.background = '#fff3cd';
      robotInfo.style.color = '#856404';
    } else if (robotMode === 'ERROR_RECOVER') {
      statusDisplay = `❌ 恢复失败 ｜ 电量：${battery}`;
      robotInfo.style.background = '#f8d7da';
      robotInfo.style.color = '#721c24';
    } else {
      robotInfo.style.background = '';
      robotInfo.style.color = '';
      robotInfo.style.fontWeight = '';
    }
    robotInfo.textContent = statusDisplay;
    state.lastNavStatus = navStatus;
    
    // 更新虚拟摇杆速度上限（根据机器人模式）
    if (joystick) {
      joystick.updateSpeedLimits(robotMode);
    }
    
    if (state.executingRoute && (navStatus === 'SUCCEEDED' || navStatus === 'FAILED')) {
      notify(`巡逻结束：${navStatus}`);
      state.executingRoute = null;
      updateExecutionIndicator();
    }
  } catch (error) {
    console.warn('Failed to fetch robot status', error);
  }
}

async function fetchMappingStatus() {
  try {
    const response = await fetch('/api/mapping/status');
    if (!response.ok) return;
    const payload = await response.json();
    
    let statusText = `建图状态：${payload.status ?? '未知'}`;
    
    // Show save progress if saving in background (includes 2D + 3D PCD map)
    if (payload.save_in_progress) {
      statusText += ` | 保存中`;
      if (payload.save_progress > 0) {
        statusText += ` ${payload.save_progress}%`;
      }
      if (payload.save_stage) {
        statusText += ` (${payload.save_stage})`;
      }
    } else if (payload.save_status === 'success') {
      statusText += ` | 已保存 (含点云地图)`;
    } else if (payload.save_status === 'failed') {
      statusText += ` | 保存失败`;
    }
    
    mappingStatus.textContent = statusText;
    
    // Show detailed process status
    if (payload.processes && payload.processes.length > 0) {
      const pids = payload.processes.join(', ');
      const uptime = payload.uptime ? Math.floor(payload.uptime) : 0;
      mappingProcesses.textContent = `进程: ${payload.processes.length} 个运行中 (PIDs: ${pids}) | 运行时间: ${uptime}s`;
      mappingProcesses.style.color = '#10b981';
    } else {
      mappingProcesses.textContent = `进程状态：无运行进程`;
      mappingProcesses.style.color = '#64748b';
    }
  } catch (error) {
    console.warn('Failed to fetch mapping status', error);
  }
}

async function fetchSensorStatus() {
  try {
    const response = await fetch('/api/sensors/status');
    if (!response.ok) return;
    const payload = await response.json();
    
    const livoxStatus = payload.livox_active ? '✓ 正常' : '✗ 无数据';
    const mapStatus = payload.map_active ? '✓ 正常' : '✗ 无数据';
    const livoxCount = payload.livox_count || 0;
    const mapSize = payload.map_size ? `${payload.map_size.width}x${payload.map_size.height}` : '0x0';
    
    sensorStatus.textContent = `雷达: ${livoxStatus} (${livoxCount}帧) | 地图: ${mapStatus} (${mapSize})`;
  } catch (error) {
    console.warn('Failed to fetch sensor status', error);
  }
}

async function fetchNavigationStatus() {
  try {
    const response = await fetch('/api/navigation/status');
    if (!response.ok) return;
    const payload = await response.json();
    
    let statusText = `导航状态：${payload.status ?? '未知'}`;
    if (payload.status === 'RUNNING') {
      statusText += ` | 进程: ${payload.alive_count}/${payload.total_count}`;
    }
    if (payload.map_name) {
      statusText += ` | 地图: ${payload.map_name}`;
    }
    
    navigationStatus.textContent = statusText;
    
    // Show detailed process status
    if (payload.processes && payload.processes.length > 0) {
      const pids = payload.processes.join(', ');
      const uptime = payload.uptime ? Math.floor(payload.uptime) : 0;
      const aliveCount = payload.alive_count || 0;
      const totalCount = payload.total_count || 0;
      
      let processText = `进程: ${aliveCount}/${totalCount} 运行中`;
      if (aliveCount < totalCount) {
        processText += ` ⚠️ 有进程已停止`;
        navigationProcesses.style.color = '#f59e0b';
      } else {
        navigationProcesses.style.color = '#10b981';
      }
      processText += ` | PIDs: ${pids} | 运行时间: ${uptime}s`;
      navigationProcesses.textContent = processText;
    } else {
      navigationProcesses.textContent = `进程状态：无运行进程`;
      navigationProcesses.style.color = '#64748b';
    }
    
    // Update map info display
    updateMapInfoDisplay(payload);
  } catch (error) {
    console.warn('Failed to fetch navigation status', error);
  }
}

function updateMapInfoDisplay(payload) {
  const mapInfoContent = document.getElementById('map-info-content');
  
  if (!mapInfoContent) return;
  
  // Check if navigation is running and has map_info
  if (payload.status === 'RUNNING' && payload.map_info) {
    const mapInfo = payload.map_info;
    
    if (!mapInfo.valid) {
      // Map validation failed
      mapInfoContent.innerHTML = `
        <div style="color: #e74c3c;">
          <div>❌ 地图验证失败</div>
          <div style="margin-top: 0.3em;">${mapInfo.error || '未知错误'}</div>
          ${mapInfo.available_maps ? `<div style="margin-top: 0.3em; font-size: 0.9em;">可用: ${mapInfo.available_maps.slice(0, 3).join(', ')}...</div>` : ''}
        </div>
      `;
    } else {
      // Map validation successful - show map details
      const pcdSizeKB = mapInfo.pcd_size_kb ? mapInfo.pcd_size_kb.toFixed(1) : '?';
      const pcdFileName = mapInfo.pcd_file ? mapInfo.pcd_file.split('/').pop() : '?';
      
      mapInfoContent.innerHTML = `
        <div style="color: #2ecc71;">
          <div>✅ ${payload.map_name || '未知地图'}</div>
        </div>
        <div style="margin-top: 0.5em; color: #bdc3c7; font-size: 0.9em;">
          <div>📦 PCD: ${pcdFileName}</div>
          <div>📊 大小: ${pcdSizeKB} KB</div>
          ${mapInfo.description ? `<div style="margin-top: 0.3em; font-style: italic;">💬 ${mapInfo.description}</div>` : ''}
        </div>
      `;
    }
  } else if (payload.status === 'ERROR') {
    // Navigation error state
    mapInfoContent.innerHTML = `
      <div style="color: #e74c3c;">
        <div>❌ 导航错误</div>
        ${payload.error ? `<div style="margin-top: 0.3em; font-size: 0.9em;">${payload.error}</div>` : ''}
      </div>
    `;
  } else {
    // Navigation not running
    mapInfoContent.innerHTML = `<div style="color: #95a5a6;">未启动导航</div>`;
  }
}

async function fetchRobotPosition() {
  try {
    const response = await fetch('/api/robot/position');
    if (response.ok) {
      state.robotPosition = await response.json();
      // Debug: log first fetch only
      if (!window._posFetched) {
        console.log('fetchRobotPosition (first):', state.robotPosition, 'hasMap:', !!state.mapImage, 'hasMeta:', !!state.mapMeta);
        window._posFetched = true;
      }
      
      // Update robot coordinates display
      updateRobotCoordinatesDisplay();
      
      renderScene();  // Redraw map to show robot
    }
  } catch (error) {
    // Robot position not available, silently ignore
  }
}

function updateRobotCoordinatesDisplay() {
  const coordsElement = document.getElementById('robot-coords');
  const frameElement = document.getElementById('robot-frame');
  
  if (!coordsElement || !frameElement) return;
  
  if (state.robotPosition) {
    const pos = state.robotPosition;
    const yawDeg = (pos.yaw * 180 / Math.PI).toFixed(1);
    
    coordsElement.textContent = `X: ${pos.x.toFixed(3)} | Y: ${pos.y.toFixed(3)} | Yaw: ${yawDeg}°`;
    coordsElement.style.color = '#2ecc71';  // Green when data available
    
    frameElement.textContent = `Frame: ${pos.frame_id || 'unknown'}`;
    frameElement.style.color = '#3498db';  // Blue
  } else {
    coordsElement.textContent = 'X: -- | Y: -- | Yaw: --°';
    coordsElement.style.color = '#95a5a6';  // Gray when no data
    
    frameElement.textContent = 'Frame: unknown';
    frameElement.style.color = '#95a5a6';  // Gray
  }
}

function renderRouteList(routes) {
  routeList.innerHTML = '';
  routes.forEach(route => {
    const li = document.createElement('li');
    const title = document.createElement('span');
    title.textContent = `${route.name} · ${route.waypoints.length} 个航点`;

    const actions = document.createElement('div');
    actions.className = 'route-actions';

    const execBtn = document.createElement('button');
    execBtn.textContent = '执行';
    execBtn.addEventListener('click', () => executeRoute(route.id));

    const delBtn = document.createElement('button');
    delBtn.textContent = '删除';
    delBtn.addEventListener('click', () => deleteRoute(route.id));

    actions.append(execBtn, delBtn);
    li.append(title, actions);
    routeList.appendChild(li);
  });
}

function updateExecutionIndicator() {
  if (!executionIndicator) return;
  if (state.executingRoute) {
    executionIndicator.textContent = `执行中：${state.executingRoute}`;
    executionIndicator.classList.remove('hidden');
    if (stopExecutionBtn) stopExecutionBtn.classList.remove('hidden');
  } else {
    executionIndicator.textContent = '当前没有巡逻任务';
    executionIndicator.classList.add('hidden');
    if (stopExecutionBtn) stopExecutionBtn.classList.add('hidden');
  }
}

async function loadRoutes() {
  try {
    const response = await fetch('/api/routes');
    if (!response.ok) return;
    const payload = await response.json();
    renderRouteList(payload.routes ?? []);
  } catch (error) {
    console.warn('Failed to load routes', error);
  }
}

function computeTwistFromKeys() {
  const twist = { x: 0, y: 0, z: 0 };
  activeKeys.forEach(key => {
    const config = KEY_BINDINGS[key];
    if (!config) return;
    if (config.linear?.x) twist.x += config.linear.x;
    if (config.linear?.y) twist.y += config.linear.y;
    if (config.angular?.z) twist.z += config.angular.z;
  });
  twist.x = clamp(twist.x, -1, 1);
  twist.y = clamp(twist.y, -1, 1);
  twist.z = clamp(twist.z, -1, 1);
  return twist;
}

async function pushTwistCommand(twist) {
  try {
    await fetch('/api/robot/cmd_vel', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ vel_x: twist.x, vel_y: twist.y, vel_theta: twist.z }),
    });
  } catch (error) {
    console.warn('发送速度指令失败', error);
  }
}

function handleKeyDown(event) {
  if (event.repeat) return;
  if (event.target instanceof HTMLInputElement || event.target instanceof HTMLTextAreaElement) {
    return;
  }
  const key = event.key.toLowerCase();
  if (!KEY_BINDINGS[key]) return;
  activeKeys.add(key);
}

function handleKeyUp(event) {
  const key = event.key.toLowerCase();
  if (!KEY_BINDINGS[key]) return;
  activeKeys.delete(key);
}

function handleCanvasClick(event) {
  if (!state.mapMeta) {
    return;
  }
  const rect = mapCanvas.getBoundingClientRect();
  const scaleX = mapCanvas.width / rect.width;
  const scaleY = mapCanvas.height / rect.height;
  const canvasX = (event.clientX - rect.left) * scaleX;
  const canvasY = (event.clientY - rect.top) * scaleY;
  
  // Convert canvas coordinates (top-left origin, Y down) to world coordinates (bottom-left origin, Y up)
  const mapX = state.mapMeta.origin_x + canvasX * state.mapMeta.resolution;
  // Flip Y: Canvas Y increases downward, ROS2 Y increases upward
  const mapY = state.mapMeta.origin_y + (state.mapMeta.height - canvasY) * state.mapMeta.resolution;
  
  console.log('🎯 Canvas Click Coordinate Transform:', {
    '1_Screen': { clientX: event.clientX, clientY: event.clientY },
    '2_Canvas': { canvasX: canvasX.toFixed(1), canvasY: canvasY.toFixed(1) },
    '3_MapMeta': { 
      width: state.mapMeta.width, 
      height: state.mapMeta.height, 
      resolution: state.mapMeta.resolution,
      origin: [state.mapMeta.origin_x, state.mapMeta.origin_y]
    },
    '4_WorldCoords': { mapX: mapX.toFixed(3), mapY: mapY.toFixed(3) }
  });
  
  // Handle initial pose or nav goal setting mode with orientation
  if (state.settingInitialPose || state.settingNavGoal) {
    if (!state.settingPoseStart) {
      // First click: record position
      state.settingPoseStart = { x: mapX, y: mapY, canvasX, canvasY };
      notify('👆 现在拖动鼠标设置方向，再次点击确认');
      return;
    } else {
      // Second click: calculate orientation and confirm
      const dx = mapX - state.settingPoseStart.x;
      const dy = mapY - state.settingPoseStart.y;
      const yaw = Math.atan2(dy, dx);
      
      if (state.settingInitialPose) {
        setInitialPoseAtPosition(state.settingPoseStart.x, state.settingPoseStart.y, yaw);
      } else if (state.settingNavGoal) {
        setNavGoalAtPosition(state.settingPoseStart.x, state.settingPoseStart.y, yaw);
      }
      
      state.settingPoseStart = null;
      return;
    }
  }
  
  // Normal waypoint adding
  state.waypoints.push({
    pixelX: canvasX,
    pixelY: canvasY,
    x: Number(mapX.toFixed(3)),
    y: Number(mapY.toFixed(3)),
    yaw: 0,
    type: 'normal',
  });
  updateWaypointIndicator();
  renderScene();
}

// Add mouse move handler for orientation preview
function handleCanvasMouseMove(event) {
  if (!state.mapMeta) return;
  if (!state.settingPoseStart) return;
  if (!state.settingInitialPose && !state.settingNavGoal) return;
  
  const rect = mapCanvas.getBoundingClientRect();
  const scaleX = mapCanvas.width / rect.width;
  const scaleY = mapCanvas.height / rect.height;
  const canvasX = (event.clientX - rect.left) * scaleX;
  const canvasY = (event.clientY - rect.top) * scaleY;
  
  const mapX = state.mapMeta.origin_x + canvasX * state.mapMeta.resolution;
  const mapY = state.mapMeta.origin_y + (state.mapMeta.height - canvasY) * state.mapMeta.resolution;
  
  const dx = mapX - state.settingPoseStart.x;
  const dy = mapY - state.settingPoseStart.y;
  const yaw = Math.atan2(dy, dx);
  
  // Store temporary orientation for rendering
  state.tempOrientation = { 
    x: state.settingPoseStart.x, 
    y: state.settingPoseStart.y, 
    yaw 
  };
  
  renderScene();
}

async function saveCurrentRoute() {
  if (!state.waypoints.length) {
    notify('请先点击地图添加航点');
    return;
  }
  const payload = {
    name: inputs.routeName.value.trim() || `巡逻路线-${Date.now()}`,
    waypoints: state.waypoints.map(wp => ({ x: wp.x, y: wp.y, yaw: wp.yaw, type: wp.type })),
  };
  try {
    const response = await fetch('/api/routes', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    if (!response.ok) throw new Error('保存失败');
    notify('航点已保存');
    state.waypoints = [];
    updateWaypointIndicator();
    renderScene();
    await loadRoutes();
  } catch (error) {
    notify(`保存航点失败：${error}`);
  }
}

function clearWaypoints() {
  state.waypoints = [];
  updateWaypointIndicator();
  renderScene();
}

async function recordWaypoint() {
  try {
    const response = await fetch('/api/waypoints/record', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({type: 'normal'})
    });
    if (!response.ok) throw new Error('录制失败');
    const waypoint = await response.json();
    
    // Convert world coordinates to pixel coordinates (with Y flip)
    if (state.mapMeta) {
      waypoint.pixelX = (waypoint.x - state.mapMeta.origin_x) / state.mapMeta.resolution;
      waypoint.pixelY = (state.mapMeta.height - (waypoint.y - state.mapMeta.origin_y) / state.mapMeta.resolution);
    }
    
    state.waypoints.push(waypoint);
    updateWaypointIndicator();
    renderScene();
    notify('已录制航点');
  } catch (error) {
    notify(`录制航点失败：${error}`);
  }
}

async function startTrajectoryRecording() {
  try {
    await fetch('/api/trajectory/start', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({interval: 1.0})
    });
    state.trajectoryRecording = true;
    notify('开始录制轨迹');
  } catch (error) {
    notify(`开始录制失败：${error}`);
  }
}

async function stopTrajectoryRecording() {
  try {
    const response = await fetch('/api/trajectory/stop', {method: 'POST'});
    const data = await response.json();
    
    // Convert world coordinates to pixel coordinates (with Y flip)
    if (state.mapMeta) {
      data.waypoints.forEach(wp => {
        wp.pixelX = (wp.x - state.mapMeta.origin_x) / state.mapMeta.resolution;
        wp.pixelY = (state.mapMeta.height - (wp.y - state.mapMeta.origin_y) / state.mapMeta.resolution);
      });
    }
    
    state.waypoints = data.waypoints;
    state.trajectoryRecording = false;
    updateWaypointIndicator();
    renderScene();
    notify(`轨迹录制完成，共 ${data.count} 个航点`);
  } catch (error) {
    notify(`停止录制失败：${error}`);
  }
}

async function executeRoute(routeId) {
  try {
    const response = await fetch(`/api/routes/${routeId}/execute`, { method: 'POST' });
    if (!response.ok) throw new Error('执行失败');
    notify('巡逻任务已下发');
    state.executingRoute = routeId;
    updateExecutionIndicator();
  } catch (error) {
    notify(`执行巡逻失败：${error}`);
  }
}

async function stopExecution() {
  try {
    const response = await fetch('/api/navigation/cancel', { method: 'POST' });
    if (!response.ok) throw new Error('停止失败');
    notify('已停止执行巡逻任务');
    state.executingRoute = null;
    updateExecutionIndicator();
  } catch (error) {
    notify(`停止执行失败：${error}`);
  }
}

async function deleteRoute(routeId) {
  try {
    const response = await fetch(`/api/routes/${routeId}`, { method: 'DELETE' });
    if (!response.ok) throw new Error('删除失败');
    await loadRoutes();
    notify('巡逻路线已删除');
  } catch (error) {
    notify(`删除巡逻路线失败：${error}`);
  }
}

// Map management functions
async function loadMapList() {
  try {
    const response = await fetch('/api/maps/list');
    const data = await response.json();
    
    // Update select dropdown
    selects.mapName.innerHTML = '<option value="">-- 选择地图 --</option>';
    data.maps.forEach(map => {
      const option = document.createElement('option');
      option.value = map.name;
      option.textContent = `${map.name} (${(map.size / 1024).toFixed(1)} KB)`;
      selects.mapName.appendChild(option);
    });
  } catch (error) {
    console.warn('Failed to load map list', error);
  }
}

async function loadSelectedMap(mapName) {
  if (!mapName) {
    notify('请选择地图');
    return;
  }
  
  try {
    // Load map from disk
    const response = await fetch(`/api/maps/${mapName}/load`);
    if (!response.ok) throw new Error('地图加载失败');
    
    const payload = await response.json();
    // Check if map data is available and valid
    if (!payload.data || typeof payload.data !== 'string') {
      throw new Error('地图数据格式错误');
    }
    const data = Uint8Array.from(atob(payload.data), c => c.charCodeAt(0));
    const image = renderOccupancyGrid(data, payload.width, payload.height);
    
    state.mapImage = { image, width: payload.width, height: payload.height };
    state.mapMeta = {
      width: payload.width,
      height: payload.height,
      resolution: payload.resolution,
      origin_x: payload.origin_x,
      origin_y: payload.origin_y,
      origin: [payload.origin_x, payload.origin_y, 0]
    };
    
    state.costmapReady = false;
    renderScene();
    notify(`地图 "${mapName}" 已加载`);
  } catch (error) {
    notify(`加载地图失败：${error.message}`);
    console.error('Failed to load map:', error);
  }
}

async function deleteSelectedMap() {
  const mapName = selects.mapName.value;
  if (!mapName) {
    notify('请先选择要删除的地图');
    return;
  }
  
  if (!confirm(`确定要删除地图 "${mapName}" 吗？此操作不可恢复！`)) {
    return;
  }
  
  try {
    const response = await fetch(`/api/maps/${mapName}`, { method: 'DELETE' });
    if (!response.ok) throw new Error('删除失败');
    notify(`地图 "${mapName}" 已删除`);
    await loadMapList();
  } catch (error) {
    notify(`删除地图失败：${error}`);
  }
}

function getSelectedOrNewMapName() {
  // Use selected map if available, otherwise use new map name input
  const selected = selects.mapName.value;
  const newName = inputs.newMapName.value.trim();
  return selected || newName || null;
}

async function startMapping() {
  const mapName = inputs.newMapName.value.trim() || null;
  
  // Check if navigation is running
  try {
    const navStatus = await fetch('/api/navigation/status');
    const navData = await navStatus.json();
    if (navData.status === 'RUNNING') {
      notify('请先停止导航系统再开始建图');
      return;
    }
    // Auto-stop navigation if there are residual processes
    if (navData.processes && navData.processes.length > 0) {
      notify('检测到残留导航进程，正在清理...');
      await fetch('/api/navigation/stop', { method: 'POST' });
      await new Promise(resolve => setTimeout(resolve, 2000)); // Wait 2s
    }
  } catch (error) {
    console.warn('Failed to check navigation status', error);
  }
  
  try {
    const response = await fetch('/api/mapping/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ map_name: mapName }),
    });
    const payload = await response.json();
    if (!response.ok) throw new Error(payload.detail || '启动失败');
    mappingStatus.textContent = `建图状态：${payload.status}`;
    notify('建图模式已启动');
  } catch (error) {
    notify(`建图启动失败：${error}`);
  }
}

async function clearMappingCache() {
  if (!confirm('确定要清空建图缓存吗？这将停止所有建图进程并重置状态。')) {
    return;
  }
  
  try {
    notify('正在清空缓存...');
    const response = await fetch('/api/mapping/clear_cache', {
      method: 'POST',
    });
    
    if (response.ok) {
      const result = await response.json();
      notify(result.message || '缓存已清空');
      
      // Clear frontend map display
      state.mapImage = null;
      state.mapMeta = null;
      state.costmapReady = false;
      state.robotPosition = null;
      drawPlaceholder();
      
      // Refresh status
      await fetchMappingStatus();
    } else {
      const error = await response.json();
      notify(`清空失败：${error.detail || '未知错误'}`);
    }
  } catch (error) {
    notify(`清空失败：${error.message}`);
    console.error('Failed to clear cache:', error);
  }
}

async function stopMapping() {
  const mapName = inputs.newMapName.value.trim() || null;
  
  try {
    const response = await fetch('/api/mapping/stop', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ save: true, map_name: mapName }),
    });
    const payload = await response.json();
    mappingStatus.textContent = `建图状态：${payload.status}`;
    notify(payload.map_saved ? '建图已停止并保存' : '建图已停止');
    
    // Refresh map list after saving
    if (payload.map_saved) {
      await loadMapList();
    }
  } catch (error) {
    notify(`停止建图失败：${error}`);
  }
}

async function startNavigation() {
  const mapName = getSelectedOrNewMapName();
  
  if (!mapName) {
    notify('请先选择或输入地图名称');
    return;
  }
  
  // Check if mapping is running
  try {
    const mapStatus = await fetch('/api/mapping/status');
    const mapData = await mapStatus.json();
    if (mapData.status === 'RUNNING') {
      notify('请先停止建图系统再启动导航');
      return;
    }
    // Auto-clear mapping cache before starting navigation
    if (mapData.processes && mapData.processes.length > 0) {
      notify('检测到残留建图进程，正在清理...');
      await fetch('/api/mapping/clear_cache', { method: 'POST' });
      await new Promise(resolve => setTimeout(resolve, 2000)); // Wait 2s
    }
  } catch (error) {
    console.warn('Failed to check mapping status', error);
  }
  
  try {
    const response = await fetch('/api/navigation/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ map_name: mapName }),
    });
    const payload = await response.json();
    
    // Check if response indicates an error (even with 200 status)
    if (payload.status === 'ERROR') {
      const errorMsg = payload.error || payload.map_info?.error || '启动失败';
      notify(`❌ 导航启动失败: ${errorMsg}`);
      
      // Still update the display to show error details
      updateMapInfoDisplay(payload);
      navigationStatus.textContent = `导航状态：${payload.status}`;
      return;
    }
    
    if (!response.ok) throw new Error(payload.detail || '启动失败');
    
    navigationStatus.textContent = `导航状态：${payload.status}`;
    
    // Update map info display immediately
    updateMapInfoDisplay(payload);
    
    // Show success notification with map info
    if (payload.map_info && payload.map_info.valid) {
      const pcdSizeKB = payload.map_info.pcd_size_kb ? payload.map_info.pcd_size_kb.toFixed(1) : '?';
      notify(`✅ 导航系统已启动\n📦 地图: ${mapName} (${pcdSizeKB} KB)`);
    } else {
      notify('导航系统已启动');
    }
    
    // Auto-load the map to frontend for visualization and coordinate conversion
    await loadSelectedMap(mapName);
  } catch (error) {
    notify(`导航启动失败：${error}`);
  }
}

async function stopNavigation() {
  try {
    const response = await fetch('/api/navigation/stop', {
      method: 'POST',
    });
    const payload = await response.json();
    if (!response.ok) throw new Error(payload.detail || '停止失败');
    navigationStatus.textContent = `导航状态：${payload.status}`;
    
    // Clear map info display
    const mapInfoContent = document.getElementById('map-info-content');
    if (mapInfoContent) {
      mapInfoContent.innerHTML = `<div style="color: #95a5a6;">未启动导航</div>`;
    }
    
    notify('导航系统已停止');
  } catch (error) {
    notify(`导航停止失败：${error}`);
  }
}

async function sendRobotMode(path, body) {
  try {
    const response = await fetch(path, {
      method: 'POST',
      headers: body ? { 'Content-Type': 'application/json' } : undefined,
      body: body ? JSON.stringify(body) : undefined,
    });
    if (!response.ok) throw new Error('命令失败');
    notify('指令已发送');
  } catch (error) {
    notify(`指令发送失败：${error}`);
  }
}

async function sendHeightAdjust(direction) {
  try {
    const response = await fetch('/api/robot/body_height', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ direction: direction }),
    });
    if (!response.ok) throw new Error('高度调节失败');
    const action = direction > 0 ? '升高' : '降低';
    notify(`✅ 机身${action} 5cm`);
  } catch (error) {
    notify(`高度调节失败：${error}`);
  }
}

function enableInitialPoseMode() {
  if (!state.mapMeta) {
    notify('请先加载地图');
    return;
  }
  state.settingInitialPose = true;
  state.settingPoseStart = null;  // 用于记录起始点
  mapCanvas.style.cursor = 'crosshair';
  notify('🎯 点击地图设置位置，然后拖动鼠标设置方向（箭头指向前方）');
}

async function triggerGlobalRelocalization() {
  try {
    notify('🌍 触发全局重定位...');
    
    // 发送(0,0,0)表示无先验，让localizer使用Scan Context全局定位
    const response = await fetch('/api/robot/set_initial_pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        x: 0.0,  // 原点表示"无先验"，触发Scan Context
        y: 0.0,
        z: 0.0,
        yaw: 0.0
      })
    });
    
    if (response.ok) {
      const result = await response.json();
      notify('✅ 全局重定位已触发，等待Scan Context定位...');
      
      // 等待几秒后刷新机器人位置
      const refreshAttempts = [1000, 2000, 3000, 5000]; // ms
      refreshAttempts.forEach(delay => {
        setTimeout(() => fetchRobotPosition(), delay);
      });
    } else {
      const error = await response.json();
      notify(`❌ 全局重定位失败：${error.detail || '未知错误'}`);
    }
  } catch (error) {
    console.error('全局重定位失败:', error);
    notify(`❌ 全局重定位失败：${error.message}`);
  }
}

async function setInitialPoseAtPosition(x, y, yaw = 0.0) {
  try {
    const response = await fetch('/api/robot/set_initial_pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y, z: 0.0, yaw })
    });
    
    if (response.ok) {
      const result = await response.json();
      const yawDeg = (yaw * 180 / Math.PI).toFixed(1);
      notify(result.message || `初始位姿已设置 (${x.toFixed(2)}, ${y.toFixed(2)}, ${yawDeg}°)，等待定位收敛...`);
      
      // Store initial pose for visualization
      state.initialPose = { x, y, yaw };
      
      // Exit initial pose setting mode
      state.settingInitialPose = false;
      state.tempOrientation = null;
      mapCanvas.style.cursor = 'default';
      
      // Re-render to show the initial pose arrow
      renderScene();
      
      // Force refresh robot position multiple times to catch the update
      const refreshAttempts = [500, 1000, 2000, 3000, 5000]; // ms
      refreshAttempts.forEach(delay => {
        setTimeout(() => fetchRobotPosition(), delay);
      });
    } else {
      const error = await response.json();
      notify(`设置失败：${error.detail || '未知错误'}`);
    }
  } catch (error) {
    notify(`设置失败：${error.message}`);
    console.error('Failed to set initial pose:', error);
  }
}

function enableNavGoalMode() {
  if (!state.mapMeta) {
    notify('请先加载地图');
    return;
  }
  state.settingNavGoal = true;
  state.settingPoseStart = null;  // 用于记录起始点
  mapCanvas.style.cursor = 'crosshair';
  notify('🎯 点击地图设置目标，然后拖动鼠标设置方向（箭头指向前方）');
}

async function setNavGoalAtPosition(x, y, yaw = 0.0) {
  console.log('🚀 Sending Navigation Goal to ROS2:', { x: x.toFixed(3), y: y.toFixed(3), yaw });
  
  try {
    const response = await fetch('/api/robot/navigation_goal', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ goal_x: x, goal_y: y, goal_theta: yaw })
    });
    
    if (response.ok) {
      const result = await response.json();
      const yawDeg = (yaw * 180 / Math.PI).toFixed(1);
      notify(result.message || `导航目标已设置 (${x.toFixed(2)}, ${y.toFixed(2)}, ${yawDeg}°)`);
      
      // Store nav goal for visualization
      state.navGoal = { x, y, yaw };
      
      console.log('✅ Nav Goal Set Successfully. Will redraw at:', {
        worldX: x.toFixed(3),
        worldY: y.toFixed(3),
        yaw: yawDeg + '°',
        note: '(This should match the blue flag position on map)'
      });
      
      // Exit nav goal setting mode
      state.settingNavGoal = false;
      state.tempOrientation = null;
      mapCanvas.style.cursor = 'default';
      
      // Re-render to show the nav goal flag
      renderScene();
    } else {
      const error = await response.json();
      notify(`设置失败：${error.detail || '未知错误'}`);
    }
  } catch (error) {
    notify(`设置失败：${error.message}`);
    console.error('Failed to set nav goal:', error);
  }
}

async function cancelNavGoal() {
  try {
    const response = await fetch('/api/navigation/cancel', {
      method: 'POST',
    });
    if (response.ok) {
      notify('已取消导航目标');
      // Clear nav goal visualization
      state.navGoal = null;
      renderScene();
    } else {
      const error = await response.json();
      notify(`取消失败：${error.detail || '未知错误'}`);
    }
  } catch (error) {
    notify(`取消失败：${error.message}`);
    console.error('Failed to cancel nav goal:', error);
  }
}

function initEventBindings() {
  document.addEventListener('keydown', handleKeyDown);
  document.addEventListener('keyup', handleKeyUp);
  mapCanvas.addEventListener('click', handleCanvasClick);
  mapCanvas.addEventListener('mousemove', handleCanvasMouseMove);

  buttons.refreshMap?.addEventListener('click', fetchMap);
  buttons.refreshCostmap?.addEventListener('click', () => fetchCostmap(state.selectedCostmap));
  buttons.refreshMaps?.addEventListener('click', loadMapList);
  buttons.deleteMap?.addEventListener('click', deleteSelectedMap);

  // Edit map button
  document.getElementById('btn-edit-map')?.addEventListener('click', () => {
    const mapName = selects.mapName.value;
    if (mapName) {
      window.location.href = `/static/map_editor.html?map=${encodeURIComponent(mapName)}`;
    } else {
      notify('请先选择一个地图！', 'warning');
    }
  });
  buttons.startMapping?.addEventListener('click', startMapping);
  buttons.stopMapping?.addEventListener('click', stopMapping);
  buttons.clearCache?.addEventListener('click', clearMappingCache);
  buttons.startNavigation?.addEventListener('click', startNavigation);
  buttons.stopNavigation?.addEventListener('click', stopNavigation);
  buttons.cancelNavGoal?.addEventListener('click', cancelNavGoal);
  buttons.stopNavSystem?.addEventListener('click', stopNavigation);
  buttons.setInitialPose?.addEventListener('click', enableInitialPoseMode);
  buttons.triggerGlobalReloc?.addEventListener('click', triggerGlobalRelocalization);
  buttons.setNavGoal?.addEventListener('click', enableNavGoalMode);
  buttons.saveRoute?.addEventListener('click', saveCurrentRoute);
  buttons.clearWaypoints?.addEventListener('click', clearWaypoints);
  buttons.recordWaypoint?.addEventListener('click', recordWaypoint);
  buttons.startTrajectory?.addEventListener('click', startTrajectoryRecording);
  buttons.stopTrajectory?.addEventListener('click', stopTrajectoryRecording);
  stopExecutionBtn?.addEventListener('click', stopExecution);
  buttons.stand?.addEventListener('click', () => sendRobotMode('/api/robot/mode/stand'));
  buttons.walk?.addEventListener('click', () => sendRobotMode('/api/robot/mode/walk'));
  buttons.sit?.addEventListener('click', () => sendRobotMode('/api/robot/mode/sit'));
  buttons.recover?.addEventListener('click', () => sendRobotMode('/api/robot/mode/recover'));
  buttons.stairOn?.addEventListener('click', () => sendRobotMode('/api/robot/mode/stair', { enable: true }));
  buttons.stairOff?.addEventListener('click', () => sendRobotMode('/api/robot/mode/stair', { enable: false }));
  buttons.heightUp?.addEventListener('click', () => sendHeightAdjust(1));
  buttons.heightDown?.addEventListener('click', () => sendHeightAdjust(-1));
  buttons.emgStop?.addEventListener('click', () => sendRobotMode('/api/robot/emergency_stop'));

  selects.costmapKind?.addEventListener('change', event => {
    state.selectedCostmap = event.target.value;
    fetchCostmap(state.selectedCostmap);
  });
  
  selects.mapName?.addEventListener('change', event => {
    const mapName = event.target.value;
    if (mapName) {
      loadSelectedMap(mapName);
    }
  });
}

function startTwistLoop() {
  setInterval(() => {
    if (sendingTwist) {
      return;
    }
    const twist = computeTwistFromKeys();
    const changed = twist.x !== lastTwist.x || twist.y !== lastTwist.y || twist.z !== lastTwist.z;
    if (!changed && twist.x === 0 && twist.y === 0 && twist.z === 0) {
      return;
    }
    lastTwist = twist;
    sendingTwist = true;
    pushTwistCommand(twist).finally(() => {
      sendingTwist = false;
    });
  }, 1000 / 30);
}

// ============ 虚拟摇杆控制 ============
class VirtualJoystick {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
    this.infoDisplay = document.getElementById('joystick-info');
    this.maxDistance = 60; // 摇杆最大偏移距离（像素）
    this.isDragging = false;
    this.centerX = 0;
    this.centerY = 0;
    this.currentX = 0;
    this.currentY = 0;
    
    // 速度参数（基础速度）
    this.baseLinearSpeed = 0.8;  // m/s
    this.baseAngularSpeed = 0.8; // rad/s
    
    // 当前速度上限（会根据楼梯模式动态调整）
    this.maxLinearSpeed = 0.8;
    this.maxAngularSpeed = 0.8;
    
    // 目标速度和当前速度（用于平滑插值）
    this.targetLinearVel = 0;
    this.targetAngularVel = 0;
    this.linearVel = 0;
    this.angularVel = 0;
    
    // 平滑参数
    this.smoothingFactor = 0.25;  // 平滑系数（0-1，越大响应越快）
    
    this.init();
    this.startVelocityLoop();
  }
  
  init() {
    // 创建摇杆元素
    this.base = document.createElement('div');
    this.base.className = 'joystick-base';
    
    this.stick = document.createElement('div');
    this.stick.className = 'joystick-stick';
    
    this.container.appendChild(this.base);
    this.container.appendChild(this.stick);
    
    // 绑定事件
    this.stick.addEventListener('mousedown', this.onStart.bind(this));
    this.stick.addEventListener('touchstart', this.onStart.bind(this), { passive: false });
    
    document.addEventListener('mousemove', this.onMove.bind(this));
    document.addEventListener('touchmove', this.onMove.bind(this), { passive: false });
    
    document.addEventListener('mouseup', this.onEnd.bind(this));
    document.addEventListener('touchend', this.onEnd.bind(this));
    
    // 启动速度发送循环
    this.startVelocityLoop();
  }
  
  onStart(e) {
    e.preventDefault();
    this.isDragging = true;
    
    const rect = this.container.getBoundingClientRect();
    this.centerX = rect.left + rect.width / 2;
    this.centerY = rect.top + rect.height / 2;
  }
  
  onMove(e) {
    if (!this.isDragging) return;
    e.preventDefault();
    
    let clientX, clientY;
    if (e.type.startsWith('touch')) {
      clientX = e.touches[0].clientX;
      clientY = e.touches[0].clientY;
    } else {
      clientX = e.clientX;
      clientY = e.clientY;
    }
    
    // 计算偏移
    let deltaX = clientX - this.centerX;
    let deltaY = clientY - this.centerY;
    
    // 限制在最大距离内
    const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    if (distance > this.maxDistance) {
      const angle = Math.atan2(deltaY, deltaX);
      deltaX = Math.cos(angle) * this.maxDistance;
      deltaY = Math.sin(angle) * this.maxDistance;
    }
    
    this.currentX = deltaX;
    this.currentY = deltaY;
    
    // 更新摇杆位置
    this.stick.style.transform = `translate(calc(-50% + ${deltaX}px), calc(-50% + ${deltaY}px))`;
    
    // 计算速度（Y轴向上为正，对应前进；X轴向右为正，对应左转）
    this.linearVel = -(deltaY / this.maxDistance) * this.maxLinearSpeed;  // 向上推为前进
    this.angularVel = -(deltaX / this.maxDistance) * this.maxAngularSpeed; // 向左推为左转
    
    // 更新显示
    this.updateInfo();
  }
  
  onEnd(e) {
    if (!this.isDragging) return;
    this.isDragging = false;
    
    // 重置摇杆位置
    this.stick.style.transform = 'translate(-50%, -50%)';
    this.currentX = 0;
    this.currentY = 0;
    this.linearVel = 0;
    this.angularVel = 0;
    
    // 更新显示
    this.updateInfo();
  }
  
  updateInfo() {
    if (this.infoDisplay) {
      this.infoDisplay.innerHTML = `
        线速度: ${this.linearVel.toFixed(2)} m/s<br>
        角速度: ${this.angularVel.toFixed(2)} rad/s
      `;
    }
  }
  
  // 根据机器人模式更新速度上限
  updateSpeedLimits(robotStatus) {
    // 检查是否为楼梯模式（STAIR）
    const isStairMode = robotStatus && 
                        (robotStatus.toUpperCase().includes('STAIR') || 
                         robotStatus.toUpperCase().includes('楼梯'));
    
    if (isStairMode) {
      // 楼梯模式：速度提高一倍
      this.maxLinearSpeed = this.baseLinearSpeed * 2;
      this.maxAngularSpeed = this.baseAngularSpeed * 2;
    } else {
      // 普通模式：使用基础速度
      this.maxLinearSpeed = this.baseLinearSpeed;
      this.maxAngularSpeed = this.baseAngularSpeed;
    }
  }
  
  startVelocityLoop() {
    setInterval(() => {
      if (this.isDragging || this.linearVel !== 0 || this.angularVel !== 0) {
        const twist = {
          x: this.linearVel,
          y: 0,
          z: this.angularVel
        };
        
        // 发送速度命令
        pushTwistCommand(twist).catch(err => {
          console.error('Failed to send joystick velocity:', err);
        });
        
        // 如果摇杆已松开，清零速度
        if (!this.isDragging) {
          this.linearVel = 0;
          this.angularVel = 0;
        }
      }
    }, 1000 / 20); // 20Hz
  }
}

// 初始化虚拟摇杆
let joystick = null;
if (document.getElementById('joystick-container')) {
  joystick = new VirtualJoystick('joystick-container');
}

// ===== Localizer Log Functions =====
async function fetchLocalizerLog() {
  try {
    const res = await fetch('/api/localizer/logs');
    const data = await res.json();
    const logDiv = document.getElementById('localizer-log');
    
    if (data.logs && data.logs.length > 0) {
      logDiv.innerHTML = data.logs.map(log => {
        let color = '#bdc3c7';  // default gray
        if (log.includes('成功') || log.includes('ICP匹配成功') || log.includes('定位成功') || log.includes('Match Found')) {
          color = '#2ecc71';  // green - 成功
        } else if (log.includes('失败') || log.includes('过高') || log.includes('定位失败')) {
          color = '#e74c3c';  // red - 失败
        } else if (log.includes('SC') || log.includes('Scan Context')) {
          color = '#1abc9c';  // teal - SC相关
        } else if (log.includes('TEASER') || log.includes('对应关系')) {
          color = '#e67e22';  // orange - TEASER++
        } else if (log.includes('粗匹配')) {
          color = '#3498db';  // blue - 粗匹配
        } else if (log.includes('精匹配')) {
          color = '#9b59b6';  // purple - 精匹配
        } else if (log.includes('全局配准') || log.includes('图匹配')) {
          color = '#f39c12';  // orange - 全局配准
        }
        return `<div style="color: ${color}; margin-bottom: 0.2em;">${log}</div>`;
      }).join('');
      // Only auto scroll if enabled
      if (localizerAutoScrollEnabled) {
        logDiv.scrollTop = logDiv.scrollHeight;
      }
    } else {
      logDiv.innerHTML = '<div>等待定位数据...</div>';
    }
  } catch (err) {
    console.error('Failed to fetch localizer log:', err);
  }
}

function clearLocalizerLog() {
  document.getElementById('localizer-log').innerHTML = '<div>日志已清除</div>';
}

function toggleLocalizerAutoScroll() {
  localizerAutoScrollEnabled = !localizerAutoScrollEnabled;
  const btn = document.getElementById('btn-toggle-localizer-autoscroll');
  if (btn) {
    if (localizerAutoScrollEnabled) {
      btn.style.backgroundColor = '#27ae60';
      btn.textContent = '✓ 自动滚动';
      // Scroll to bottom immediately
      const logDiv = document.getElementById('localizer-log');
      if (logDiv) {
        logDiv.scrollTop = logDiv.scrollHeight;
      }
    } else {
      btn.style.backgroundColor = '#95a5a6';
      btn.textContent = '✗ 自动滚动';
    }
  }
}

// ===== Navigation Log Functions =====
async function fetchNavigationLog() {
  try {
    const navLogDiv = document.getElementById('navigation-log');
    const navStatus = state.robot?.navigation?.status || '未知';
    const lastGoal = state.navGoal;
    
    let logs = [];
    logs.push(`<div style="color: #3498db;">导航状态: ${navStatus}</div>`);
    
    if (lastGoal) {
      logs.push(`<div style="color: #2ecc71;">目标位置: (${lastGoal.x.toFixed(2)}, ${lastGoal.y.toFixed(2)})</div>`);
    }
    
    if (state.robot?.navigation) {
      const nav = state.robot.navigation;
      if (nav.error_code) {
        logs.push(`<div style="color: #e74c3c;">错误代码: ${nav.error_code}</div>`);
      }
      if (nav.distance_remaining !== undefined) {
        logs.push(`<div style="color: #f39c12;">剩余距离: ${nav.distance_remaining.toFixed(2)} m</div>`);
      }
    }
    
    navLogDiv.innerHTML = logs.join('') || '<div>等待导航数据...</div>';
  } catch (err) {
    console.error('Failed to fetch navigation log:', err);
  }
}

function clearNavigationLog() {
  document.getElementById('navigation-log').innerHTML = '<div>日志已清除</div>';
}

// ===== Sensor Log Functions =====
function updateSensorLog() {
  try {
    const sensorLogDiv = document.getElementById('sensor-log');
    const sensorData = state.robot?.robot_info;
    
    let logs = [];
    
    if (state.robotPosition) {
      logs.push(`<div style="color: #2ecc71;">✓ 定位: 正常 (${state.robotPosition.frame_id})</div>`);
    } else {
      logs.push(`<div style="color: #e74c3c;">✗ 定位: 无数据</div>`);
    }
    
    if (sensorData) {
      if (sensorData.imu) {
        logs.push(`<div style="color: #3498db;">✓ IMU: 正常</div>`);
      }
      if (sensorData.lidar) {
        logs.push(`<div style="color: #3498db;">✓ LiDAR: 正常</div>`);
      }
    }
    
    logs.push(`<div style="color: #95a5a6;">更新时间: ${new Date().toLocaleTimeString()}</div>`);
    
    sensorLogDiv.innerHTML = logs.join('') || '<div>等待传感器数据...</div>';
  } catch (err) {
    console.error('Failed to update sensor log:', err);
  }
}

// Bind localizer log buttons
document.getElementById('btn-refresh-localizer-log')?.addEventListener('click', fetchLocalizerLog);
document.getElementById('btn-clear-localizer-log')?.addEventListener('click', clearLocalizerLog);
document.getElementById('btn-toggle-localizer-autoscroll')?.addEventListener('click', toggleLocalizerAutoScroll);

// Bind navigation log buttons
document.getElementById('btn-refresh-navigation-log')?.addEventListener('click', fetchNavigationLog);
document.getElementById('btn-clear-navigation-log')?.addEventListener('click', clearNavigationLog);

// Bind point cloud checkbox
document.getElementById('checkbox-show-pointcloud')?.addEventListener('change', (e) => {
  state.showPointCloud = e.target.checked;
  if (state.showPointCloud && !state.pointCloudImage) {
    // Fetch point cloud if not already loaded
    fetchPointCloud();
  }
  renderScene();
});

// ===== Point Cloud Functions =====
async function fetchPointCloud() {
  try {
    // Fetch 2D laser scan points from backend
    const response = await fetch('/api/robot/scan_points');
    if (!response.ok) {
      console.log('Scan points not available (robot not localized or no scan data)');
      return;
    }
    
    const payload = await response.json();
    
    if (!payload.points || !state.mapMeta) {
      console.log('No scan points or map metadata available');
      return;
    }
    
    console.log(`📊 Rendering ${payload.points.length} scan points in map frame`);
    
    // Create an overlay canvas to draw scan points
    const width = state.mapMeta.width;
    const height = state.mapMeta.height;
    const image = ctx.createImageData(width, height);
    const pixels = image.data;
    
    // Make everything transparent first
    for (let i = 0; i < pixels.length; i += 4) {
      pixels[i + 3] = 0;  // Alpha = 0 (transparent)
    }
    
    // Convert each point from map coordinates to pixel coordinates and draw
    const resolution = state.mapMeta.resolution;
    const origin_x = state.mapMeta.origin_x;
    const origin_y = state.mapMeta.origin_y;
    
    for (const [px_map, py_map] of payload.points) {
      // Convert map coordinates to pixel coordinates
      const pixelX = Math.floor((px_map - origin_x) / resolution);
      const pixelY = Math.floor(height - (py_map - origin_y) / resolution);
      
      // Check bounds
      if (pixelX >= 0 && pixelX < width && pixelY >= 0 && pixelY < height) {
        const idx = (pixelY * width + pixelX) * 4;
        
        // Render scan points in bright cyan/lime green
        pixels[idx] = 0;       // R
        pixels[idx + 1] = 255; // G (bright green)
        pixels[idx + 2] = 255; // B (cyan)
        pixels[idx + 3] = 200; // A (semi-transparent)
        
        // Also draw adjacent pixels for better visibility
        for (let dy = -1; dy <= 1; dy++) {
          for (let dx = -1; dx <= 1; dx++) {
            const nx = pixelX + dx;
            const ny = pixelY + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
              const nidx = (ny * width + nx) * 4;
              if (pixels[nidx + 3] === 0) {  // Only draw if not already drawn
                pixels[nidx] = 0;
                pixels[nidx + 1] = 255;
                pixels[nidx + 2] = 255;
                pixels[nidx + 3] = 150;  // Slightly more transparent for adjacent pixels
              }
            }
          }
        }
      }
    }
    
    state.pointCloudImage = image;
    renderScene();
  } catch (error) {
    console.log('Point cloud fetch failed:', error.message);
  }
}

updateWaypointIndicator();
initEventBindings();
drawPlaceholder();

// Verify notification system on page load
(function() {
  const container = document.getElementById('notification-container');
  if (!container) {
    console.error('❌ Notification container missing! Creating fallback...');
    const fallback = document.createElement('div');
    fallback.id = 'notification-container';
    fallback.className = 'notification-container';
    document.body.insertBefore(fallback, document.body.firstChild);
  } else {
    console.log('✅ Notification system ready');
  }
})();

probeBackend();
fetchRobotStatus();
fetchMappingStatus();
fetchSensorStatus();
fetchMap();
loadRoutes();
loadMapList();
startTwistLoop();
scheduleCostmapRefresh();
fetchLocalizerLog();  // Initial fetch
fetchNavigationLog();  // Initial fetch
updateSensorLog();  // Initial sensor log

setInterval(probeBackend, 10000);
setInterval(fetchRobotStatus, 5000);
setInterval(fetchMappingStatus, 5000);
setInterval(fetchNavigationStatus, 5000);
setInterval(fetchSensorStatus, 2000);
setInterval(fetchRobotPosition, 200);  // 5Hz update for robot position
setInterval(loadRoutes, 15000);
setInterval(fetchLocalizerLog, 2000);  // Update localizer log every 2 seconds
setInterval(fetchNavigationLog, 3000);  // Update navigation log every 3 seconds
setInterval(updateSensorLog, 3000);  // Update sensor log every 3 seconds
setInterval(() => {
  if (state.showPointCloud) {
    fetchPointCloud();  // Refresh point cloud if enabled
  }
}, 5000);


// ===== Storage Management =====
const storage = {
  diskText: document.getElementById('disk-usage-text'),
  diskBar: document.getElementById('disk-usage-bar'),
  bagList: document.getElementById('rosbag-list'),
  btnCleanup: document.getElementById('btn-cleanup-rosbags')
};

async function updateStorageInfo() {
  if (!storage.diskText) return;
  try {
    const res = await fetch('/api/system/storage');
    const data = await res.json();
    storage.diskText.textContent = `${data.used_gb}GB / ${data.total_gb}GB (${data.percent}%)`;
    storage.diskBar.style.width = `${data.percent}%`;
    
    if (data.percent > 90) {
       storage.diskBar.style.backgroundColor = '#e74c3c'; // Red
       // Throttle notification? For now simple
       // notify("⚠️ 磁盘空间不足！请清理旧数据");
    } else if (data.percent > 75) {
       storage.diskBar.style.backgroundColor = '#f39c12'; // Orange
    } else {
       storage.diskBar.style.backgroundColor = '#2ecc71'; // Green
    }
  } catch (e) {
    console.error(e);
  }
}

async function updateRosbagList() {
  if (!storage.bagList) return;
  try {
    const res = await fetch('/api/rosbags');
    const bags = await res.json();
    
    if (bags.length === 0) {
       storage.bagList.innerHTML = '<div style="color: #bdc3c7; text-align: center; padding: 10px;">无残留录包</div>';
       return;
    }
    
    storage.bagList.innerHTML = bags.map(bag => `
      <div style="display: flex; justify-content: space-between; align-items: center; padding: 5px; border-bottom: 1px solid #34495e;">
        <div>
           <div style="color: #ecf0f1;">${bag.name}</div>
           <div style="color: #95a5a6; font-size: 0.9em;">${bag.size_mb} MB | ${bag.modified_time}</div>
        </div>
        ${!bag.is_active ? `<button onclick="deleteRosbag('${bag.name}')" style="background: #e74c3c; border: none; color: white; padding: 2px 6px; border-radius: 3px; cursor: pointer;">🗑️</button>` : '<span style="color: #2ecc71; font-size: 0.8em;">[录制中]</span>'}
      </div>
    `).join('');
  } catch (e) {
    console.error(e);
  }
}

window.deleteRosbag = async (name) => {
    if(!confirm(`确定要删除录包 ${name} 吗？`)) return;
    try {
        const res = await fetch(`/api/rosbags/${name}`, { method: 'DELETE' });
        if(res.ok) {
            updateRosbagList();
            updateStorageInfo();
            notify(`已删除 ${name}`);
        } else {
            const err = await res.json();
            notify(`删除失败: ${err.detail}`);
        }
    } catch(e) {
        notify(`错误: ${e}`);
    }
};

if (storage.btnCleanup) {
    storage.btnCleanup.addEventListener('click', async () => {
        if(!confirm("确定要清理所有超过 7 天的旧录包吗？")) return;
        try {
            const res = await fetch('/api/rosbags/cleanup?days=7', { method: 'POST' });
            const data = await res.json();
            notify(`清理完成，删除了 ${data.deleted_count} 个录包`);
            updateRosbagList();
            updateStorageInfo();
        } catch(e) {
            notify(`清理错误: ${e}`);
        }
    });
}

// Initial load
updateStorageInfo();
updateRosbagList();

// Periodic update
setInterval(() => {
    updateStorageInfo();
    updateRosbagList();
}, 10000);
