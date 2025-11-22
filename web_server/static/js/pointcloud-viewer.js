// 3D点云可视化模块 - 展示匹配过程（source & target + 对应关系连线）
let scene, camera, renderer, controls;
let teaserSourceCloud, teaserTargetCloud;
let teaserCorrespondenceLines;  // 对应关系连线
let roughSourceCloud, roughTargetCloud;
let refineSourceCloud, refineTargetCloud;
let animationId;

// 获取DOM元素
const modal = document.getElementById('pointcloud-modal');
const showBtn = document.getElementById('btn-show-3d-viewer');
const closeBtn = document.getElementById('btn-close-modal');
const refreshBtn = document.getElementById('btn-refresh-clouds');
const resetCameraBtn = document.getElementById('btn-reset-camera');
const triggerGlobalRelocBtn = document.getElementById('btn-trigger-global-relocalization');
const toggleAutoRefreshBtn = document.getElementById('btn-toggle-pointcloud-autorefresh');
const container = document.getElementById('three-container');
const cloudInfo = document.getElementById('cloud-info');
const pointcloudStatus = document.getElementById('pointcloud-status');

// 自动刷新控制
let autoRefreshEnabled = false;
let autoRefreshInterval = null;

// 复选框 - 改为显示source和target
const showTeaserCheckbox = document.getElementById('show-teaser');
const showRoughCheckbox = document.getElementById('show-rough');
const showRefineCheckbox = document.getElementById('show-refine');

// 统计元素 - 会显示source/target点数
const statTeaser = document.getElementById('stat-teaser');
const statRough = document.getElementById('stat-rough');
const statRefine = document.getElementById('stat-refine');

// 初始化Three.js场景
function initThreeJS() {
    // 创建场景
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a1a);
    
    // 创建相机
    const width = container.clientWidth;
    const height = container.clientHeight;
    camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.set(5, 5, 5);
    camera.lookAt(0, 0, 0);
    
    // 创建渲染器
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    container.appendChild(renderer.domElement);
    
    // 添加轨道控制器
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.minDistance = 1;
    controls.maxDistance = 100;
    
    // 添加坐标轴辅助器
    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);
    
    // 添加网格地面
    const gridHelper = new THREE.GridHelper(20, 20, 0x555555, 0x333333);
    scene.add(gridHelper);
    
    // 添加环境光和方向光
    const ambientLight = new THREE.AmbientLight(0x404040, 1.5);
    scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 10);
    scene.add(directionalLight);
    
    // 启动渲染循环
    animate();
    
    console.log('Three.js场景初始化完成');
}

// 动画循环
function animate() {
    animationId = requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// 停止动画
function stopAnimation() {
    if (animationId) {
        cancelAnimationFrame(animationId);
        animationId = null;
    }
}

// 创建点云对象
function createPointCloud(points, color, name) {
    if (!points || points.length === 0) {
        console.warn(`${name}: 点云数据为空`);
        return null;
    }
    
    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(points.length * 3);
    
    for (let i = 0; i < points.length; i++) {
        positions[i * 3] = points[i].x;
        positions[i * 3 + 1] = points[i].z;  // 注意：Z轴向上
        positions[i * 3 + 2] = -points[i].y; // Y轴反向
    }
    
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    
    const material = new THREE.PointsMaterial({
        color: color,
        size: 0.05,
        sizeAttenuation: true
    });
    
    const pointCloud = new THREE.Points(geometry, material);
    pointCloud.name = name;
    
    console.log(`${name}: 创建了 ${points.length} 个点`);
    return pointCloud;
}

// 创建对应关系连线
function createCorrespondenceLines(correspondences) {
    if (!correspondences || correspondences.length === 0) {
        console.warn('对应关系数据为空');
        return null;
    }
    
    const geometry = new THREE.BufferGeometry();
    const positions = [];
    const colors = [];
    
    for (const corr of correspondences) {
        // Source点
        positions.push(corr.src.x, corr.src.z, -corr.src.y);
        // Target点
        positions.push(corr.tgt.x, corr.tgt.z, -corr.tgt.y);
        
        // 每个端点都需要颜色
        colors.push(corr.color.r, corr.color.g, corr.color.b);
        colors.push(corr.color.r, corr.color.g, corr.color.b);
    }
    
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    
    const material = new THREE.LineBasicMaterial({
        vertexColors: true,
        linewidth: 2,
        opacity: 0.6,
        transparent: true
    });
    
    const lines = new THREE.LineSegments(geometry, material);
    lines.name = 'TEASER++_Correspondences';
    
    console.log(`创建了 ${correspondences.length} 对对应关系连线`);
    return lines;
}

// 从API获取点云数据
// 从API获取点云数据 - 展示source和target
async function fetchPointClouds() {
    try {
        pointcloudStatus.textContent = '点云状态：正在加载...';
        
        // 如果模态窗口还未打开，只更新状态
        if (!scene) {
            const response = await fetch('/api/robot/matching_clouds');
            const data = await response.json();
            
            if (data.Result !== 0) {
                throw new Error(data.Error || '获取点云数据失败');
            }
            
            let hasData = false;
            let statusText = '';
            if (data.teaser_source && data.teaser_source.count > 0) {
                hasData = true;
                statusText += `TEASER:${data.teaser_source.count}+${data.teaser_target.count} `;
            }
            if (data.rough_source && data.rough_source.count > 0) {
                hasData = true;
                statusText += `粗:${data.rough_source.count}+${data.rough_target.count} `;
            }
            if (data.refine_source && data.refine_source.count > 0) {
                hasData = true;
                statusText += `精:${data.refine_source.count}+${data.refine_target.count}`;
            }
            
            if (hasData) {
                pointcloudStatus.textContent = `点云状态：已就绪 (${statusText.trim()}) - 点击"显示3D点云"查看`;
                pointcloudStatus.style.background = '#d4edda';
                pointcloudStatus.style.color = '#155724';
            } else {
                pointcloudStatus.textContent = '点云状态：无数据';
                pointcloudStatus.style.background = '#fff3cd';
                pointcloudStatus.style.color = '#856404';
            }
            return;
        }
        
        cloudInfo.innerHTML = '<div>正在从API获取点云数据...</div>';
        
        const response = await fetch('/api/robot/matching_clouds');
        const data = await response.json();
        
        if (data.Result !== 0) {
            throw new Error(data.Error || '获取点云数据失败');
        }
        
        // 清除旧的点云和连线
        if (teaserSourceCloud) scene.remove(teaserSourceCloud);
        if (teaserTargetCloud) scene.remove(teaserTargetCloud);
        if (teaserCorrespondenceLines) scene.remove(teaserCorrespondenceLines);
        if (roughSourceCloud) scene.remove(roughSourceCloud);
        if (roughTargetCloud) scene.remove(roughTargetCloud);
        if (refineSourceCloud) scene.remove(refineSourceCloud);
        if (refineTargetCloud) scene.remove(refineTargetCloud);
        
        let hasData = false;
        let infoText = '<div style="line-height: 1.6;"><strong>匹配过程可视化：</strong><br/>';
        
        // 创建TEASER++ source & target点云
        if (data.teaser_source && data.teaser_source.points && data.teaser_target && data.teaser_target.points) {
            teaserSourceCloud = createPointCloud(data.teaser_source.points, 0xff6b6b, 'TEASER++_Source');  // 红色
            teaserTargetCloud = createPointCloud(data.teaser_target.points, 0x4ecdc4, 'TEASER++_Target');  // 青色
            
            if (teaserSourceCloud && teaserTargetCloud) {
                scene.add(teaserSourceCloud);
                scene.add(teaserTargetCloud);
                teaserSourceCloud.visible = showTeaserCheckbox.checked;
                teaserTargetCloud.visible = showTeaserCheckbox.checked;
                statTeaser.textContent = `${data.teaser_source.count}+${data.teaser_target.count}`;
                infoText += `<div>🔴 TEASER++ Source: ${data.teaser_source.count} 点</div>`;
                infoText += `<div>🔵 TEASER++ Target: ${data.teaser_target.count} 点</div>`;
                
                // 绘制对应关系连线
                if (data.teaser_correspondences && data.teaser_correspondences.length > 0) {
                    teaserCorrespondenceLines = createCorrespondenceLines(data.teaser_correspondences);
                    if (teaserCorrespondenceLines) {
                        scene.add(teaserCorrespondenceLines);
                        teaserCorrespondenceLines.visible = showTeaserCheckbox.checked;
                        infoText += `<div>🔗 对应关系: ${data.teaser_correspondences.length} 对</div>`;
                    }
                }
                
                hasData = true;
            }
        } else {
            statTeaser.textContent = '0';
            infoText += '<div style="color: #95a5a6;">TEASER++: 无数据</div>';
        }
        
        // 创建GICP粗匹配 source & target点云
        if (data.rough_source && data.rough_source.points && data.rough_target && data.rough_target.points) {
            roughSourceCloud = createPointCloud(data.rough_source.points, 0xff9f1c, 'GICP粗_Source');  // 橙色
            roughTargetCloud = createPointCloud(data.rough_target.points, 0x4a90e2, 'GICP粗_Target');  // 蓝色
            
            if (roughSourceCloud && roughTargetCloud) {
                scene.add(roughSourceCloud);
                scene.add(roughTargetCloud);
                roughSourceCloud.visible = showRoughCheckbox.checked;
                roughTargetCloud.visible = showRoughCheckbox.checked;
                statRough.textContent = `${data.rough_source.count}+${data.rough_target.count}`;
                infoText += `<div>🟠 GICP粗 当前点云: ${data.rough_source.count} 点</div>`;
                infoText += `<div>🔵 GICP粗 地图点云: ${data.rough_target.count} 点</div>`;
                hasData = true;
            }
        } else {
            statRough.textContent = '0';
            infoText += '<div style="color: #95a5a6;">GICP粗: 无数据</div>';
        }
        
        // 创建GICP精匹配 source & target点云
        if (data.refine_source && data.refine_source.points && data.refine_target && data.refine_target.points) {
            refineSourceCloud = createPointCloud(data.refine_source.points, 0x2ed573, 'GICP精_Source');  // 绿色
            refineTargetCloud = createPointCloud(data.refine_target.points, 0x5f27cd, 'GICP精_Target');  // 紫色
            
            if (refineSourceCloud && refineTargetCloud) {
                scene.add(refineSourceCloud);
                scene.add(refineTargetCloud);
                refineSourceCloud.visible = showRefineCheckbox.checked;
                refineTargetCloud.visible = showRefineCheckbox.checked;
                statRefine.textContent = `${data.refine_source.count}+${data.refine_target.count}`;
                infoText += `<div>🟢 GICP精 当前点云: ${data.refine_source.count} 点</div>`;
                infoText += `<div>🟣 GICP精 地图点云: ${data.refine_target.count} 点</div>`;
                hasData = true;
            }
        } else {
            statRefine.textContent = '0';
            infoText += '<div style="color: #95a5a6;">GICP精: 无数据</div>';
        }
        
        infoText += '</div>';
        cloudInfo.innerHTML = infoText;
        
        if (hasData) {
            pointcloudStatus.textContent = '点云状态：已加载';
            pointcloudStatus.style.background = '#d4edda';
            pointcloudStatus.style.color = '#155724';
        } else {
            pointcloudStatus.textContent = '点云状态：无数据';
            pointcloudStatus.style.background = '#fff3cd';
            pointcloudStatus.style.color = '#856404';
            cloudInfo.innerHTML = '<div>📭 当前没有可用的点云数据</div><div style="font-size: 0.9em; color: #95a5a6; margin-top: 0.5em;">请先触发重定位操作</div>';
        }
        
        console.log('点云数据加载完成');
        
    } catch (error) {
        console.error('获取点云数据失败:', error);
        pointcloudStatus.textContent = `点云状态：加载失败 - ${error.message}`;
        pointcloudStatus.style.background = '#f8d7da';
        pointcloudStatus.style.color = '#721c24';
        cloudInfo.innerHTML = `<div>❌ 加载失败</div><div style="font-size: 0.9em; margin-top: 0.5em;">${error.message}</div>`;
    }
}


// 重置相机位置
function resetCamera() {
    camera.position.set(5, 5, 5);
    camera.lookAt(0, 0, 0);
    controls.reset();
}

// 触发重定位（使用TEASER++全局配准，不依赖初始位姿）
async function triggerRelocalization() {
    try {
        triggerRelocBtn.disabled = true;
        triggerRelocBtn.textContent = '⏳ 触发全局配准...';
        pointcloudStatus.textContent = '点云状态：启动TEASER++全局配准...';
        pointcloudStatus.style.background = '#fff3cd';
        pointcloudStatus.style.color = '#856404';
        
        console.log(`🔥 触发TEASER++全局配准（不使用初始位姿）`);
        
        // 不发送initialpose，让localizer自动进入TEASER++全局配准模式
        // 通过发送一个"无效"的初始位姿（需要修改localizer识别）
        // 或者等待localizer自动启动全局配准
        
        // 方案：触发一次ICP匹配（通过发送当前odom位姿）
        // 由于没有valid prior，localizer会自动使用TEASER++
        triggerRelocBtn.textContent = '⏳ 等待TEASER++配准...';
        pointcloudStatus.textContent = '点云状态：TEASER++全局配准进行中...';
        
        // 注意：实际上我们不发送initialpose，让系统自动进入全局配准
        // 但为了触发ICP流程，我们需要确保localizer收到数据
        // 这里暂时使用一个标记值，后续需要修改localizer代码识别
        const poseResp = await fetch('/api/robot/set_initial_pose', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                x: 0.0,  // 使用原点，表示"无先验"
                y: 0.0,
                z: 0.0,
                yaw: 0.0,
                use_global_registration: true  // 新增标志：强制使用全局配准
            })
        });
        
        const poseData = await poseResp.json();
        
        if (poseData.success === true || poseData.status === 'ok' || poseData.Result === 0) {
            pointcloudStatus.textContent = '点云状态：全局配准已触发，等待TEASER++结果...';
            console.log('✅ Initial pose发送成功:', poseData.message || poseData);
            
            // 等待3秒让TEASER++和GICP完成匹配，然后自动刷新点云
            setTimeout(async () => {
                await fetchPointClouds();
                triggerRelocBtn.disabled = false;
                triggerRelocBtn.textContent = '🎯 触发重定位';
            }, 3000);
        } else {
            throw new Error(poseData.error || poseData.Error || poseData.message || '触发全局配准失败');
        }
        
    } catch (error) {
        console.error('触发重定位失败:', error);
        pointcloudStatus.textContent = `点云状态：触发失败 - ${error.message}`;
        pointcloudStatus.style.background = '#f8d7da';
        pointcloudStatus.style.color = '#721c24';
        triggerRelocBtn.disabled = false;
        triggerRelocBtn.textContent = '🎯 触发重定位';
        
        // 提供更友好的错误提示
        let errorMsg = error.message;
        if (errorMsg.includes('set_initial_pose')) {
            errorMsg = 'API端点不可用，请确认Web服务器和ROS节点正常运行';
        }
        alert(`触发全局配准失败：${errorMsg}\n\n💡 提示：也可以直接点击"刷新点云"查看已有的匹配结果`);
    }
}

// 显示模态窗口
function showModal() {
    modal.style.display = 'flex';
    modal.style.position = 'fixed';
    modal.style.top = '0';
    modal.style.left = '0';
    modal.style.width = '100%';
    modal.style.height = '100%';
    modal.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
    modal.style.justifyContent = 'center';
    modal.style.alignItems = 'center';
    modal.style.zIndex = '1000';
    
    // 初始化场景
    if (!scene) {
        setTimeout(() => {
            initThreeJS();
            fetchPointClouds();
        }, 100);
    } else {
        // 调整渲染器大小
        const width = container.clientWidth;
        const height = container.clientHeight;
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
    }
}

// 隐藏模态窗口
function hideModal() {
    modal.style.display = 'none';
}

// 切换自动刷新
function toggleAutoRefresh() {
    autoRefreshEnabled = !autoRefreshEnabled;
    
    const autoRefreshStatus = document.getElementById('auto-refresh-status');
    
    if (autoRefreshEnabled) {
        toggleAutoRefreshBtn.style.backgroundColor = '#27ae60';
        toggleAutoRefreshBtn.textContent = '✓ 自动刷新点云';
        if (autoRefreshStatus) {
            autoRefreshStatus.textContent = '✓ 已开启 (每3秒)';
            autoRefreshStatus.style.color = '#27ae60';
        }
        // 立即刷新一次
        fetchPointClouds();
        // 每3秒刷新一次点云数据
        autoRefreshInterval = setInterval(() => {
            if (modal.style.display !== 'none') {
                // 只有在模态窗口打开时才刷新
                fetchPointClouds();
            } else {
                // 模态窗口关闭时也更新状态
                fetchPointClouds();
            }
        }, 3000);
    } else {
        toggleAutoRefreshBtn.style.backgroundColor = '#95a5a6';
        toggleAutoRefreshBtn.textContent = '✗ 自动刷新点云';
        if (autoRefreshStatus) {
            autoRefreshStatus.textContent = '✗ 已关闭';
            autoRefreshStatus.style.color = '#95a5a6';
        }
        if (autoRefreshInterval) {
            clearInterval(autoRefreshInterval);
            autoRefreshInterval = null;
        }
    }
}

// 事件监听器
showBtn.addEventListener('click', showModal);
closeBtn.addEventListener('click', hideModal);
refreshBtn.addEventListener('click', fetchPointClouds);
resetCameraBtn.addEventListener('click', resetCamera);
triggerGlobalRelocBtn.addEventListener('click', triggerGlobalRelocalization);
toggleAutoRefreshBtn.addEventListener('click', toggleAutoRefresh);

// 点云显示控制 - 控制source和target
showTeaserCheckbox.addEventListener('change', (e) => {
    if (teaserSourceCloud) teaserSourceCloud.visible = e.target.checked;
    if (teaserTargetCloud) teaserTargetCloud.visible = e.target.checked;
    if (teaserCorrespondenceLines) teaserCorrespondenceLines.visible = e.target.checked;
});

showRoughCheckbox.addEventListener('change', (e) => {
    if (roughSourceCloud) roughSourceCloud.visible = e.target.checked;
    if (roughTargetCloud) roughTargetCloud.visible = e.target.checked;
});

showRefineCheckbox.addEventListener('change', (e) => {
    if (refineSourceCloud) refineSourceCloud.visible = e.target.checked;
    if (refineTargetCloud) refineTargetCloud.visible = e.target.checked;
});

// 点击模态窗口背景关闭
modal.addEventListener('click', (e) => {
    if (e.target === modal) {
        hideModal();
    }
});

// 窗口大小改变时调整渲染器
window.addEventListener('resize', () => {
    if (scene && modal.style.display !== 'none') {
        const width = container.clientWidth;
        const height = container.clientHeight;
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
    }
});

// 导出函数供外部使用
window.PointCloudViewer = {
    show: showModal,
    hide: hideModal,
    refresh: fetchPointClouds,
    resetCamera: resetCamera
};

console.log('点云查看器模块已加载');

