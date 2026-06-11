document.addEventListener("DOMContentLoaded", () => {
    init()
});
let canvas, ctx
function init(){

    let jobActive = false
    let lastLinear = 0
    let lastAngular = 0
    let isMoving = false
    let orderedWaypoints = []
    let addingStation, addingFavorite = false

    let videoConnected = false
    let localStations = []      // created on map
    let selectedStations = []   // multi-selection
    // ---------------- CANVAS ----------------
    canvas = document.getElementById("mapCanvas");
    ctx = canvas.getContext("2d");
    let mapCanvas = null
    let mapWidth = 0
    let mapHeight = 0
    // ---------------- JOYSTICK ----------------

    const base = document.getElementById("joystick-base")
    const knob = document.getElementById("joystick-knob")

    const baseRadius = 100
    const knobRadius = 40

    let dragging = false

    base.addEventListener("mousedown", () => dragging = true);
    document.addEventListener("mouseup", stop);
    document.addEventListener("mousemove", move);

    base.addEventListener("touchstart", start)
    document.addEventListener("touchmove", move)
    document.addEventListener("touchend", stop)

    // ---------------- ESTOP ----------------

    const estopBtn = document.getElementById("estop-button")

    if(estopBtn){
        estopBtn.onclick = () => {
            ws.send(JSON.stringify({
                type: "estop",
                state: 1
            }))
        }
    }

    const stationFilter =
        document.getElementById("stationFilter")

    stationFilter.oninput = () => {

        renderStations()
    }
    // ---------------- DRAWER ----------------

    const controlDrawer =
        document.getElementById("controlDrawer")

    const controlToggle =
        document.getElementById("controlToggle")

    let drawerOpen = false

    controlToggle.onclick = () => {

        drawerOpen = !drawerOpen

        controlDrawer.classList.toggle(
            "open",
            drawerOpen
        )

        controlToggle.innerHTML =
            drawerOpen
                ? "❮"
                : "❯"
    }
    // ---------------- ROBOT ----------------
    let selectedStation = null;
    let rosStations = []
    //let rosStations = JSON.parse(localStorage.getItem("rosStations") || "[]");
    const robotImg = new Image()
    robotImg.src = "data:image/svg+xml;utf8," + encodeURIComponent(`
    <svg width="64" height="64" viewBox="0 0 64 64"
         xmlns="http://www.w3.org/2000/svg">
      <rect x="18" y="16" width="28" height="32" rx="6"
            fill="#4CAF50" stroke="#000" stroke-width="2"/>
      <polygon points="32,10 40,20 24,20" fill="#ffffff"/>
    </svg>
    `)

    let robotReady = false
    robotImg.onload = () => {
        robotReady = true
        drawScene()
    }

    resizeCanvas()
    window.addEventListener("resize", () => {
        resizeCanvas()
        drawScene()
    })

    // ---------------- STATE ----------------

    let lastMap = null
    let lastOdom = null

    // RViz-like zoom (pixels per meter)
    let pixelsPerMeter = 20
    let cameraX = 0
    let cameraY = 0
    // ---------------- WEBSOCKET ----------------

    const ws = new WebSocket(
        (location.protocol === "https:" ? "wss://" : "ws://") +
        location.host +
        "/ws/mqtt_ugv"
    )

    ws.onopen = () => {

        const savedIP =
        localStorage.getItem("brokerIP")

        if (savedIP){

            connectBroker(savedIP)
        }
    }

    ws.onmessage = (event) => {

        const msg = JSON.parse(event.data)
        console.log("WS MSG:", msg.topic)
        if (msg.topic === "/api/map"){
            lastMap = msg.payload.map || msg.payload;
            buildMapCanvas();
            if (cameraX === 0 && cameraY === 0) {

                const scale =
                    pixelsPerMeter *
                    lastMap.info.resolution

                const width =
                    lastMap.info.width

                const height =
                    lastMap.info.height

                const origin =
                    lastMap.info.origin.position

                // map center in ROS world coords
                const centerWorldX =
                    origin.x +
                    (width * lastMap.info.resolution) / 2

                const centerWorldY =
                    origin.y +
                    (height * lastMap.info.resolution) / 2

                // convert world -> map pixels
                const centerPixelX =
                    (centerWorldX - origin.x) /
                    lastMap.info.resolution

                const centerPixelY =
                    (centerWorldY - origin.y) /
                    lastMap.info.resolution

                // place center on screen center
                cameraX =
                    canvas.width / 2 -
                    centerPixelX * scale

                cameraY =
                    canvas.height / 2 -
                    (height - centerPixelY) * scale
            }
            requestAnimationFrame(() => {

                drawScene()

                // second frame helps large maps
                requestAnimationFrame(() => {
                    drawScene()
                })
            })
        }

        if (msg.topic === "/api/odom"){
            lastOdom = msg.payload;
            drawScene();
        }

        if (msg.topic === "/api/station/list") {
            const stations = msg.payload.stations || []

            // store in memory
            rosStations = stations

            // persist in browser
            localStorage.setItem("rosStations", JSON.stringify(stations))
            renderStations(msg.payload.stations)
        }

        if (msg.topic === "/api/job/feedback") {

            const status = msg.payload.job_status?.label

            console.log("Job feedback:", status)

            if (status === "IN_PROGRESS") {
                jobActive = true
                document.getElementById("startMission").disabled = true
                showToast("Job started", "info")
            }

            const task = msg.payload.tasks_status?.[0]

            if (task?.status?.label === "RUNNING") {
                showToast("Executing task", "warn")
            }

        }

        if (msg.topic === "/api/job/result") {

            const status = msg.payload.job_status?.label

            console.log("Job result:", status)

            if (status === "COMPLETED" || status === "CANCELLED") {

                resetUI()   // clears map + selections

                const statusDiv = document.getElementById("jobStatus")
                if (statusDiv) statusDiv.innerText = `Status: ${status}`
            }
        }
    }

    // ---------------- ZOOM (VERY IMPORTANT) ----------------

    /*canvas.addEventListener("wheel", (e) => {

        e.preventDefault()

        const mouseX = e.offsetX
        const mouseY = e.offsetY

        const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9

        // world coords before zoom
        const worldX = (mouseX - cameraX) / pixelsPerMeter
        const worldY = (mouseY - cameraY) / pixelsPerMeter

        pixelsPerMeter *= zoomFactor

        // keep cursor fixed during zoom
        cameraX = mouseX - worldX * pixelsPerMeter
        cameraY = mouseY - worldY * pixelsPerMeter

        drawScene()
    });*/

    // ================================
    // PAN + NORMAL ZOOM
    // ================================

    let isPanning = false
    let hasDragged = false

    let panStartX = 0
    let panStartY = 0

    let startCameraX = 0
    let startCameraY = 0

    const DRAG_THRESHOLD = 5

    // ---------------- PAN START ----------------

    canvas.addEventListener("mousedown", (e) => {

        if (addingStation) return

        isPanning = true
        hasDragged = false

        panStartX = e.clientX
        panStartY = e.clientY

        startCameraX = cameraX
        startCameraY = cameraY

        canvas.style.cursor = "grabbing"
    })

    // ---------------- PAN MOVE ----------------

    window.addEventListener("mousemove", (e) => {

        if (!isPanning) return

        const dx = e.clientX - panStartX
        const dy = e.clientY - panStartY

        // detect actual drag
        if (
            Math.abs(dx) > DRAG_THRESHOLD ||
            Math.abs(dy) > DRAG_THRESHOLD
        ) {
            hasDragged = true
        }

        cameraX = startCameraX + dx
        cameraY = startCameraY + dy

        drawScene()
    })

    // ---------------- PAN END ----------------

    window.addEventListener("mouseup", () => {

        isPanning = false

        canvas.style.cursor = "grab"

        // small timeout prevents click firing immediately
        setTimeout(() => {
            hasDragged = false
        }, 50)
    })

    // ---------------- ZOOM ----------------

    canvas.addEventListener("wheel", (e) => {

        e.preventDefault()

        const zoom =
            e.deltaY < 0
                ? 1.12
                : 0.88

        const mouseX = e.offsetX
        const mouseY = e.offsetY

        // position before zoom
        const wx =
            (mouseX - cameraX) /
            pixelsPerMeter

        const wy =
            (mouseY - cameraY) /
            pixelsPerMeter

        pixelsPerMeter *= zoom

        // LIMITS
        pixelsPerMeter =
            Math.max(
                5,
                Math.min(150, pixelsPerMeter)
            )

        // maintain cursor focus
        cameraX =
            mouseX - wx * pixelsPerMeter

        cameraY =
            mouseY - wy * pixelsPerMeter

        drawScene()
    },
    {
        passive:false
    })

    /*// ---------------- TEST (NO ROS) ----------------

    // fake map
    const fakeMap = {
      info: {
        resolution: 0.05,
        width: 20,
        height: 20,
        origin: { position: { x: -0.5, y: -0.5, z: 0 } }
      },
      data: new Array(400).fill(0)
    }

    lastMap = fakeMap

    // fake odom movement
    setInterval(() => {
        lastOdom = {
            x: Math.cos(Date.now()/1000),
            y: Math.sin(Date.now()/1000),
            theta: Date.now()/1000
        }
        drawScene()
    }, 50)*/

    //drawScene();

    const startBtn = document.getElementById("startMission")

    startBtn.onclick = () => {

        if (jobActive) {
            console.warn("Job already running")
            return
        }

        const selectedGoalStations = selectedStations
            .map(name => rosStations.find(s => s.name === name))
            .filter(Boolean)

        if (selectedGoalStations.length === 0) {
            console.warn("No stations selected")
            return
        }

        const mapWaypoints = orderedWaypoints
            .filter(w => w.type === "map")
            .map(w => ({
                x: w.x,
                y: w.y,
                theta: w.theta
            }))

        const payload = {

            type: "job_goal",

            tasks: [
                {
                    type: 1,

                    task_station: {

                        stations: selectedGoalStations.map(st => ({
                            station_name: st.name,

                            station_pose: {
                                x: st.x,
                                y: st.y,
                                theta: st.theta
                            }
                        })),

                        waypoints: mapWaypoints
                    }
                }
            ]
        }
        // lock UI
        jobActive = true
        startBtn.disabled = true

        ws.send(JSON.stringify(payload))

        console.log("Mission sent:", payload)
    }
    /*document.getElementById("clearMission").onclick = () => {
        selectedStation = null
        selectedWaypoints = []
        clearSelectionUI()
    }*/

    canvas.addEventListener("click", async (e) => {

        // ignore click after dragging
        if (hasDragged) return;

        if (!lastMap) return;

        const rect = canvas.getBoundingClientRect()

        const screenX = e.clientX - rect.left
        const screenY = e.clientY - rect.top

        const pose = screenToMap(canvas, screenX, screenY);

        if (addingStation) {

            addingStation = false
            addStationBtn.style.background = ""

            openStationDialog(async (name) => {

                try {

                    await fetch(
                        `/stations/add?name=${name}&x=${pose.x}&y=${pose.y}&theta=${pose.theta}`
                    )

                    await loadStations()

                    drawScene()

                    console.log("Station created:", name)

                } catch (err) {

                    console.error("Station create failed:", err)
                }
            })

            return
        }

        orderedWaypoints.push({
            type: "map",
            x: pose.x,
            y: pose.y,
            theta: pose.theta
        })

        console.log("Waypoint added:", pose)

        drawScene()
    })

    document.getElementById("clearMission").onclick = () => {

        // get all selected (goal + waypoint)
        const selectedNames = [...selectedStations]

        if (selectedNames.length === 0) {
            console.warn("No stations selected")
            return
        }

        const selectedGoals = selectedNames
            .map(name => rosStations.find(s => s.name === name))
            .filter(Boolean)

        // remove ONLY from localStations (not ROS)
        localStations = localStations.filter(
            s => !selectedNames.includes(s.name)
        )

        // update storage
        localStorage.setItem("localStations", JSON.stringify(localStations))

        // clear selection
        selectedStations = []

        // refresh UI
        renderStations([...rosStations, ...localStations])
        drawScene()

        console.log("cleared:", selectedNames)
    }

    const cancelBtn = document.getElementById("cancelMission")

    cancelBtn.onclick = () => {

        if (!ws || ws.readyState !== WebSocket.OPEN) return

        ws.send(JSON.stringify({
            type: "job_cancel"
        }))

        console.log("Cancel sent")

        resetUI()

        ws.send(JSON.stringify({
            type: "map_request",
            request: "get_map"
        }))
    }

    //--------------MAP CACHING-------------------
    function buildMapCanvas(){

        if (!lastMap) return

        const width = lastMap.info.width
        const height = lastMap.info.height

        mapWidth = width
        mapHeight = height

        const canvasTmp = document.createElement("canvas")
        canvasTmp.width = width
        canvasTmp.height = height

        const ctxTmp = canvasTmp.getContext("2d")
        const image = ctxTmp.createImageData(width, height)

        for (let i = 0; i < lastMap.data.length; i++) {

            let value = Number(lastMap.data[i])

            // ROS int8 fix
            if (value === 255) value = -1

            let color = 255

            if (value >= 65) {
                color = 0          // occupied
            }
            else if (value === -1) {
                color = 180        // unknown
            }
            else {
                color = 255        // free
            }

            image.data[i * 4]     = color
            image.data[i * 4 + 1] = color
            image.data[i * 4 + 2] = color
            image.data[i * 4 + 3] = 255
        }
        console.log({
            width: lastMap.info.width,
            height: lastMap.info.height,
            dataLength: lastMap.data.length,
            first: lastMap.data[0],
            sample: lastMap.data.slice(0, 20)
        })
        ctxTmp.imageSmoothingEnabled = false
        ctx.webkitImageSmoothingEnabled = false
        ctx.mozImageSmoothingEnabled = false
        ctxTmp.putImageData(image, 0, 0)

        mapCanvas = canvasTmp

        console.log("Map cached ✔")
    }

    function screenToMap(canvas, screenX, screenY){

        if (!lastMap) return null

        const res = lastMap.info.resolution

        const scale = pixelsPerMeter * res

        const origin = lastMap.info.origin.position

        // remove camera offset
        const localX = screenX - cameraX
        const localY = screenY - cameraY

        // undo scale
        const mapPixelX = localX / scale
        const mapPixelY =
            lastMap.info.height - (localY / scale)

        // convert map pixel -> world
        const x =
            mapPixelX * res + origin.x

        const y =
            mapPixelY * res + origin.y

        return { x, y, theta: 0 }
    }

    function clearSelectionUI(){
        document.querySelectorAll(".stationItem").forEach(el => {
            el.classList.remove("active", "selected")
        })
    }

    function renderStations(stations){
        stations =
            stations ||
            [...rosStations, ...localStations]
        if (!Array.isArray(stations)) {
            console.error("renderStations got invalid data:", stations)
            return
        }
        const container = document.getElementById("stationsContainer")
        container.innerHTML = ""
        const filterText =
            stationFilter.value
                .trim()
                .toLowerCase()
        stations.filter(st =>
                st.name
                    .toLowerCase()
                    .includes(filterText)
            ).forEach(station => {

            const div = document.createElement("div")
            div.className = "stationItem"

            const name = document.createElement("span")
            name.innerText = station.name

            const delBtn = document.createElement("button")
            delBtn.innerText = "🗑"
            delBtn.className = "deleteBtn"

            // DELETE SINGLE STATION
            delBtn.onclick = async (e) => {
                e.stopPropagation()

                try {
                    await fetch(`/stations/delete?name=${station.name}`)

                    await loadStations()   // refresh from backend
                    drawScene()

                    console.log("Deleted:", station.name)

                } catch (err) {
                    console.error("Delete failed:", err)
                }
            }

            // existing click logic
            div.onclick = () => {

                const index =
                    selectedStations.indexOf(station.name)

                if (index >= 0) {

                    selectedStations.splice(index, 1)


                } else {

                    selectedStations.push(station.name)

                }

                renderStations(stations)

                drawScene()
            }

            if (selectedStations.includes(station.name)) {
                div.classList.add("active")
            }
            div.appendChild(name)
            div.appendChild(delBtn)

            container.appendChild(div)
        })
    }

    // ---------------- GRID ----------------
    function drawGrid(){

        const spacing = pixelsPerMeter

        ctx.strokeStyle = "#1f2937"
        ctx.lineWidth = 1

        for(let x=0; x<canvas.width; x+=spacing){
            ctx.beginPath()
            ctx.moveTo(x,0)
            ctx.lineTo(x,canvas.height)
            ctx.stroke()
        }

        for(let y=0; y<canvas.height; y+=spacing){
            ctx.beginPath()
            ctx.moveTo(0,y)
            ctx.lineTo(canvas.width,y)
            ctx.stroke()
        }
    }

    // ---------------- DRAW ----------------

    function drawScene(){
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, canvas.width, canvas.height)

        if (!lastMap) return

        const width = lastMap.info.width
        const height = lastMap.info.height
        const res = lastMap.info.resolution
        const origin = lastMap.info.origin.position

        const scale = pixelsPerMeter * res

        // map size in pixels
        const mapWidthPx = width * scale
        const mapHeightPx = height * scale

        // ---- DRAW MAP ----
        if (!mapCanvas) return

        ctx.save()

        ctx.translate(cameraX, cameraY)
        ctx.scale(scale, scale)
        // ROS map flip
        ctx.translate(0, height)
        ctx.scale(1, -1)

        ctx.imageSmoothingEnabled = false
        ctx.drawImage(mapCanvas, 0, 0)

        // ---- DRAW ROBOT ----
        if (robotReady && lastOdom){

            const x = (lastOdom.x - origin.x) / res
            const y = (lastOdom.y - origin.y) / res

            ctx.save()
            ctx.translate(x, y)
            ctx.scale(1, -1)
            ctx.rotate(lastOdom.theta || 0)

            ctx.drawImage(robotImg, -15, -15, 30, 30)

            ctx.restore()
        }

        // ---- DRAW STATIONS ----
        [...rosStations, ...localStations].forEach(st => {

            const x = (st.x - origin.x) / res
            const y = (st.y - origin.y) / res

            ctx.save()

            ctx.translate(x, y)

            // unflip local object
            ctx.scale(1, -1)

            const selected =
                selectedStations.includes(st.name)

            // station dot
            ctx.fillStyle = "#3b82f6"

            ctx.beginPath()
            ctx.arc(0, 0, 6, 0, Math.PI * 2)
            ctx.fill()

            // selected flag
            if (selected) {

                // pole
                ctx.strokeStyle = "#10b981"
                ctx.lineWidth = 2

                ctx.beginPath()
                ctx.moveTo(0, -6)
                ctx.lineTo(0, -24)
                ctx.stroke()

                // flag
                ctx.fillStyle = "#10b981"

                ctx.beginPath()
                ctx.moveTo(0, -24)
                ctx.lineTo(14, -19)
                ctx.lineTo(0, -14)
                ctx.closePath()

                ctx.fill()

                // order number
                const order =
                    selectedStations.indexOf(st.name) + 1

                ctx.fillStyle = "white"
                ctx.font = "10px sans-serif"
                ctx.fillText(order, 4, -17)
            }

            // label
            ctx.fillStyle = "gray"
            ctx.font = "12px sans-serif"
            ctx.fillText(st.name, 8, -3)

            ctx.restore()
        })

        // ---- DRAW WAYPOINTS ----
        orderedWaypoints.forEach((wp, i) => {

            let x, y

            if (wp.type === "map") {
                x = (wp.x - origin.x) / res
                y = (wp.y - origin.y) / res
            }


            ctx.save()
            ctx.translate(x, y)
            ctx.scale(1, -1)
            ctx.fillStyle = "#f59e0b"
            ctx.beginPath()
            ctx.arc(0, 0, 5, 0, Math.PI * 2)
            ctx.fill()

            // index number (correct order)
            ctx.fillStyle = "#000"
            ctx.fillText(i + 1, 8, 3)

            ctx.restore()
        });

        ctx.restore()
    }

    function resizeCanvas(){
        const rect = canvas.getBoundingClientRect()
        canvas.width = rect.width
        canvas.height = rect.height
    }

    function move(e){

        if(!dragging) return

        const rect = base.getBoundingClientRect()
        const centerX = rect.left + baseRadius
        const centerY = rect.top + baseRadius

        let dx = e.clientX - centerX
        let dy = e.clientY - centerY

        const dist = Math.sqrt(dx*dx + dy*dy)

        if(dist > baseRadius - knobRadius){
            dx = dx/dist * (baseRadius - knobRadius)
            dy = dy/dist * (baseRadius - knobRadius)
        }

        knob.style.left = (50 + (dx/baseRadius)*50) + "%"
        knob.style.top = (50 + (dy/baseRadius)*50) + "%"
        knob.style.transform = "translate(-50%, -50%)"

        //sendCommand(-dy/baseRadius, dx/baseRadius)
        let linear  = -dy / baseRadius
        let angular =  dx / baseRadius

        // Flip lower half (when joystick is below center)
        /*if (dy > 0) {
            linear  = -linear
            angular = -angular
        }*/

        // optional deadzone
        const deadzone = 0.05
        if (Math.abs(linear) < deadzone) linear = 0
        if (Math.abs(angular) < deadzone) angular = 0

        lastLinear = linear
        lastAngular = angular
        isMoving = true
    }
    function start(){
        dragging = true
    }

    function stop(){

        dragging = false
        isMoving = false
        lastLinear = 0
        lastAngular = 0
        knob.style.left = "50%"
        knob.style.top = "50%"
        knob.style.transform = "translate(-50%, -50%)"

        sendCommand(0, 0)
    }

    // ---------------- POSITION ----------------

    function getPosition(e){
        if(e.touches){
            return {
                x: e.touches[0].clientX,
                y: e.touches[0].clientY
            }
        }
        return {
            x: e.clientX,
            y: e.clientY
        }
    }

    async function loadStations(){

        try {
            const res = await fetch("/stations")

            if (!res.ok) {
                console.error("Failed to load stations")
                return
            }

            const data = await res.json()

            rosStations = data.stations || []

            renderStations(rosStations)
            drawScene()


            console.log("Stations loaded:", rosStations)

        } catch (err) {
            console.error("Error loading stations:", err)
        }
    }

    // ---------------- SEND COMMAND (THROTTLED) ----------------

    let lastSent = 0

    function sendCommand(linear, angular){
        if (!driveEnabled) return;

        if(ws.readyState !== WebSocket.OPEN) return

        const now = Date.now()
        if(now - lastSent < 50) return   // 20 Hz

        lastSent = now

        const msg = {
            type: "cmd_vel",
            linear: {
                x: Number(linear.toFixed(3)),
                y: 0.0
            },
            angular: {
                z: Number(angular.toFixed(3))
            }
        }

        ws.send(JSON.stringify(msg))
    }

    function resetUI(){

        // clear selections
        selectedStations = [];

        // clear clicked waypoints
        orderedWaypoints = []


        // unlock start button
        jobActive = false
        document.getElementById("startMission").disabled = false

        // optional: reset status text
        const status = document.getElementById("jobStatus")
        if (status) status.innerText = "Status: Idle"

        drawScene()
    }
    loadStations();

    function showToast(message, type = "info"){

        const container = document.getElementById("notifContainer")

        const toast = document.createElement("div")
        toast.className = `toast ${type}`
        toast.innerText = message

        container.appendChild(toast)

        // trigger animation
        setTimeout(() => toast.classList.add("show"), 10)

        // remove after 3 sec
        setTimeout(() => {
            toast.classList.remove("show")
            setTimeout(() => toast.remove(), 300)
        }, 3000)
    }

    const addStationBtn = document.getElementById("addStationBtn")

    addStationBtn.onclick = () => {

        addingStation = !addingStation

        if (addingStation) {
            showToast("Click map to place station", "info")
            addStationBtn.style.background = "#10b981"
        } else {
            addStationBtn.style.background = ""
        }
    }

    function openStationDialog(onSave){

        const overlay = document.getElementById("stationDialog")
        const input = document.getElementById("stationNameInput")

        const saveBtn = document.getElementById("stationSaveBtn")
        const cancelBtn = document.getElementById("stationCancelBtn")

        overlay.classList.remove("hidden")
        overlay.classList.add("show")

        input.value = ""
        input.focus()

        function close(){

            overlay.classList.remove("show")
            overlay.classList.add("hidden")
            // CLEANUP
            saveBtn.onclick = null
            cancelBtn.onclick = null
            input.onkeydown = null
        }

        saveBtn.onclick = () => {

            const name = input.value.trim()

            if (!name) return

            close()

            onSave(name)
        }

        cancelBtn.onclick = () => {

            close()
        }

        input.onkeydown = (e) => {

            if (e.key === "Enter") {
                saveBtn.click()
            }

            if (e.key === "Escape") {
                close()
            }
        }
    }


    /* =========================================================
       LIGHT TOGGLE
    ========================================================= */

    const controls = [

        {
            key: "FL",
            button: document.getElementById("frontBtn"),
            state: document.getElementById("frontState")
        },

        {
            key: "RL",
            button: document.getElementById("rearBtn"),
            state: document.getElementById("rearState")
        },

        {
            key: "FG",
            button: document.getElementById("fogBtn"),
            state: document.getElementById("fogState")
        }

    ]

    controls.forEach(control => {

        control.button.addEventListener("click", () => {

            control.button.classList.toggle("active")

            const active =
                control.button.classList.contains("active")

            const value =
                active ? 1 : 0

            control.state.innerText =
                active ? "ON" : "OFF"

            control.state.style.background =
                active ? "#10b981" : "rgba(255,255,255,0.06)"

            control.state.style.color =
                active ? "white" : "#9ca3af"

            const payload = {
                FL: document
                        .getElementById("frontBtn")
                        .classList.contains("active") ? 1 : 0,

                RL: document
                        .getElementById("rearBtn")
                        .classList.contains("active") ? 1 : 0,

                FG: document
                        .getElementById("fogBtn")
                        .classList.contains("active") ? 1 : 0
            }

            // SEND
            ws.send(JSON.stringify({
                type: "light",
                topic: "/api/light",
                payload
            }))

            console.log(
                "Light payload:",
                payload
            )

        })

    })
    connectVideoBtn.onclick = async () => {

        // DISCONNECT
        if (videoConnected){

            cameraFeed.src = "about:blank"

            connectVideoBtn.innerHTML = "🔗"

            connectVideoBtn.classList.remove("connected")

            videoConnected = false
            ipcInput.disabled = false
            return
        }

        // CONNECT
        const ip =
            ipcInput.value.trim()

        if (!ip) return

        // VIDEO
        cameraFeed.src =`http://${ip}:8889/webcam`

        // MQTT BROKER
        const connected = await connectBroker(ip)

        connectVideoBtn.innerHTML = "⛔"
        ipcInput.disabled = true
        connectVideoBtn.classList.add("connected")

        videoConnected = true
    }

    /* =========================================================
       STATE
    ========================================================= */

    let selectedFavorite = null
    let missionRunning = false
    let runningFavorite = null

    let favorites = []

    /* =========================================================
       TAB ELEMENTS
    ========================================================= */

    const favoritesTabBtn =
        document.getElementById("favoritesTabBtn")

    const stationsTabBtn =
        document.getElementById("stationsTabBtn")

    const favoritesView =
        document.getElementById("favoritesView")

    const stationsView =
        document.getElementById("stationsView")

    const drawerActionId = document.getElementById("drawerActionId");
    /* =========================================================
       TAB SWITCHING
    ========================================================= */

    favoritesTabBtn.addEventListener("click", () => {

        favoritesTabBtn.classList.add("active")
        stationsTabBtn.classList.remove("active")

        favoritesView.classList.remove("hidden")
        stationsView.classList.add("hidden")
        drawerActionId.classList.add("hidden")
    })

    stationsTabBtn.addEventListener("click", () => {

        stationsTabBtn.classList.add("active")
        favoritesTabBtn.classList.remove("active")

        stationsView.classList.remove("hidden")
        favoritesView.classList.add("hidden")
        drawerActionId.classList.remove("hidden")
    })

    /* =========================================================
       LOAD FAVORITES
    ========================================================= */

    async function loadFavorites(){

        const res =
            await fetch("/api/favorites")

        const data =
            await res.json()

        favorites = data.favorites || []

        renderFavorites()
    }

    /* =========================================================
       RENDER FAVORITES
    ========================================================= */

    function renderFavorites(){

        const container =
            document.getElementById("favoritesContainer")

        container.innerHTML = ""

        favorites.forEach(fav => {

            const isSelected =
                selectedFavorite === fav.name

            const isRunning =
                runningFavorite === fav.name

            const card =
                document.createElement("div")

            card.className =
                "favoriteCard"

            if(isSelected)
                card.classList.add("selected")

            if(isRunning)
                card.classList.add("running")

            /* =================================================
               HEADER
            ================================================= */

            const header =
                document.createElement("div")

            header.className =
                "favoriteHeader"

            header.innerHTML = `

                <div>

                    <div class="favoriteName">
                        ${fav.name}
                    </div>

                    <div class="favoriteMeta">
                        ${fav.stations.length} STATIONS
                    </div>

                </div>
            `

            header.onclick = () => {
                if(selectedFavorite === fav.name){

                    selectedFavorite = null

                }else{

                    selectedFavorite = fav.name
                }

                renderFavorites()
            }

            card.appendChild(header)

            /* =================================================
               CONTROLS
            ================================================= */

            if(isSelected){

                const controls =
                    document.createElement("div")

                controls.className =
                    "favoriteControls"

                /* =============================================
                   RUNNING
                ============================================= */

                if(isRunning){

                    controls.innerHTML = `
                        <button class="favAction stop">■</button>`

                    controls.querySelector(".favAction.stop")
                        .onclick = () =>
                            stopFavorite(fav.name)

                }

                /* =============================================
                   IDLE
                ============================================= */

                else{

                    controls.innerHTML = `
                        <button class="favAction start">▶</button>
                        <button class="favAction delete">🗑</button>
                    `

                    controls.querySelector(".favAction.start")
                        .onclick = () =>
                            startFavorite(fav)

                    controls.querySelector(".favAction.delete")
                        .onclick = () =>
                            deleteFavorite(fav.name)
                }

                card.appendChild(controls)
            }

            container.appendChild(card)
        })
    }

    /* =========================================================
       START FAVORITE
    ========================================================= */

    async function startFavorite(fav){

        runningFavorite = fav.name

        renderFavorites()

        selectedStations = [...fav.stations]

        orderedWaypoints = [...fav.waypoints]

        drawScene()

        const selectedGoalStations = selectedStations
            .map(name => rosStations.find(s => s.name === name))
            .filter(Boolean)

        const mapWaypoints = orderedWaypoints
            .map(w => ({
                x: w.x,
                y: w.y,
                theta: w.theta || 0
            }))

        const payload = {

            type: "job_goal",

            tasks: [
                {
                    type: 1,

                    task_station: {

                        stations:
                            selectedGoalStations.map(st => ({

                                station_name: st.name,

                                station_pose: {
                                    x: st.x,
                                    y: st.y,
                                    theta: st.theta
                                }
                            })),

                        waypoints: mapWaypoints
                    }
                }
            ]
        }

        ws.send(JSON.stringify(payload))

        console.log(
            "Favorite mission started:",
            payload
        )
    }
    /* =========================================================
       STOP FAVORITE
    ========================================================= */

    async function stopFavorite(favId){

        ws.send(JSON.stringify({
            type: "job_cancel"
        }))

        runningFavorite = null

        resetUI()

        renderFavorites()

        console.log(
            "Favorite stopped:",
            favId
        )
    }

    /* =========================================================
       DELETE FAVORITE
    ========================================================= */
    let favoritesData = []
    async function deleteFavorite(favName){

        try{

            const res = await fetch(

                `/favorites/delete?name=${encodeURIComponent(favName)}`,

                {
                    method: "DELETE"
                }
            )

            const data = await res.json()

            console.log(data)

            /* -------------------------------------
               REMOVE FROM LOCAL STATE
            ------------------------------------- */

            favorites = favorites.filter(
                    f => f.name !== favName
                )

            /* -------------------------------------
               CLEAR SELECTION
            ------------------------------------- */

            if(selectedFavorite === favName){

                selectedFavorite = null
            }

            /* -------------------------------------
               RE-RENDER
            ------------------------------------- */

            renderFavorites()

            showToast(
                "Favorite deleted",
                "success"
            )

        }catch(err){

            console.error(err)

            showToast(
                "Delete failed",
                "error"
            )
        }
    }

    /* =========================================================
       INIT
    ========================================================= */

    loadFavorites()

    const favoritesFilter = document.getElementById("favoritesFilter")

    if(favoritesFilter){

        favoritesFilter.addEventListener("input", () => {

            const term = favoritesFilter.value.toLowerCase()

            document.querySelectorAll(".favoriteCard")
            .forEach(card => {

                const name =
                    card.innerText.toLowerCase()

                card.style.display =
                    name.includes(term)
                    ? ""
                    : "none"
            })
        })
    }

    /* =========================================================
       SAVE FAVORITE
    ========================================================= */

    const saveFavoriteBtn =
        document.getElementById("saveFavoriteBtn");

    saveFavoriteBtn.onclick = () => {
        showToast("Save Favorites", "info")
        saveFavoriteBtn.style.background = "#10b981"
        if (selectedStations.length === 0){
            showToast(
                "No stations or waypoints selected",
                "warn"
            )

            return
        }

        openFavDialog(async (name) => {

            try {
                const payload = {

                    name: name.trim(),

                    stations: [...selectedStations],

                    waypoints: orderedWaypoints.map(w => ({
                        x: w.x,
                        y: w.y,
                        theta: w.theta || 0
                    }))
                }
                const res = await fetch(

                    "/favorites/add",

                    {
                        method: "POST",

                        headers: {
                            "Content-Type": "application/json"
                        },

                        body: JSON.stringify(payload)
                    }
                )

                const data = await res.json()

                console.log(data)

                showToast(
                    "Favorite saved",
                    "success"
                )
                loadFavorites();
            } catch (err) {

                 console.error(err)

                showToast(
                    "Failed to save favorite",
                    "error"
                )
            }
        })

    }

    function openFavDialog(onSave){
        const favOverlay = document.getElementById("favDialog")
        const favInput = document.getElementById("favNameInput")

        const favSaveBtn = document.getElementById("favSaveBtn")
        const favCancelBtn = document.getElementById("favCancelBtn")

        favOverlay.classList.remove("hidden")
        favOverlay.classList.add("show")

        favInput.value = ""
        favInput.focus()

        function close(){
            favOverlay.classList.add("hidden")
            favOverlay.classList.remove("show")

            // CLEANUP
            favSaveBtn.onclick = null
            favCancelBtn.onclick = null
            favInput.onkeydown = null
            saveFavoriteBtn.style.background = ""
        }

        favSaveBtn.onclick = () => {

            const name = favInput.value.trim()

            if (!name) return

            close()

            onSave(name)
            saveFavoriteBtn.style.background = ""
        }

        favCancelBtn.onclick = () => {

            close()
        }

        favInput.onkeydown = (e) => {

            if (e.key === "Enter") {
                favSaveBtn.click()
            }

            if (e.key === "Escape") {
                close()
            }
        }
    }
    ipcInput.value = localStorage.getItem("brokerIP") || ""

    const savedBrokerIP =
        localStorage.getItem("brokerIP")

    if (savedBrokerIP){

        cameraFeed.src =
        `http://${savedBrokerIP}:8889/webcam`

        videoConnected = true

        connectVideoBtn.innerHTML = "⛔"

        connectVideoBtn.classList.add("connected")

        ipcInput.disabled = true
    }

    async function connectBroker(ip){

        if (!ip) return false

        try{

            const brokerRes = await fetch("/api/set-broker", {

                method: "POST",

                headers: {
                    "Content-Type": "application/json"
                },

                body: JSON.stringify({ ip })
            })

            const brokerData = await brokerRes.json()

            console.log("Broker:", brokerData)

            if (!brokerData.ok){

                showToast(
                    "Broker connection failed",
                    "error"
                )

                return false
            }

            localStorage.setItem("brokerIP", ip)

            ws.send(JSON.stringify({
                type: "map_request",
                request: "get_map"
            }))

            ws.send(JSON.stringify({
                type: "station_list_request"
            }))

            showToast(
                "Broker connected",
                "success"
            )

            return true

        }catch(err){

            console.error(err)

            showToast(
                "Broker connection failed",
                "error"
            )

            return false
        }
    }

    let driveEnabled = false;

    setInterval(() => {

        if (!driveEnabled) {
            sendCommand(0, 0);
            return;
        }

        if (dragging) {
            sendCommand(lastLinear, lastAngular);
        } else {
            sendCommand(0, 0);
        }

    }, 100); // 10Hz
    const driveSwitch =
        document.getElementById("driveEnableSwitch");

    driveSwitch.addEventListener("change", () => {

        driveEnabled = driveSwitch.checked;

        if (!driveEnabled) {

            sendCommand(0, 0);

            showToast("Drive Disabled","warning");

        } else {

            showToast("Drive Enabled","success");
        }

    });
}
