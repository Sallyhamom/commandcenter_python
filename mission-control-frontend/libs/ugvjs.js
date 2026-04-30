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
    setInterval(() => {

        if (dragging) {
            sendCommand(lastLinear, lastAngular)
        } else {
            sendCommand(0, 0)
        }

    }, 100)

    const stationStates = new Map();
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
    // ---------------- DRAWER ----------------

    const toggleBtn = document.getElementById("controlToggle");
    const drawer = document.getElementById("controlDrawer");

    toggleBtn.onclick = () => {
        drawer.classList.toggle("open")
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
    let pixelsPerMeter = 80

    // ---------------- WEBSOCKET ----------------

    const ws = new WebSocket(
        (location.protocol === "https:" ? "wss://" : "ws://") +
        location.host +
        "/ws/mqtt_ugv"
    )

    ws.onopen = () => {

        ws.send(JSON.stringify({
            type: "map_request",
            request: "get_map"
        }))

        ws.send(JSON.stringify({
            type: "station_list_request"
        }))
    }

    ws.onmessage = (event) => {

        const msg = JSON.parse(event.data)

        if (msg.topic === "/api/map"){
            lastMap = msg.payload.map || msg.payload;
            buildMapCanvas();
            drawScene();
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

    canvas.addEventListener("wheel", (e) => {

        e.preventDefault()

        if (e.deltaY < 0) pixelsPerMeter *= 1.1
        else pixelsPerMeter *= 0.9

        drawScene()
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

        const goalEntry = [...stationStates.entries()]
            .find(([_, state]) => state === "goal")

        if (!goalEntry) {
            console.warn("No goal selected")
            return
        }

        const goalName = goalEntry[0]

        const allStations = rosStations   // backend now owns stations

        const goalStation = allStations.find(s => s.name === goalName)

        // station-based waypoints
        const stationWaypoints = [...stationStates.entries()]
            .filter(([_, state]) => state === "waypoint")
            .map(([name]) => rosStations.find(s => s.name === name))
            .filter(Boolean)

        // merge both
        const allWaypoints = orderedWaypoints.map(w => {

            if (w.type === "map") {
                return {
                    x: w.x,
                    y: w.y,
                    theta: w.theta
                }
            }

            if (w.type === "station") {
                const s = rosStations.find(st => st.name === w.name)
                if (!s) return null

                return {
                    x: s.x,
                    y: s.y,
                    theta: s.theta
                }
            }

        }).filter(Boolean)

        // BUILD PAYLOAD HERE
        const payload = {
        type: "job_goal",
        tasks: [
            {
                type: 1,
                task_station: {
                    station_name: goalStation.name,
                    station_pose: {
                        x: goalStation.x,
                        y: goalStation.y,
                        theta: goalStation.theta
                    },
                    waypoints: allWaypoints
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

        if (!lastMap) return;

        const rect = canvas.getBoundingClientRect()

        const screenX = e.clientX - rect.left
        const screenY = e.clientY - rect.top

        const pose = screenToMap(canvas, screenX, screenY)

        //add stations
        if (e.shiftKey) {
            try {
                const name = prompt("Station name:")
                if (!name) return

                await fetch(`/stations/add?name=${name}&x=${pose.x}&y=${pose.y}&theta=${pose.theta}`)

                await loadStations()

                drawScene()

                console.log("Station created:", name)

            } catch (err) {
                console.error("Station create failed:", err)
            }

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
        const selectedNames = [...stationStates.keys()]

        if (selectedNames.length === 0) {
            console.warn("No stations selected")
            return
        }

        // remove ONLY from localStations (not ROS)
        localStations = localStations.filter(
            s => !selectedNames.includes(s.name)
        )

        // update storage
        localStorage.setItem("localStations", JSON.stringify(localStations))

        // clear selection
        stationStates.clear()

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

            let value = lastMap.data[i]
            let color = 255

            if (value === 100) color = 0
            else if (value === -1) color = 200

            image.data[i * 4] = color
            image.data[i * 4 + 1] = color
            image.data[i * 4 + 2] = color
            image.data[i * 4 + 3] = 255
        }

        ctxTmp.putImageData(image, 0, 0)

        mapCanvas = canvasTmp

        console.log("Map cached ✔")
    }

    function screenToMap(canvas, screenX, screenY){

        const res = lastMap.info.resolution

        const scale = pixelsPerMeter * res

        const width = lastMap.info.width
        const height = lastMap.info.height

        const mapWidthPx = width * scale
        const mapHeightPx = height * scale

        const offsetX = (canvas.width - mapWidthPx) / 2
        const offsetY = (canvas.height - mapHeightPx) / 2

        const mapPixelX = (screenX - offsetX) / scale
        const mapPixelY = (mapHeightPx - (screenY - offsetY)) / scale

        const x = mapPixelX * res + lastMap.info.origin.position.x
        const y = mapPixelY * res + lastMap.info.origin.position.y

        return { x, y, theta: 0 }
    }

    function clearSelectionUI(){
        document.querySelectorAll(".stationItem").forEach(el => {
            el.classList.remove("active", "selected")
        })
    }

    function renderStations(stations){

         if (!Array.isArray(stations)) {
            console.error("renderStations got invalid data:", stations)
            return
        }
        const container = document.getElementById("stationsContainer")
        container.innerHTML = ""

        stations.forEach(station => {

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
                const current = stationStates.get(station.name)

                if (!current) {
                    for (let [k,v] of stationStates){
                        if (v === "goal") stationStates.delete(k)
                    }
                    stationStates.set(station.name, "goal")
                }
                else if (current === "goal") {

                    stationStates.set(station.name, "waypoint")

                    //ADD TO ORDERED LIST
                    orderedWaypoints.push({
                        type: "station",
                        name: station.name
                    })
                }
                else {
                    stationStates.delete(station.name)

                    // REMOVE FROM ORDERED LIST
                    orderedWaypoints = orderedWaypoints.filter(w =>
                        !(w.type === "station" && w.name === station.name)
                    )
                }

                renderStations(stations)
                drawScene()
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

        // center map on canvas
        const offsetX = (canvas.width - mapWidthPx) / 2
        const offsetY = (canvas.height - mapHeightPx) / 2

        // ---- DRAW MAP ----
        if (!mapCanvas) return

        ctx.save()

        ctx.translate(offsetX, offsetY + mapHeightPx)
        ctx.scale(scale, -scale)

        ctx.imageSmoothingEnabled = false
        ctx.drawImage(mapCanvas, 0, 0)

        ctx.restore()

        // ---- DRAW ROBOT ----
        if (robotReady && lastOdom){

            const x = (lastOdom.x - origin.x) / res * scale
            const y = (lastOdom.y - origin.y) / res * scale

            ctx.save()
            ctx.translate(offsetX + x, offsetY + mapHeightPx - y)
            ctx.rotate(lastOdom.theta || 0)

            ctx.drawImage(robotImg, -15, -15, 30, 30)

            ctx.restore()
        }

        // ---- DRAW STATIONS ----
        [...rosStations, ...localStations].forEach(st => {

            const x = (st.x - origin.x) / res * scale
            const y = (st.y - origin.y) / res * scale

            ctx.save()
            ctx.translate(offsetX + x, offsetY + mapHeightPx - y)

            const state = stationStates.get(st.name)

            if (state === "goal") ctx.fillStyle = "#10b981"       // green
            else if (state === "waypoint") ctx.fillStyle = "#f59e0b" // yellow
            else ctx.fillStyle = "#3b82f6"                        // blue (default)

            ctx.beginPath()
            ctx.arc(0, 0, 6, 0, Math.PI * 2)
            ctx.fill()

            // label
            ctx.fillStyle = "#ffffff"
            ctx.font = "10px sans-serif"
            ctx.fillText(st.name, 8, 3)

            ctx.restore()
        })
        // ---- DRAW WAYPOINTS ----
        orderedWaypoints.forEach((wp, i) => {

            let x, y

            if (wp.type === "map") {
                x = (wp.x - origin.x) / res * scale
                y = (wp.y - origin.y) / res * scale
            }

            if (wp.type === "station") {
                const s = rosStations.find(st => st.name === wp.name)
                if (!s) return

                x = (s.x - origin.x) / res * scale
                y = (s.y - origin.y) / res * scale
            }

            ctx.save()
            ctx.translate(offsetX + x, offsetY + mapHeightPx - y)

            ctx.fillStyle = "#f59e0b"
            ctx.beginPath()
            ctx.arc(0, 0, 5, 0, Math.PI * 2)
            ctx.fill()

            // index number (correct order)
            ctx.fillStyle = "#fff"
            ctx.fillText(i + 1, 8, 3)

            ctx.restore()
        });
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
        if (dy > 0) {
            linear  = -linear
            angular = -angular
        }

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
        stationStates.clear()

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
}
