<script>
  import { onMount } from "svelte";
  import StatusPanel from "./components/StatusPanel.svelte";
  import MapView from "./components/MapView.svelte";
  import GuidancePanel from "./components/GuidancePanel.svelte";
  import ControlPanel from "./components/ControlPanel.svelte";
  import SettingsPanel from "./components/SettingsPanel.svelte";

  let ws = null;
  let state = {
    status: {
      connected: false,
      engaging: false,
      targetReady: false,
      interceptorReady: false,
    },
    target: null,
    interceptor: null,
    guidance: {
      type: "none",
      range: 0,
      rangeHor: 0,
      rangeVer: 0,
      accelLat: 0,
      accelVer: 0,
      ttg: 0,
      zem: 0,
    },
    network: { mimIp: "0.0.0.0", skymapIp: "0.0.0.0" },
    settings: {
      nav: { N: 3, maxRollDeg: 45, attackAngleDeg: 20, attackFactor: 1 },
      pitcher: { kp: 0.1, ki: 0.005, kd: 0.02, maxRate: 0.5, inverted: false },
    },
  };

  function connect() {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const url = `${protocol}//${window.location.host}/ws`;

    ws = new WebSocket(url);

    ws.onopen = () => {
      console.log("WebSocket connected");
    };

    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);

        if (message.type === "state") {
          state = {
            ...state,
            ...message,
            timestamp: Date.now(),
          };

          // Update all components with new state
        }

        if (message.type === "settings") {
          // Settings confirmed, no action needed
        }
      } catch (error) {
        console.error("Failed to parse message:", error);
      }
    };

    ws.onclose = () => {
      console.log("WebSocket disconnected");
      ws = null;

      // Show disconnected state
      state.status.connected = false;
    };

    ws.onerror = (error) => {
      console.error("WebSocket error:", error);
      ws = null;
    };
  }

  function sendCommand(type, data = {}) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      const command = { type, ...data, timestamp: Date.now() };
      ws.send(JSON.stringify(command));
    }
  }

  onMount(() => {
    connect();

    // Reconnect on visibility change
    const handleVisibilityChange = () => {
      if (!document.hidden && (!ws || ws.readyState !== WebSocket.OPEN)) {
        connect();
      }
    };

    document.addEventListener("visibilitychange", handleVisibilityChange);

    return () => {
      document.removeEventListener("visibilitychange", handleVisibilityChange);
      if (ws) {
        ws.close();
      }
    };
  });
</script>

<div class="container">
  <div class="panel status">
    <StatusPanel status={state.status} />
  </div>

  <div class="panel map">
    <MapView target={state.target} interceptor={state.interceptor} />
  </div>

  <div class="panel guidance">
    <GuidancePanel guidance={state.guidance} />
  </div>

  <div class="panel control">
    <ControlPanel
      engaging={state.status.engaging}
      onEngage={() =>
        sendCommand("cmd", { action: "engage", value: !state.status.engaging })}
    />
  </div>

  <div class="panel settings">
    <SettingsPanel
      settings={state.settings}
      onSettingsChange={(settings) => sendCommand("settings", settings)}
    />
  </div>
</div>
