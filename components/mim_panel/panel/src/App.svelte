<script>
  import { onMount } from "svelte";
  import StatusPanel from "./components/StatusPanel.svelte";
  import MapView from "./components/MapView.svelte";
  import GuidancePanel from "./components/GuidancePanel.svelte";
  import ControlPanel from "./components/ControlPanel.svelte";
  import SettingsPanel from "./components/SettingsPanel.svelte";

  let ws = $state(null);
  let state = $state({
    status: {
      connected: false,
      engaging: false,
      targetReady: false,
      interceptorReady: false,
      interceptorSource: "none",
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
  });

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
          // Properly update state in Svelte 5
          if (message.status)
            state.status = { ...state.status, ...message.status };
          if (message.target) state.target = message.target;
          if (message.interceptor) state.interceptor = message.interceptor;
          if (message.guidance)
            state.guidance = { ...state.guidance, ...message.guidance };
          if (message.network)
            state.network = { ...state.network, ...message.network };
          if (message.settings)
            state.settings = { ...state.settings, ...message.settings };
          state.timestamp = Date.now();

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

<style>
  :global(body) {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Arial,
      sans-serif;
    margin: 0;
    padding: 0;
    background: #1a1a1a;
    color: #eee;
    font-size: 14px;
  }

  .container {
    max-width: 1400px;
    margin: 0 auto;
    padding: 8px;
    display: grid;
    grid-template-columns: 1fr;
    gap: 8px;
    grid-template-areas:
      "status"
      "control"
      "map"
      "guidance"
      "settings";
  }

  .panel {
    background: #2a2a2a;
    border-radius: 6px;
    padding: 12px;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
  }

  .panel.status {
    grid-area: status;
  }
  .panel.map {
    grid-area: map;
  }
  .panel.guidance {
    grid-area: guidance;
  }
  .panel.control {
    grid-area: control;
  }
  .panel.settings {
    grid-area: settings;
  }

  /* Tablet and up */
  @media (min-width: 768px) {
    .container {
      padding: 12px;
      gap: 12px;
      grid-template-columns: 280px 1fr;
      grid-template-areas:
        "status map"
        "control map"
        "guidance map"
        "settings settings";
    }

    .panel {
      padding: 16px;
    }
  }

  /* Desktop */
  @media (min-width: 1024px) {
    .container {
      padding: 16px;
      gap: 16px;
      grid-template-columns: 300px 1fr 320px;
      grid-template-areas:
        "status map guidance"
        "control map settings"
        "control map settings";
    }

    .panel {
      padding: 20px;
    }
  }
</style>
