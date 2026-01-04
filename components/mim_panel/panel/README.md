# CRSF-MIM Web Control Panel

A web-based control panel for the CRSF Man-In-The-Middle system.

## Features

- **Real-time Status**: Connection status, engage/disengage, target/interceptor readiness
- **Interactive Map**: Shows target and interceptor positions with markers
- **Guidance Display**: Range, TTG, ZEM, and acceleration data
- **Full Control**: Engage/disengage button and complete settings management
- **Responsive Design**: Works on desktop and tablet devices

## Architecture

- **Backend**: ESP32 HTTP server + WebSocket for real-time updates
- **Frontend**: Svelte + Vite, lightweight and reactive
- **Protocol**: JSON over WebSocket for bidirectional communication

## Building the Frontend

### Prerequisites
- Node.js (v18+)
- npm

### Development
```bash
# Install dependencies
npm install

# Start development server
npm run dev

# Build for production
npm run build
```

### ESP32 Integration
The panel is automatically embedded into the ESP32 firmware when built with `CONFIG_MIM_PANEL_ENABLED=y`.

After flashing the firmware:
1. Connect to the same network as your development machine
2. Access the panel at `http://[ESP32_IP]`

## API Protocol

### State Update (ESP32 → Browser)
```json
{
  "type": "state",
  "ts": 1704200000000,
  "status": {
    "connected": true,
    "engaging": false,
    "targetReady": true,
    "interceptorReady": true,
    "interceptorSource": "skymap"
  },
  "target": {
    "lat": 50.4501,
    "lon": 30.5234,
    "alt": 150.5,
    "velN": 25.0,
    "velE": -10.0,
    "velU": 2.0,
    "ageMs": 120
  },
  "interceptor": {
    "lat": 50.4480,
    "lon": 30.5200,
    "alt": 145.0,
    "velN": 30.0,
    "velE": -5.0,
    "velU": 1.5,
    "ageMs": 80
  },
  "guidance": {
    "type": "pronav",
    "range": 451,
    "rangeHor": 450,
    "rangeVer": 5,
    "accelLat": 2.5,
    "accelVer": 0.3,
    "ttg": 12.5,
    "zem": 8.2
  },
  "network": {
    "mimIp": "192.168.1.100",
    "skymapIp": "192.168.1.50"
  },
  "settings": { ... }
}
```

### Commands (Browser → ESP32)
```json
// Engage/Disengage
{
  "type": "cmd",
  "action": "engage",
  "value": true
}

// Update Settings
{
  "type": "settings",
  "nav": {
    "N": 3.5,
    "maxRollDeg": 45,
    "attackAngleDeg": 20,
    "attackFactor": 1.0
  },
  "pitcher": {
    "kp": 0.1,
    "ki": 0.005,
    "kd": 0.02,
    "maxRate": 0.5,
    "inverted": false
  }
}

// Save Settings
{
  "type": "cmd",
  "action": "save_settings"
}
```

## Files Structure

```
panel/
├── src/
│   ├── App.svelte              # Main application
│   ├── components/
│   │   ├── StatusPanel.svelte    # Connection/Engage status
│   │   ├── MapView.svelte       # Leaflet map with markers
│   │   ├── GuidancePanel.svelte  # Gauges and guidance data
│   │   ├── ControlPanel.svelte   # Engage button
│   │   └── SettingsPanel.svelte # Settings management
│   └── lib/
│       └── websocket.js        # WebSocket client
├── public/
│   └── index.html             # Main HTML with layout
├── package.json
└── vite.config.js
```

## Components

- **StatusPanel**: Shows connection status with colored indicators
- **MapView**: Interactive map using Leaflet.js
- **GuidancePanel**: Displays all guidance parameters as gauges
- **ControlPanel**: Simple engage/disengage button
- **SettingsPanel**: Forms for all navigation and controller settings