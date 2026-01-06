<script>
  let { settings, onSettingsChange } = $props();

  function handleNavChange(key, value) {
    onSettingsChange("nav", { ...settings.nav, [key]: parseFloat(value) });
  }

  function handlePitcherChange(key, value) {
    onSettingsChange("pitcher", {
      ...settings.pitcher,
      [key]: parseFloat(value),
    });
  }

  function handlePitcherToggle(key, checked) {
    onSettingsChange("pitcher", { ...settings.pitcher, [key]: checked });
  }

  function handleSave() {
    // Send save command
    fetch("/api/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
    }).then(() => {
      console.log("Settings saved");
    });
  }
</script>

<div class="settings-container">
  <div class="setting-group">
    <h4>Navigation</h4>
    <div class="setting-row">
      <label for="id-n">N</label>
      <input
        id="id-n"
        type="number"
        value={settings.nav.N}
        oninput={(e) => handleNavChange("N", e.target.value)}
        step="0.1"
        min="1"
        max="10"
      />
    </div>
    <div class="setting-row">
      <label for="id-max-roll">Max Roll</label>
      <input
        id="id-max-roll"
        type="number"
        value={settings.nav.maxRollDeg}
        oninput={(e) => handleNavChange("maxRollDeg", e.target.value)}
        min="10"
        max="90"
      />
    </div>
    <div class="setting-row">
      <label for="id-aoa">Attack Angle</label>
      <input
        id="id-aoa"
        type="number"
        value={settings.nav.attackAngleDeg}
        oninput={(e) => handleNavChange("attackAngleDeg", e.target.value)}
        min="0"
        max="90"
      />
    </div>
    <div class="setting-row">
      <label for="id-atk-factor">Attack Factor</label>
      <input
        id="id-atk-factor"
        type="number"
        value={settings.nav.attackFactor}
        oninput={(e) => handleNavChange("attackFactor", e.target.value)}
        step="0.1"
        min="0.1"
        max="5"
      />
    </div>
  </div>

  <div class="setting-group">
    <h4>Pitch Controller</h4>
    <div class="setting-row">
      <label for="id-kp">Kp</label>
      <input
        id="id-kp"
        type="number"
        value={settings.pitcher.kp}
        oninput={(e) => handlePitcherChange("kp", e.target.value)}
        step="0.01"
        min="0"
        max="1"
      />
    </div>
    <div class="setting-row">
      <label for="id-ki">Ki</label>
      <input
        id="id-ki"
        type="number"
        value={settings.pitcher.ki}
        oninput={(e) => handlePitcherChange("ki", e.target.value)}
        step="0.001"
        min="0"
        max="0.1"
      />
    </div>
    <div class="setting-row">
      <label for="id-kd">Kd</label>
      <input
        id="id-kd"
        type="number"
        value={settings.pitcher.kd}
        oninput={(e) => handlePitcherChange("kd", e.target.value)}
        step="0.001"
        min="0"
        max="0.1"
      />
    </div>
    <div class="setting-row">
      <label for="id-max-rate">Max Rate</label>
      <input
        id="id-max-rate"
        type="number"
        value={settings.pitcher.maxRate}
        oninput={(e) => handlePitcherChange("maxRate", e.target.value)}
        step="0.1"
        min="0.1"
        max="2"
      />
    </div>
    <div class="setting-row">
      <label for="id-inverted">Inverted</label>
      <input
        id="id-inverted"
        type="checkbox"
        checked={settings.pitcher.inverted}
        onchange={(e) => handlePitcherToggle("inverted", e.target.checked)}
      />
    </div>
  </div>

  <button class="save-button" onclick={handleSave}>Save Settings</button>
</div>

<style>
  .settings-container {
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  .setting-group {
    background: rgba(0, 0, 0, 0.2);
    padding: 10px;
    border-radius: 4px;
  }

  .setting-group h4 {
    margin: 0 0 8px 0;
    color: #4caf50;
    font-size: 13px;
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }

  .setting-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 6px;
    font-size: 12px;
  }

  .setting-row label {
    flex: 1;
    color: #ccc;
  }

  .setting-row input[type="number"] {
    width: 80px;
    padding: 4px 6px;
    border: 1px solid #444;
    border-radius: 3px;
    background: #1a1a1a;
    color: #fff;
    font-size: 12px;
  }

  .setting-row input[type="checkbox"] {
    width: 18px;
    height: 18px;
    cursor: pointer;
  }

  .save-button {
    padding: 12px;
    font-weight: bold;
    font-size: 14px;
    border: none;
    border-radius: 4px;
    cursor: pointer;
    background: #4caf50;
    color: white;
    text-transform: uppercase;
    letter-spacing: 1px;
    transition: all 0.2s;
    touch-action: manipulation;
  }

  .save-button:active {
    transform: scale(0.98);
  }

  @media (min-width: 768px) {
    .settings-container {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
    }

    .save-button {
      grid-column: span 2;
    }

    .save-button:hover {
      background: #45a049;
      transform: translateY(-1px);
    }
  }

  @media (min-width: 1024px) {
    .settings-container {
      grid-template-columns: 1fr;
    }

    .save-button {
      grid-column: span 1;
    }
  }
</style>
