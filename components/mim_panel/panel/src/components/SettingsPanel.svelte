<script>
  export let settings;
  export let onSettingsChange;

  console.log(settings);

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

<div class="settings-grid">
  <div class="setting-group">
    <h4>Navigation</h4>
    <div class="setting-row">
      <label for="id-n">N</label>
      <input
        id="id-n"
        type="number"
        value={settings.nav.N}
        on:input={(e) => handleNavChange("N", e.target.value)}
        step="0.1"
        min="1"
        max="10"
      />
    </div>
    <div class="setting-row">
      <label for="id-max-roll">Max Roll (deg)</label>
      <input
        id="id-max-roll"
        type="number"
        value={settings.nav.maxRollDeg}
        on:input={(e) => handleNavChange("maxRollDeg", e.target.value)}
        min="10"
        max="90"
      />
    </div>
    <div class="setting-row">
      <label for="id-aoa">Attack Angle (deg)</label>
      <input
        id="id-aoa"
        type="number"
        value={settings.nav.attackAngleDeg}
        on:input={(e) => handleNavChange("attackAngleDeg", e.target.value)}
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
        on:input={(e) => handleNavChange("attackFactor", e.target.value)}
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
        on:input={(e) => handlePitcherChange("kp", e.target.value)}
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
        on:input={(e) => handlePitcherChange("ki", e.target.value)}
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
        on:input={(e) => handlePitcherChange("kd", e.target.value)}
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
        on:input={(e) => handlePitcherChange("maxRate", e.target.value)}
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
        on:change={(e) => handlePitcherToggle("inverted", e.target.checked)}
      />
    </div>
  </div>

  <button class="save-button" on:click={handleSave}>Save Settings</button>
</div>

<style>
  .settings-grid {
    display: grid;
    grid-template-columns: 150px 1fr;
    gap: 10px;
    padding: 10px;
  }

  .setting-group {
    background: rgba(0, 0, 0, 0.1);
    padding: 10px;
    border-radius: 4px;
  }

  .setting-group h4 {
    margin: 0 0 10px 0;
    color: #333;
  }

  .setting-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 8px;
  }

  .setting-row input {
    padding: 6px;
    border: 1px solid #555;
    border-radius: 3px;
    background: white;
  }

  .save-button {
    grid-column: span 2;
    padding: 12px;
    font-weight: bold;
    border: none;
    border-radius: 4px;
    cursor: pointer;
    background: #4caf50;
    color: white;
  }

  .save-button:hover {
    background: #45a049;
  }
</style>
