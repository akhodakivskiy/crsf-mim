<script>
  export let guidance;
</script>

{#if guidance.type === "none"}
  <div class="no-guidance">No Guidance</div>
{:else}
  <div class="guidance-data">
    <div class="gauge-row">
      <div class="gauge-label">Type</div>
      <div class="gauge-value">{guidance.type}</div>
    </div>
    <div class="gauge-row">
      <div class="gauge-label">Range</div>
      <div class="gauge-value">{guidance.range.toFixed(1)}m</div>
    </div>
    <div class="gauge-row">
      <div class="gauge-label">Range H</div>
      <div class="gauge-value">{guidance.rangeHor.toFixed(1)}m</div>
    </div>
    <div class="gauge-row">
      <div class="gauge-label">Range V</div>
      <div class="gauge-value">{guidance.rangeVer.toFixed(1)}m</div>
    </div>
    <div class="gauge-row">
      <div class="gauge-label">Accel Lat</div>
      <div class="gauge-value">{guidance.accelLat.toFixed(2)}g</div>
    </div>
    <div class="gauge-row">
      <div class="gauge-label">Accel Ver</div>
      <div class="gauge-value">{guidance.accelVer.toFixed(2)}g</div>
    </div>
    <div class="gauge-row">
      <div class="gauge-label">TTG</div>
      <div class="gauge-value">{guidance.ttg.toFixed(1)}s</div>
    </div>
    <div class="gauge-row">
      <div class="gauge-label">ZEM</div>
      <div class="gauge-value">{guidance.zem.toFixed(1)}m</div>
    </div>
  </div>
{/if}

<style>
  .guidance-data {
    display: grid;
    gap: 10px;
  }

  .gauge-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 4px 8px;
    background: rgba(0, 0, 0, 0.1);
  }

  .gauge-label {
    font-weight: bold;
    font-size: 12px;
  }

  .gauge-value {
    font-family: monospace;
    font-size: 14px;
    text-align: right;
  }

  .no-guidance {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 100px;
    color: #666;
  }
</style>
