<script>
  let { guidance } = $props();
</script>

{#if guidance.type === "none"}
  <div class="no-guidance">No Guidance</div>
{:else}
  <div class="guidance-header">{guidance.type}</div>
  <div class="guidance-grid">
    <div class="gauge-item">
      <div class="gauge-label">Range</div>
      <div class="gauge-value">{guidance.range.toFixed(0)}m</div>
    </div>
    <div class="gauge-item">
      <div class="gauge-label">H</div>
      <div class="gauge-value">{guidance.rangeHor.toFixed(0)}m</div>
    </div>
    <div class="gauge-item">
      <div class="gauge-label">V</div>
      <div class="gauge-value">{guidance.rangeVer.toFixed(0)}m</div>
    </div>
    <div class="gauge-item">
      <div class="gauge-label">Lat</div>
      <div class="gauge-value">{guidance.accelLat.toFixed(2)}g</div>
    </div>
    <div class="gauge-item">
      <div class="gauge-label">Ver</div>
      <div class="gauge-value">{guidance.accelVer.toFixed(2)}g</div>
    </div>
    <div class="gauge-item">
      <div class="gauge-label">TTG</div>
      <div class="gauge-value">{guidance.ttg.toFixed(1)}s</div>
    </div>
    <div class="gauge-item">
      <div class="gauge-label">ZEM</div>
      <div class="gauge-value">{guidance.zem.toFixed(0)}m</div>
    </div>
  </div>
{/if}

<style>
  .guidance-header {
    text-align: center;
    font-weight: bold;
    color: #4caf50;
    margin-bottom: 8px;
    padding: 4px;
    background: rgba(76, 175, 80, 0.1);
    border-radius: 4px;
    text-transform: uppercase;
    font-size: 12px;
  }

  .guidance-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 6px;
  }

  .gauge-item {
    background: rgba(0, 0, 0, 0.2);
    padding: 6px;
    border-radius: 4px;
    text-align: center;
  }

  .gauge-label {
    font-weight: bold;
    font-size: 10px;
    color: #999;
    margin-bottom: 2px;
  }

  .gauge-value {
    font-family: monospace;
    font-size: 13px;
    color: #fff;
  }

  .no-guidance {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 80px;
    color: #666;
    font-size: 14px;
  }

  @media (min-width: 768px) {
    .guidance-grid {
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
    }

    .gauge-item {
      padding: 8px;
    }

    .gauge-label {
      font-size: 11px;
    }

    .gauge-value {
      font-size: 14px;
    }
  }

  @media (min-width: 1024px) {
    .guidance-grid {
      grid-template-columns: 1fr;
    }
  }
</style>
