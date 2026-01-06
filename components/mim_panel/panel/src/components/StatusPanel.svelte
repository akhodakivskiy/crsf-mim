<script>
  let { status = {} } = $props();
</script>

<div class="status-grid">
  <div class="status-item">
    <span class={`status-indicator ${status.connected ? "ready" : "not-ready"}`}
    ></span>
    <span class="status-label">Connected</span>
  </div>

  <div class="status-item">
    <span class={`status-indicator ${status.engaging ? "engaging" : "idle"}`}
    ></span>
    <span class="status-label">Engaging</span>
  </div>

  <div class="status-item">
    <span
      class={`status-indicator ${status.targetReady ? "ready" : "not-ready"}`}
    ></span>
    <span class="status-label">Target</span>
  </div>

  <div class="status-item">
    <div class="status-indicator-row">
      <span
        class={`status-indicator ${status.interceptorReady ? "ready" : "not-ready"}`}
      ></span>
      {#if status.interceptorSource}
        <span class="status-source">{status.interceptorSource}</span>
      {/if}
    </div>
    <span class="status-label">Interceptor</span>
  </div>
</div>

<style>
  .status-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 4px;
  }

  .status-item {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    padding: 6px 4px;
    background: rgba(0, 0, 0, 0.2);
    border-radius: 4px;
    font-size: 12px;
    min-width: 0;
    text-align: center;
  }

  .status-indicator-row {
    display: flex;
    align-items: center;
    gap: 3px;
    margin-bottom: 0px;
  }

  .status-indicator {
    display: inline-block;
    width: 10px;
    height: 10px;
    border-radius: 50%;
    flex-shrink: 0;
  }

  .status-label {
    white-space: nowrap;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    max-width: 100%;
  }

  .status-source {
    font-size: 8px;
    opacity: 0.7;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    max-width: 50px;
  }

  .ready {
    background-color: #4caf50;
    box-shadow: 0 0 4px #4caf50;
  }

  .not-ready {
    background-color: #f44336;
  }

  .idle {
    background-color: #9e9e9e;
  }

  .engaging {
    background: linear-gradient(135deg, #ff9800 0%, #ff5722 100%);
    animation: pulse 1.5s infinite;
  }

  @keyframes pulse {
    0%,
    100% {
      opacity: 1;
    }
    50% {
      opacity: 0.6;
    }
  }

  @media (min-width: 768px) {
    .status-grid {
      grid-template-columns: 1fr;
      gap: 8px;
    }

    .status-item {
      flex-direction: row;
      padding: 8px 10px;
      font-size: 13px;
      text-align: left;
      justify-content: flex-start;
    }

    .status-indicator-row {
      margin-bottom: 0;
      margin-right: 6px;
      gap: 0;
    }

    .status-indicator {
      width: 12px;
      height: 12px;
    }

    .status-label {
      flex: 1;
    }

    .status-source {
      font-size: 10px;
      margin-left: 4px;
      max-width: none;
    }
  }
</style>
