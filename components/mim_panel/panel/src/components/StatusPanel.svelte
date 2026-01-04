<script>
  export let status = {};
</script>

<div class="status-item">
  <span class={`status-indicator ${status.connected ? "ready" : "not-ready"}`}
  ></span>
  Connected
</div>

<div class="status-item">
  <span class={`status-indicator ${status.engaging ? "engaging" : "idle"}`}
  ></span>
  Engaging
</div>

<div class="status-item">
  <span class={`status-indicator ${status.targetReady ? "ready" : "not-ready"}`}
  ></span>
  Target Ready
</div>

<div class="status-item">
  <span
    class={`status-indicator ${status.interceptorReady ? "ready" : "not-ready"}`}
  ></span>
  Interceptor Ready
  <div class="status-subitem">
    Source: {status.interceptorSource}
  </div>
</div>

<style>
  .status-item {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 12px;
    border-bottom: 1px solid rgba(255, 255, 255, 0.1);
  }

  .status-indicator {
    display: inline-block;
    width: 12px;
    height: 12px;
    border-radius: 50%;
    margin-right: 8px;
  }

  .status-subitem {
    margin-left: auto;
    font-size: 0.9em;
    opacity: 0.8;
  }

  .ready {
    background-color: #4caf50;
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
    0% {
      opacity: 1;
    }
    50% {
      opacity: 0.6;
    }
    100% {
      opacity: 1;
    }
  }
</style>
