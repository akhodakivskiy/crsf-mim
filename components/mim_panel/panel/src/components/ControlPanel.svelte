<script>
  let { engaging, onEngage } = $props();

  function handleEngage() {
    onEngage(!engaging);
  }
</script>

<button
  class="engage-button"
  class:engage={engaging}
  onclick={handleEngage}
  disabled={!onEngage}
>
  {engaging ? "DISENGAGE" : "ENGAGE"}
</button>

<style>
  .engage-button {
    width: 100%;
    cursor: pointer;
    font-weight: bold;
    font-size: 16px;
    padding: 14px 20px;
    border: none;
    border-radius: 6px;
    transition: all 0.2s;
    background: #4caf50;
    color: white;
    text-transform: uppercase;
    letter-spacing: 1px;
    touch-action: manipulation;
    -webkit-tap-highlight-color: transparent;
  }

  .engage-button:active {
    transform: scale(0.98);
  }

  .engage-button.engage {
    background: linear-gradient(135deg, #f44336 0%, #d32f2f 100%);
    box-shadow: 0 0 0 8px rgba(244, 67, 54, 0.3);
  }

  .engage-button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  @media (min-width: 768px) {
    .engage-button {
      font-size: 18px;
      padding: 16px 30px;
    }

    .engage-button:hover:not(:disabled) {
      transform: translateY(-2px);
      box-shadow: 0 6px 12px rgba(0, 0, 0, 0.3);
    }

    .engage-button.engage {
      box-shadow: 0 0 0 10px rgba(244, 67, 54, 0.5);
    }
  }
</style>
