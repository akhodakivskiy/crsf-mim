<script>
  export let engaging;
  export let onEngage;

  function handleEngage() {
    onEngage(!engaging);
  }
</script>

<button
  class="engage-button"
  class:engage={engaging}
  on:click={handleEngage}
  disabled={!onEngage}
>
  {engaging ? "DISENGAGE" : "ENGAGE"}
</button>

<style>
  .engage-button {
    cursor: pointer;
    font-weight: bold;
    font-size: 18px;
    padding: 15px 30px;
    border: none;
    border-radius: 6px;
    transition: all 0.3s;
  }

  .engage-button:hover {
    transform: translateY(-2px);
    box-shadow: 0 6px 12px rgba(0, 0, 0, 0.2);
  }

  .engage-button.engage {
    background: linear-gradient(135deg, #f44336 0%, #f44336 100%);
    color: white;
    box-shadow: 0 0 0 10px rgba(244, 67, 54, 0.5);
  }
</style>
