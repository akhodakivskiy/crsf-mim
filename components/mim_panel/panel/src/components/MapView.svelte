<script>
  export let target;
  export let interceptor;

  import { onMount } from "svelte";
  import L from "leaflet";
  import "leaflet/dist/leaflet.css"; // Add this line

  let map;
  let targetMarker;
  let interceptorMarker;

  onMount(() => {
    // Initialize map centered on interceptor if available
    const center = interceptor
      ? [interceptor.lat, interceptor.lon]
      : [50.45, 30.52];

    map = L.map("map-view").setView(center, 13);

    // Add tiles
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: "&copy; OpenStreetMap contributors",
      maxZoom: 19,
    }).addTo(map);

    // Update markers when props change
    updateMarkers();
  });

  // Fix: Use $: reactive statement outside onMount
  $: if (map) {
    updateMarkers();
  }

  function updateMarkers() {
    if (!map) return;

    // Remove old markers
    if (targetMarker) map.removeLayer(targetMarker);
    if (interceptorMarker) map.removeLayer(interceptorMarker);

    // Add target marker
    if (target) {
      const targetIcon = L.divIcon({
        html: '<div style="background: #ff4444; color: white; border-radius: 50%; width: 20px; height: 20px; text-align: center; line-height: 20px;">✈</div>',
        className: "target-marker",
        iconSize: [20, 20],
      });

      targetMarker = L.marker([target.lat, target.lon], { icon: targetIcon })
        .addTo(map)
        .bindPopup(
          `Target<br>Lat: ${target.lat.toFixed(6)}<br>Lon: ${target.lon.toFixed(6)}<br>Alt: ${(target.alt || 0).toFixed(1)}m`,
        );
    }

    // Add interceptor marker
    if (interceptor) {
      const interceptorIcon = L.divIcon({
        html: '<div style="background: #4444ff; color: white; border-radius: 50%; width: 20px; height: 20px; text-align: center; line-height: 20px;">◆</div>',
        className: "interceptor-marker",
        iconSize: [20, 20],
      });

      interceptorMarker = L.marker([interceptor.lat, interceptor.lon], {
        icon: interceptorIcon,
      })
        .addTo(map)
        .bindPopup(
          `Interceptor<br>Lat: ${interceptor.lat.toFixed(6)}<br>Lon: ${interceptor.lon.toFixed(6)}<br>Alt: ${(interceptor.alt || 0).toFixed(1)}m<br>Vel: ${Math.sqrt(interceptor.vel_north ** 2 + interceptor.vel_east ** 2).toFixed(1)}m/s`,
        );
    }
  }
</script>

<div id="map-view"></div>

<style>
  #map-view {
    width: 100%;
    height: 400px; /* Add explicit height */
    position: relative;
    z-index: 1;
  }

  :global(.target-marker) {
    background: #ff4444;
    border: 2px solid #cc0000;
    border-radius: 50%;
  }

  :global(.interceptor-marker) {
    background: #4444ff;
    border: 2px solid #0000cc;
    border-radius: 50%;
  }

  :global(.leaflet-popup-content-wrapper) {
    font-family: monospace;
    font-size: 12px;
  }
</style>
